// include companion file
#include "threadC.hpp"

// other local file includes
#include "track_update_queue.hpp"
#include "threadC_params.h"

// other includes
#include <cstdint>
#include <iostream>
#include "yaml-cpp/yaml.h"      
#include <eigen3/Eigen/Dense>   
#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>           
#include <fstream>               
#include <iomanip>
#include <memory>
#include <math.h>
#include <opencv2/core/core.hpp>     
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp>  
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <unistd.h>

namespace threadC {

    namespace { // global variables for this namespace


        // --- A cam recent unallocated list ---
        const int A_un_buffer_max_size = 20;
        const double max_unallocated_A_age = 1.0; // in seconds

        std::array<A_cam_unallocated_list, 20> A_un_m_buffer{}; // currently 20 slots long. change later if needed
        size_t A_un_m_mruIndex = 0; // tracks the most recent valid index
        size_t A_un_m_activeCount = 0; // current number of active, unexpired entries


    } // end namespace for global variables

    namespace {
        // Resolves the directory containing the currently-running executable, by
        // reading the Linux-specific /proc/self/exe self-symlink. This is
        // independent of the process's current working directory (unlike a plain
        // relative path), and independent of argv[0] (which isn't always a
        // resolvable path - e.g. when found via $PATH).
        //
        // Assumes configs/ sits next to the directory the built binary lives in
        // (src/../configs, per the project's current layout). If you later move
        // the build output somewhere else (e.g. a build/ directory), adjust the
        // ".." below - or better, make the configs directory itself overridable
        // (see the comment in setup() below).
        std::filesystem::path executable_directory() {
            char buffer[4096];
            const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (length <= 0) {
                throw std::runtime_error("could not resolve executable path via /proc/self/exe");
            }
            buffer[length] = '\0';
            return std::filesystem::path(buffer).parent_path();
        }
    }

    // extra functions go here (void, etc.)

    // --- A unallocated IDs list ---

    // helper function to recalculate the MRU on early deletion
    void recalculateMRU() {
        double newest_tg = -1.0; // Assumes timestamps are non-negative
        bool found = false;

        for (size_t i = 0; i < A_un_buffer_max_size; ++i) {
            if (A_un_m_buffer[i].isValid && A_un_m_buffer[i].tg > newest_tg) {
                newest_tg = A_un_m_buffer[i].tg;
                A_un_m_mruIndex = i;
                found = true;
            }
        }
        if (!found) {
            A_un_m_mruIndex = 0;
        }
    }

    // insert a new unallocated A entry into the list - always moves forward and overwrite oldest entry if full
    void insert_unallocated_A_id(uint64_t A_id, double tg) {
        size_t target_slot = 0;
        bool found_empty_slot = false;
        double oldest_tg = std::numeric_limits<double>::max();

        // Single cache-friendly sweep of 20 elements
        for (size_t i = 0; i < A_un_buffer_max_size; ++i) {
            if (!A_un_m_buffer[i].isValid) {
                target_slot = i;
                found_empty_slot = true;
                break; // Instantly claim the first early-deleted or expired slot
            }
            if (A_un_m_buffer[i].tg < oldest_tg) {
                oldest_tg = A_un_m_buffer[i].tg;
                target_slot = i; // Track oldest active item in case buffer is totally packed
            }
        }

        // If we are forced to overwrite an active oldest item, reduce active count
        if (!found_empty_slot && A_un_m_buffer[target_slot].isValid) {
            A_un_m_activeCount--;
        }

        // Write the data into the safe slot
        A_un_m_buffer[target_slot] = A_cam_unallocated_list{
            .A_id = A_id, 
            .tg = tg,
            .isValid = true
        };

        A_un_m_mruIndex = target_slot; 
        A_un_m_activeCount++;
    }
    // A unallocated list MRU lookup: the newest entry is always right behind the tail pointer
    int getMostRecentIndex() {
        if (A_un_m_activeCount == 0 || !A_un_m_buffer[A_un_m_mruIndex].isValid) {
            return -1;
        }
        return static_cast<int>(A_un_m_mruIndex);
    }
    // helper function to access the buffer safely using an index
    const A_cam_unallocated_list* getEntryAt(size_t index) {
        if (index >= A_un_buffer_max_size || !A_un_m_buffer[index].isValid) {
            return nullptr;
        }
        return &A_un_m_buffer[index];
    }
    // Early deletion - remove an item from the list if it gets moved to the candidate allocation side of processing
    bool removeEarlyByIndex(size_t index) {
        if (index >= A_un_buffer_max_size || !A_un_m_buffer[index].isValid) {
            return false; // slot is already empty or index is invalid
        }

        A_un_m_buffer[index].isValid = false; // mark this slot as dead
        A_un_m_activeCount--;

        // if we deleted the MRU item early, we must find the next newest active slot for consistency
        if (index == A_un_m_mruIndex) {
            recalculateMRU();
        }
        return true;
    }
    // A unallocated list expiry check: because it's always sorted, the oldest is ALWAYS at A_un_m_head
    void remove_expired_unallocated_A_ids(double tg_now) {
        if (A_un_m_activeCount == 0) return;

        double cutoff = tg_now - max_unallocated_A_age;
        bool mru_was_invalidated = false;

        for (size_t i = 0; i < A_un_buffer_max_size; ++i) {
            if (A_un_m_buffer[i].isValid && A_un_m_buffer[i].tg < cutoff) {
                A_un_m_buffer[i].isValid = false;
                A_un_m_activeCount--;
                if (i == A_un_m_mruIndex) { 
                    mru_was_invalidated = true; 
                }
            }
        }

        if (mru_was_invalidated) {
            recalculateMRU();
        }
    }

    //
    
    bool setup(const std::string &config_name) {
        // PARSE THE CONFIG //
        try {
            // configs/ is a sister directory of src/ (where this binary is built).
            // See executable_directory()'s comment above if you move the build output elsewhere.
            const std::filesystem::path file_config_path =
                executable_directory() / ".." / "configs" / (config_name + ".yaml");
            if (!std::filesystem::exists(file_config_path)) {
                throw std::runtime_error("config file not found: " + file_config_path.string());
            }
            params = loadThreadCParametersFromYAML(file_config_path.string());
        } catch (const std::exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
            return false; // Fail fast if config is missing/invalid
        }

        /// etc etc

        return true;
    }

    void process_track_update(const TrackUpdateMsg& msg) {

    }

    void teardown() {

    }
} // namespace threadC