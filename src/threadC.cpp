// include companion file
#include "threadC.hpp"

// other local file includes
#include "track_update_queue.hpp"
#include "threadC_params.h"
#include "registry.hpp"

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

        // --- Resistry pointer -- //
        size_t POOL_SIZE = 128;
        Registry<POOL_SIZE> reg;

        // --- Evaluation counters - //
        int waiting_eval_counter = 0;
        const int WAITING_EVAL_FREQ = 20; // number of updates between each evaluation of the WAITING tracks
        const double WAITING_MAX_AGE = 1; // WAITING A tracks over this age (in seconds) will be removed


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

    // Epipolar and velocity projection check between an A and B state pair
    // Returns {success, cost}
    std::pair<bool, double> passes_match_check(const RawState& a, const RawState& b) {
        // NEEDS IMPLEMENTING HERE!!
        return {true, -1.0}; // needs changing obviously
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

    void process_track_update(const RawState& state) {
        // look up this camera_id and track_id combo in the registry
        uint16_t idx = reg.find(state.cam_id, state.track_id);

        // ---- BRANCH 1: NEW COMBO WE'VE NEVER SEEN ---- //
        if (idx == INVALID_SLOT) {

            // -- If this update was from Camera A: -- //
            if (state.cam_id == CamId::A) {
                // create a new WAITING slot for it
                idx = reg.create_waiting(state.cam_id, state.track_id);
                
                // if the registry is actually full, alert this problem to the user - will need to be fixed!
                if (idx == INVALID_SLOT) {
                    std::cerr << "[Thread C] OVERFLOW! Registry pool is exhausted! -- raise POOL_SIZE!!\n";
                    return;
                }

                // access the new track slot that the registry assigned for us
                Track& track = reg.get(idx);

                // update the track's initial A state info
                track.last_a = state;
                track.last_a_time = state.tg;
            } else {
            // -- If this update was from Camera B: -- //

                // New B: scan all unassigned WAITING A-only tracks for a match
                
                // And HERE we begin the age-old courting process as a B track speed dates a selection of potential A track suitors
                uint16_t best = INVALID_SLOT;
                double best_compatibility_score = 1e8; // large starting value
                reg.for_each_active([&](uint16_t suitor_idx, Track& suitor) {
                    // make sure the suitor isn't itself (you can't date yourself)
                    if (suitor_idx == idx) return;
                    // make sure the suitor has status: single
                    if (suitor.status != TrackStatus::WAITING) return;
                    // make sure the suitor is an A, not a B, and also double check it didn't somehow sneak in a partner since that previous check (monogamous bee tracks, thank you very much)
                    if (!suitor.has_a || suitor.has_b) return;
                    // make sure the suitor isn't too old for it (appropriate age gaps are all the rage. They're also very wise)
                    if ((state.tg - suitor.last_a_time) > WAITING_MAX_AGE){
                        reg.close(suitor_idx); // remove this suitor from the lineup entirely
                        return;
                    }
                    // make sure it didn't just date this suitor, and therefore is in a cooldown period (no rebounds back to exes thank you very much!)
                    if (suitor.has_cooldown && suitor.cooldown_key == track.key_b) return;
                    // Let's do a compatibility test!
                    auto [suitable, compatibility_score] = passes_match_check(suitor.last_a, state);
                    // If these two are suitable, and have a better compatibility score than previous suitors, take a note and update their standards before moving on
                    if (suitable && compatibility_score < best_compatibility_score) {
                        best = suitor_idx;
                        best_compatibility_score = compatibility_score;
                    }
                });
                // See if anyone in the lineup was actually suitable
                if (best != INVALID_SLOT) {
                    // Pair them up!
                    reg.merge_into_candidate(best, state.track_id, state);
                }
                // If it wasn't matched, the data is dropped and we move on to the next one coming in
            }

            return;
        } // END BRANCH 1 (new combo we've never seen)

        // ---- BRANCH 2: EXISTING COMBO WE'VE SEEN BEFORE ---- //
        // we need to update the relevant side, do CANDIDATE buffering or VALIDATED stream to file,
        // and update counters for pass/fail vailidation steps

        // access the track that the registry assigned to this existing combo
        Track& track = reg.get(idx);

        // update the relevant side's last-seen state data
        if (state.cam_id == CamId::A) {
            // Update was from Camera A
            track.last_a = state;
            track.last_a_time = state.tg;
        } else {
            // Update was from Camera B
            track.last_b = state;
            track.last_b_time = state.tg;
        }

        // -- Track is CANDIDATE -- //
        if (track.status == TrackStatus::CANDIDATE) {
            //...
        } else if (track.status == TrackStatus::VALIDATED) {
        // -- Track is VALIDATED -- //
            //...
        }
        // ... etc...

        // END BRANCH 2 (existing combo we've seen before)

    }

    void teardown() {

    }
} // namespace threadC