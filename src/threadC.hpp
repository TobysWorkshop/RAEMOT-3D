#pragma once

#include "frame_queue.hpp"
#include "sepia.hpp"
#include "track_update_queue.hpp"
#include "threadC_params.h"

#include <string>
#include <vector>

namespace threadC {

    bool setup(const std::string& config_name);

    void process_track_update(const TrackUpdateMsg& msg);

    void teardown();

    // list to store rolling recent unallocated A cam IDs
    struct A_cam_unallocated_list {
        uint64_t A_id;
        double tg;
        bool isValid = false; // tracks if a slot has been removed early
    };
    void recalculateMRU() {}
    void insert_unallocated_A_id(uint64_t A_id, double tg) {}
    int getMostRecentIndex() {}
    const A_cam_unallocated_list* getEntryAt(size_t index) {}
    bool removeEarlyByIndex(size_t index) {}
    void remove_expired_unallocated_A_ids(double tg_now) {}
    

}