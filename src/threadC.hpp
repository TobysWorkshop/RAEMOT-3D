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
    
    std::pair<bool, double> passes_match_check(const RawState& a, const RawState& b);
    
}