#pragma once

#include "frame_queue.hpp"
#include "sepia.hpp"
#include "track_update_queue.hpp"
#include "threadC_params.h"

#include <string>
#include <vector>

namespace threadC {

    bool setup(const std::string& config_name);

    void process_track_update(const RawState& msg);

    void teardown();
    
}