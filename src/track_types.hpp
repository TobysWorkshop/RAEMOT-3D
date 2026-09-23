#pragma once
#include <cstdint>
#include <cstddef>

// ---- THREAD C 3D TRACK TYPE DEFINITIONS ---- //

// raw state as it comes off of thread C from the cameras
enum class CamId : uint8_t { A = 0, B = 1 };

struct RawState {
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double tg = 0.0; // global synced time
    int32_t track_id = -1;
    CamId cam_id = CamId::A;
};

// triangulated 3D state (what gets written out of thread C to file)
struct State3D {
    double tg = 0.0;
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
};

// track lifecyle indicators
enum class TrackStatus : uint8_t {
    INACTIVE = 0,   // slot is free
    WAITING,        // has one camera side (A) only, looking for a partner (from B)
    CANDIDATE,      // has both sides, buffering, not yet trusted
    VALIDATED       // confirmed pair, streaming straight to the file writer
};

// pack the cam_id and track_id into one 64-bit look-up key:
// cam_id in the high bits, track_id in the low 32
inline uint64_t make_key(CamId cam_id, int32_t track_id) {
    return (static_cast<uint64_t>(cam_id) << 32) | static_cast<uint64_t>(static_cast<uint32_t>(track_id));
}