#pragma once

#include "track_types.hpp"

#include <array>
#include <cstdint>
#include <cassert>

// How many 3D states a CANDIDATE track can buffer before it's either
// validated (buffer gets flushed to writer) or killed (buffer gets discarded)
constexpr size_t MAX_CANDIDATE_BUFFER = 8;

constexpr uint16_t INVALID_SLOT = 0xFFFF;

// ---- One Track Object ---- //
// Occupies one pool slot for its entire lifetime:
// INACTIVE -> WAITING -> CANDIDATE -> VALIDATED -> (reset) -> INACTIVE
// The object is reused across lifetimes rather than being destoyed and recreated.
struct alignas(64) Track {
    TrackStatus status = TrackStatus::INACTIVE;
    // ===== cache line 0: touched by every scan and every update =====
    bool has_a = false, has_b = false;
    // Cooldown: last partner key this track diverged from + expiry time,
    // so that it doesn't immediately re-match the same wrong id.
    bool has_cooldown = false;
    // bools used to gate evaluation only to new update pairs, not stale older updates
    bool fresh_a = false, fresh_b = false;
    // counters for candidate validation
    uint16_t pass_count = 0;
    uint16_t fail_count = 0;
    uint16_t buffer_count = 0;

    uint64_t cooldown_key = 0;
    double cooldown_until = 0.0;
    double last_a_time = 0.0;
    double last_b_time = 0.0;
    double status_since = 0.0;
    uint64_t key_a = 0;
    
    // ===== cache line 1: read by the WAITING scan =====
    RawState last_a{};
    uint64_t key_b = 0;
    // global id assigned at the moment of validation, used downstream for fiel output
    // -1 until validated
    int64_t global_id = -1;

    // ===== only touched once paired =====
    RawState last_b{};
    std::array<State3D, MAX_CANDIDATE_BUFFER> buffer{};
    

    // reset function to clear old furnature from previous track tenants in this slot
    void reset() { *this = Track{}; }

    // function to append a 3D state to the buffer. Returns true if added, false if already full
    bool push_buffer(const State3D& state) {
        if (buffer_count >= MAX_CANDIDATE_BUFFER) return false; // FULL

        buffer[buffer_count++] = state;
        return true; // STILL SPACE
    }
};

// ---- Fixed-size Track Pool ---- //
template <size_t POOL_SIZE>
class TrackPool {
public:
    TrackPool() {
        // Initial free list starts full - every slot available, in index order
        for (uint16_t i = 0; i < POOL_SIZE; ++i) {
            free_stack[i] = i;
        }
        free_top = POOL_SIZE;
    }

    // Returns INVALID_SLOT if the pool is exhausted -- if this happens, increase the POOL_SIZE
    uint16_t acquire() {
        if (free_top == 0) {
            return INVALID_SLOT;
        }
        uint16_t idx = free_stack[--free_top];
        slots[idx].status = TrackStatus::WAITING;
        return idx;
    }

    // Resets the object in place and returns the slot to the free list
    void release(uint16_t idx) {
        assert(idx < POOL_SIZE);
        slots[idx].reset();
        free_stack[free_top++] = idx;
    }

    Track& get(uint16_t idx) {
        assert(idx < POOL_SIZE);
        return slots[idx];
    }

    size_t capacity() const { return POOL_SIZE; }
    size_t in_use() const { return POOL_SIZE - free_top; }

private:
    std::array<Track, POOL_SIZE> slots{};
    std::array<uint16_t, POOL_SIZE> free_stack;
    size_t free_top = 0;
};