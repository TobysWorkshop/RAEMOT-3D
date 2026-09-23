#pragma once

#include "track_types.hpp"

#include <array>
#include <cstdint>
#include <cassert>

// How many 3D states a CANDIDATE track can buffer before it's either
// validated (buffer gets flushed to writer) or killed (buffer gets discarded)
constexpr size_t MAX_CANDIDATE_BUFFER = 64;

constexpr uint16_t INVALID_SLOT = 0xFFFF;

// ---- One Track Object ---- //
// Occupies one pool slot for its entire lifetime:
// INACTIVE -> WAITING -> CANDIDATE -> VALIDATED -> (reset) -> INACTIVE
// The object is reused across lifetimes rather than being destoyed and recreated.
struct Track {
    TrackStatus status = TrackStatus::INACTIVE;

    // which registry key (cam_id, track_id) is currently assigned to this slot
    // A WAITING track only has one side populated (has_a / has_b tells you which)
    uint64_t key_a = 0, key_b = 0;
    bool has_a = false, has_b = false;

    RawState last_a{}, last_b{};
    double last_a_time = 0.0, last_b_time = 0.0;

    // global id assigned at the moment of validation, used downstream for fiel output
    // -1 until validated
    int64_t global_id = -1;

    // counters for candidate validation
    uint16_t pass_count = 0;
    uint16_t fail_count = 0;

    // state buffer to be filled while still a CANDIDATE. Drained and unused once VALIDATED
    std::array<State3D, MAX_CANDIDATE_BUFFER> buffer{};
    uint16_t buffer_count = 0;

    // Cooldown: last partner key this track diverged from + expiry time,
    // so that it doesn't immediately re-match the same wrong id.
    uint64_t cooldown_key = 0;
    double cooldown_until = 0.0;
    bool has_cooldown = false;

    // reset function to clear old furnature from previous track tenants in this slot
    void reset() {
        status = TrackStatus::INACTIVE;
        key_a = key_b = 0;
        has_a = has_b = false;
        last_a = RawState{};
        last_b = RawState{};
        last_a_time = last_b_time = 0.0;
        global_id = -1;
        pass_count = fail_count = 0;
        buffer_count = 0;
        has_cooldown = false;
    }

    // function to append a 3D state to the buffer. Returns true if added, false if already full
    bool push_buffer(const State3D& state) {
        if (buffer_count >= MAX_CANDIDATE_BUFFER) return false; // FULL

        buffer[buffer_count++] = state;
        return true; // STILL SPACE
    }
};

// ---- Fixed-size Track Pool ---- //
template <uint16_t POOL_SIZE>
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