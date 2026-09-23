#pragma once

#include "track_types.hpp"
#include "track_pool.hpp"

#include <unordered_map>

// --- This owns the track pool and the key to slot mapping --- //
// This is the only thing that Thread C touches for track lookup/creation/teardown
template <size_t POOL_SIZE>
class Registry {
public:
    Registry() {
        // Reserve the map up front - * 2 because each validated pair holds 2 keys per slot
        map.reserve(POOL_SIZE * 2);
    }

    // Look up an existing track for this key (cam_id, track_id)
    // Returns INVALID_SLOT if not present
    uint16_t find(CamId cam_id, int32_t track_id) const {
        auto it = map.find(make_key(cam_id, track_id));
        return it == map.end() ? INVALID_SLOT : it->second;
    }

    // Create a brand-new WAITING track for this key and register it
    // Caller is responsible for populating last_a/last_b on the returned Track.
    // Returns INVALID_SLOT if the pool is exhausted
    uint16_t create_waiting(CamId cam_id, int32_t track_id) {
        uint16_t idx = pool.acquire();
        if (idx == INVALID_SLOT) {
            return INVALID_SLOT; // pool exhausted
        }
        uint64_t key = make_key(cam_id, track_id);
        Track& track = pool.get(idx);
        if (cam_id == CamId::A) {
            track.key_a = key;
            track.has_a = true;
        } else {
            track.key_b = key;
            track.has_b = true;
        }
        map[key] = idx;
        return idx;
    }

    // Attach a matched B state onto an existing WAITING A-track and promote them to a CANDIDATE slot.
    void merge_into_candidate(uint16_t a_idx, int32_t b_track_id, const RawState& b_state) {
        Track& a = pool.get(a_idx);

        a.key_b = make_key(CamId::B, b_track_id);
        a.has_b = true;
        a.last_b = b_state;
        a.last_b_time = b_state.tg;
        a.status = TrackStatus::CANDIDATE;
        a.pass_count = 0;
        a.fail_count = 0;

        map[a.key_b] = a_idx; // Now findable by its B key too, since they've paired up
    }

    // Candidate wasn't successful. Drop the B side and keep the A slot alive again as WAITING
    void dissolve(uint16_t idx, double now, double cooldown_seconds) {
        Track& track = pool.get(idx);

        map.erase(track.key_b); // B key no longer resolvable

        track.cooldown_key = track.key_b; // remember who to avoid rematching
        track.has_cooldown = true;
        track.cooldown_until = now + cooldown_seconds;

        track.has_b = false;
        track.key_b = 0;
        track.last_b = RawState{};
        track.last_b_time = 0.0;
       
        track.status = TrackStatus::WAITING;
        track.pass_count = 0;
        track.fail_count = 0;
        track.buffer_count = 0; // discard buffered 3D states (unwritten)

    }

    // Fully close a track: erase both registered keys and free the slot
    void close(uint16_t idx) {
        Track& track = pool.get(idx);
        if (track.has_a) map.erase(track.key_a);
        if (track.has_b) map.erase(track.key_b);
        pool.release(idx);
    }

    Track& get(uint16_t idx) { return pool.get(idx); }
    size_t in_use() const { return pool.in_use(); }
    size_t capacity() const { return pool.capacity(); }

    // For the periodic evaluation and cleanup call
    template <typename Fn>
    void for_each_active(Fn&& fn) {
        // sweep over the whole pool
        for (uint16_t i = 0; i < POOL_SIZE; ++i) {
            Track& track = pool.get(i);
            if (track.status != TrackStatus::INACTIVE) {
                fn(i, track);
            }
        }
    }

private:
    TrackPool<POOL_SIZE> pool;
    std::unordered_map<uint16_t, uint16_t> map;
};