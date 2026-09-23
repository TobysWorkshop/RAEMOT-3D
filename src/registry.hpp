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

    // Promotes a WAITING A-track and a WAITING B-track into one shared CANDIDATE slot.
    // Frees the now-redundant B slot back to the pool and repoints the B key at the A key slot,
    // so that both keys resolve to one track.
    void merge_into_candidate(uint16_t a_idx, uint16_t b_idx) {
        Track& a = pool.get(a_idx);
        Track& b = pool.get(b_idx);

        a.key_b = b.key_b;
        a.has_b = true;
        a.last_b = b.last_b;
        a.last_b_time = b.last_b_time;
        a.status = TrackStatus::CANDIDATE;
        a.pass_count = 0;
        a.fail_count = 0;

        map[a.key_b] = a_idx; // b's key now points at the merged slot
        pool.release(b_idx); // b's original slot is freed immediately - they've now paired up
    }

    // Split a CANDIDATE pair that failed early - recreate two independant
    // WAITING tracks (each carrying its last know state), set cooldown on both
    // against each other, and free the original merged slot.
    // Returns the two new slot indicies as {a_idx, b_idx}
    std::pair<uint16_t, uint16_t> dissolve(uint16_t merged_idx, double now, double cooldown_seconds) {
        Track& track = pool.get(merged_idx);
        uint64_t a_key = track.key_a, b_key = track.key_b;
        RawState last_a = track.last_a, last_b = track.last_b;
        double la_t = track.last_a_time, lb_t = track.last_b_time;

        pool.release(merged_idx);

        uint16_t a_idx = pool.acquire();
        uint16_t b_idx = pool.acquire();

        if (a_idx != INVALID_SLOT) {
            Track& ta = pool.get(a_idx);
            ta.key_a = a_key; ta.has_a = true;
            ta.last_a = last_a; ta.last_a_time = la_t;
            ta.cooldown_key = b_key; ta.has_cooldown = true;
            ta.cooldown_until = now + cooldown_seconds;
            map[a_key] = a_idx;
        }
        if (b_idx != INVALID_SLOT) {
            Track& tb = pool.get(b_idx);
            tb.key_a = b_key; tb.has_a = true; // stored in key_a slot of its own waiting record
            tb.last_a = last_b; tb.last_a_time = lb_t;
            tb.cooldown_key = a_key; tb.has_cooldown = true;
            tb.cooldown_until = now + cooldown_seconds;
            map[b_key] = b_idx;
        }

        return {a_idx, b_idx}; 
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