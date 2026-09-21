#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>

// THREAD C QUEUE //
// Emitted once per Kalman flush, (currently) for VALIDATED tracks only
// NOTE: track_id is only unique WITHIN a single camera's TrackManager. 
// If you're going to edit this code later, do not make the mistake of collating these numbers downstream.

// tg is synced to global time!

struct TrackUpdateMsg {
    uint8_t camera_id; // 0 = camera A (master), 1 = camera B (servant)
    uint64_t track_id;
    double tg, x, y, vx, vy;
};

// Bounded multi-producer (cam A and cam B), single-consumer (thread C) queue.
// push() is NON-BLOCKING by design. Processing threads should never stall waiting for
// thread C. If Thread C falls behind, the oldest message is dropped to make room.

class TrackUpdateQueue {

public:
    explicit TrackUpdateQueue(size_t capacity) : capacity(capacity) {}

    void push(const TrackUpdateMsg& msg) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (queue.size() >= capacity) {
                queue.pop_front();
                dropped_count_++;
            }
            queue.push_back(msg);
        }
        cv.notify_one();
    }

    // Blocks until a message is available or stop() has been called and the queue is drained.
    // Returns false only once fully stopped and drained.
    bool pop(TrackUpdateMsg& out) {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return !queue.empty() || stopping; });
        if (queue.empty()) return false;
        out = queue.front();
        queue.pop_front();
        return true;
    }

    void stop() {
        { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
        cv.notify_all();
    }

    uint64_t dropped_count() const { return dropped_count_.load(); }

private:
    size_t capacity;
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<TrackUpdateMsg> queue;
    bool stopping = false;
    std::atomic<uint64_t> dropped_count_{0};
};