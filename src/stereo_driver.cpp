#include "evk4.hpp" // pulls in camera.hpp, psee.hpp and sepia.hpp too

#include "event_queue.hpp"
#include "processing.hpp"
#include "track_update_queue.hpp"
#include "threadC.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

std::atomic<bool> running{true};

void handle_sigint(int) {
    running.store(false);
}

// Here is everything that one camera needs: its own queues, its own ProcessingPipeline
// instance, its own worker/render threads, its own physical camera handle.
// Two of these run concurrently with no shared mutable state between them.
// The only thing that they share is the TrackUpdate Queue reference passed into start(),
// which is itself designed for multi-producer use.
struct CameraContext {
    uint8_t camera_id;
    std::string config_name;
    sepia::usb::device_properties device;

    event_queue events{256};
    frame_queue frames{3};
    processing::ProcessingPipeline pipeline;
    std::promise<bool> setup_promise;
    std::thread worker;
    std::thread render_thread;
    std::unique_ptr<sepia::evk4::camera> camera_handle;

    CameraContext(uint8_t id, std::string cfg, sepia::usb::device_properties dev)
        : camera_id(id), config_name(std::move(cfg)), device(std::move(dev)), pipeline(id) {}
    
    bool start(TrackUpdateQueue& threadC_queue) {
        // Worker thread: this is the ONLY thread that calls into this camera's
        // ProcessingPipeline, so process_batch never needs to worry about
        // concurrent calls.
        worker = std::thread([this, &threadC_queue]() {
            const bool ok = pipeline.setup(config_name, frames, threadC_queue);
            setup_promise.set_value(ok);

            if (!ok) {
                std::cerr << "[cam " << int(camera_id) << "] processing setup failed, aborting\n";
                running.store(false);
                return;
            }
            std::vector<sepia::dvs_event> batch;
            while (events.pop(batch)) {
                pipeline.process_batch(batch);
            }
            pipeline.teardown();
        });

        if (!setup_promise.get_future().get()) {
            events.stop();
            worker.join();
            return false;
        }

        // render thread
        render_thread = std::thread([this]() {
            pipeline.render_setup();
            frame_job job;
            while (frames.pop(job)) {
                if (pipeline.render_frame(job)) {
                    running.store(false);
                    break;
                }
            }
            pipeline.render_teardown();
        });

        // Physical camera wiring
        auto current_batch = std::make_shared<std::vector<sepia::dvs_event>>();
        current_batch->reserve(4096);

        auto before_buffer = [](std::size_t, std::size_t) { return true; };

        auto after_buffer = [this, current_batch]() {
            if (!current_batch->empty()) {
                events.push(std::move(*current_batch));
                current_batch->clear();
                current_batch->reserve(4096);
            }
        };

        auto handle_trigger_event = [](sepia::evk4::trigger_event) {};

        auto handle_event = [current_batch](sepia::dvs_event event) {
            current_batch->push_back(event);
        };

        auto handle_exception = [this](std::exception_ptr exception) {
            try {
                std::rethrow_exception(exception);
            } catch (const std::exception& error) {
                std::cerr << "[cam " << int(camera_id) << "] camera error: " << error.what() << "\n";
            }
            running.store(false);
        };

        camera_handle = std::make_unique<sepia::evk4::camera>(sepia::evk4::make_camera(
            handle_event,
            handle_trigger_event,
            before_buffer,
            after_buffer,
            handle_exception,
            sepia::evk4::default_parameters,
            device.serial,
            std::chrono::milliseconds(100),
            128, // buffers_count
            16384, // fifo_size
            [this]() { std::cerr << "[cam " << int(camera_id) << "] warning: packet dropped\n"; }
        ));
    
        return true;
    }

    void shutdown() {
        camera_handle.reset(); // stop the physical device first
        events.stop();
        if (worker.joinable()) worker.join();
        frames.stop();
        if (render_thread.joinable()) render_thread.join();
    }

};

// THREAD C //
void run_threadC(TrackUpdateQueue& threadC_queue) {
    TrackUpdateMsg msg;
    while (threadC_queue.pop(msg)) {
        threadC::process_track_update(msg);
    }
}


// ENTRY POINT //
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " <config_name_a> <config_name_b> [serial_a] [serial_b]\n";
        std::cerr << "  (config names are looked up as configs/<config_name>.yaml by processing::setup)\n";
        std::cerr << "  (if serials are omitted, the first two EVK4 devices found are used, in enumeration order)\n";
        return 1;
    }
    const std::string config_name_a = argv[1];
    const std::string config_name_b = argv[2];
    const std::string serial_a = (argc > 3) ? argv[3] : "";
    const std::string serial_b = (argc > 4) ? argv[4] : "";

    std::signal(SIGINT, handle_sigint);

    // --- Find two EVK4 devices --- //
    std::vector<sepia::usb::device_properties> evk4_devices;
    for (const auto& available_device : sepia::psee::available_devices()) {
        if (available_device.type == sepia::psee::EVK4) {
            evk4_devices.push_back(available_device);
        }
    }

    auto pick_device = [&](const std::string& serial) -> sepia::usb::device_properties* {
        if (!serial.empty()) {
            for (auto& d : evk4_devices) {
                if (d.serial == serial) return &d;
            }
            return nullptr;
        }
        return nullptr;
    };

    sepia::usb::device_properties* dev_a = pick_device(serial_a);
    sepia::usb::device_properties* dev_b = pick_device(serial_b);

    // Fall back to enumeration order for whichever wasn't matched by an explicit serial
    std::vector<sepia::usb::device_properties*> remaining;
    for (auto& d : evk4_devices) {
        if (&d != dev_a && &d != dev_b) remaining.push_back(&d);
    }
    size_t next_remaining = 0;
    if (dev_a == nullptr) {
        if (next_remaining >= remaining.size()) {
            std::cerr << "could not find a device for camera A (need serial \"" << serial_a << "\" or an unassigned EVK4)\n";
            return 1;
        }
        dev_a = remaining[next_remaining++];
    }
    if (dev_b == nullptr) {
        if (next_remaining >= remaining.size()) {
            std::cerr << "could not find a device for camera B (need serial \"" << serial_b << "\" or a second EVK4)\n";
            return 1;
        }
        dev_b = remaining[next_remaining++];
    }

    std::cout << "[Camera A]: opening EVK4 " << dev_a->serial << ", with config: configs/" << config_name_a << ".yaml...\n";
    std::cout << "[Camera B]: opening EVK4 " << dev_b->serial << ", with config: configs/" << config_name_b << ".yaml...\n";
 
    // Thread C
    TrackUpdateQueue threadC_queue(4096);
    std::thread threadC_thread(run_threadC, std::ref(threadC_queue));

    // Camera A and Camera B
    CameraContext cam_a(0, config_name_a, *dev_a);
    CameraContext cam_b(1, config_name_b, *dev_b);

    if (!cam_a.start(threadC_queue)) { return 1; }
    if (!cam_b.start(threadC_queue)) { cam_a.shutdown(); return 1; }

    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[Camera 0 (Camera A)] queue dropped " << cam_a.events.dropped_batches() << "/" << cam_a.events.pushed_batches() + cam_a.events.dropped_batches() << " batches\n";
        std::cout << "[Camera 1 (Camera B)] queue dropped " << cam_b.events.dropped_batches() << "/" << cam_b.events.pushed_batches() + cam_b.events.dropped_batches() << " batches\n";
        std::cout << "[Thread C] queue dropped " << threadC_queue.dropped_count() << " messages so far\n";
    }   

    cam_a.shutdown();
    cam_b.shutdown();

    threadC_queue.stop();
    threadC_thread.join();

    return 0;

}