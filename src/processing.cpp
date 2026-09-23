// Companion file
#include "processing.hpp"

// Local files
#include "kalman.hpp"       
#include "parameters.h"     
#include "TrackManager.hpp" 
#include "SAEdetector.hpp"  
#include "track_logger.hpp"
#include "track_summary_logger.hpp"
#include "detection_corroboration.hpp"
#include "raw_event_logger.hpp"
#include "track_types.hpp"

#include "profiler.hpp"

// Other includes
#include <cstdint>
#include <iostream>
#include "yaml-cpp/yaml.h"      
#include <eigen3/Eigen/Dense>   
#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>           
#include <fstream>               
#include <iomanip>
#include <memory>
#include <math.h>
#include <opencv2/core/core.hpp>     
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp>  
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <unistd.h>

namespace processing {

    namespace {
        // Resolves the directory containing the currently-running executable, by
        // reading the Linux-specific /proc/self/exe self-symlink. This is
        // independent of the process's current working directory (unlike a plain
        // relative path), and independent of argv[0] (which isn't always a
        // resolvable path - e.g. when found via $PATH).
        //
        // Assumes configs/ sits next to the directory the built binary lives in
        // (src/../configs, per the project's current layout). If you later move
        // the build output somewhere else (e.g. a build/ directory), adjust the
        // ".." below - or better, make the configs directory itself overridable
        // (see the comment in setup() below).
        std::filesystem::path executable_directory() {
            char buffer[4096];
            const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (length <= 0) {
                throw std::runtime_error("could not resolve executable path via /proc/self/exe");
            }
            buffer[length] = '\0';
            return std::filesystem::path(buffer).parent_path();
        }
    }

    // moves 'slot' to the front of the MRU list, evicting the oldest entry
    void ProcessingPipeline::mru_touch(int slot) {
        int existing = -1;
        for (int k = 0; k < mru_len; k++) {
            if (mru_slots[k] == slot) { existing = k; break; }
        }
        int shift_from = (existing >= 0) ? existing : std::min(mru_len, MRU_SIZE - 1);
        for (int k = shift_from; k > 0; k--) {
            mru_slots[k] = mru_slots[k - 1];
        }
        mru_slots[0] = slot;
        if (existing < 0 && mru_len < MRU_SIZE) mru_len++;
    }

    // ---- HIGH PASS FILTERS ---- //
    // (per-pixel updates) A classic Surface of Active Events (SAE) visual reconstruction.
    void ProcessingPipeline::high_pass(double ts, int x, int y, int p, int alpha)
    {
        double* li_row = log_intensity_state.ptr<double>(y);
        double* ts_row = ts_array.ptr<double>(y);

        const double decay = std::exp(-alpha * (ts - ts_row[x]));
        li_row[x] = li_row[x] * decay + (p > 0 ? -contrast_threshold : contrast_threshold);
        ts_row[x] = ts; // remember when this pixel was last activated
    }

    // (whole-image updates) Applies an update to the entire SAE at once.
    void ProcessingPipeline::high_pass_global(double ts, int &alpha)
    {
        cv::Mat beta;
        cv::exp(-alpha * (ts - ts_array), beta); // decay factor per-pixel
        log_intensity_state = log_intensity_state.mul(beta);
        ts_array.setTo(ts); // mark every pixel as up to date as of ts
    }

    // ---- FRAME PUBLISHING ---- //
    // Builds a self-contained snapshot of the current visual state and
    // hands it to the render thread. Called from process_batch(), on the
    // PROCESSING thread - this function must stay cheap and must never
    // touch OpenCV's GUI/window/video-writer APIs; all of that lives in
    // render_frame() on the render thread instead.
    void ProcessingPipeline::publish_frame(double ts) {
        high_pass_global(ts, params.alpha);
        if (frame_output == nullptr) {
            return; // shouldn't happen if setup() succeeded, but don't crash if it does
        }
        frame_job job;
        job.log_intensity_snapshot = log_intensity_state.clone(); // deep copy - render thread gets its own
        job.ts = ts;
        auto tracks = track_manager->getTracks();
        job.tracks.reserve(tracks.size());
        for (auto* track : tracks) {
            // ->state() and ->P_x() already return copies (Eigen::MatrixXd
            // by value), not references into the live filter - safe to
            // capture here even though this specific pool slot could be
            // reset() and reused for a different physical track before the
            // render thread gets around to drawing this frame.
            job.tracks.push_back(track_snapshot{track->state(), track->P_x(), track->validated == 1});
        }
        frame_output->push(std::move(job));
    }

    // RENDER AND SAVE ONE FRAME - RENDER THREAD ONLY.
    // Converts a snapshot's reconstructed image into a viewable 8-bit RGB
    // image. Draws an ellipse for each snapshotted track. Optionally
    // writes the frame to a video file/png. Never reads shared tracking
    // state - everything it needs is in `job`.
    bool ProcessingPipeline::render_frame(const frame_job& job) {
        cv::Mat image;
        cv::exp(job.log_intensity_snapshot, image); // undo the log: back to linear "intensity"
        double minVal = 1.4;
        double maxVal = 0.4;
        // NOTE: minVal > maxVal here, so this normalization is intentionally inverted
        // (produces a particular contrast look for this sensor's typical value range).
        image = (image - minVal) / (maxVal - minVal);
        cv::Mat img(image.rows, image.cols, CV_64FC1, (char*)image.data);
        img.convertTo(img, CV_8U, 255.0 / 1.0); // scale to 8-bit grayscale
        cv::Mat cimg;
        cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB); // convert to RGB so we can draw colored ellipses
 
        for (const auto& track : job.tracks) {
            const auto& x_hat_track = track.state;
            const auto& P_track = track.P;

            // 1. Check ALL state variables including the angle at index 6
            if (std::isnan(x_hat_track(0)) || std::isnan(x_hat_track(1)) ||
                std::isnan(x_hat_track(4)) || std::isnan(x_hat_track(5)) || std::isnan(x_hat_track(6)) ||
                std::isinf(x_hat_track(0)) || std::isinf(x_hat_track(1)) ||
                std::isinf(x_hat_track(4)) || std::isinf(x_hat_track(5)) || std::isinf(x_hat_track(6))) {
                continue; 
            }

            // Convert orientation angle to degrees safely
            double rotationAngle = x_hat_track(6) * 57.295;
            rotationAngle = std::fmod(rotationAngle, 360.0);
            if (rotationAngle < 0.0) {
                rotationAngle += 360.0;
            }

            // Explicitly clamp values to avoid extreme overflows
            int ellipse_w = std::max(0, cvRound(std::abs(x_hat_track(4))));
            int ellipse_h = std::max(0, cvRound(std::abs(x_hat_track(5))));
            cv::Point center_pt(cvRound(x_hat_track(0)), cvRound(x_hat_track(1)));

            // Track state ellipse
            if (track.validated) {
                cv::ellipse(cimg, center_pt, cv::Size(ellipse_w, ellipse_h), rotationAngle, 0, 360, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            } else if (params.f_show_candidates) {
                cv::ellipse(cimg, center_pt, cv::Size(ellipse_w, ellipse_h), rotationAngle, 0, 360, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            }

            // Uncertainty (covariance) ellipse
            if (params.disp_covariance_flag) {
                if (!std::isnan(P_track(0, 0)) && !std::isnan(P_track(1, 1)) &&
                    !std::isinf(P_track(0, 0)) && !std::isinf(P_track(1, 1))) {
                    
                    double cov_w_raw = 50.0 * std::sqrt(std::abs(P_track(0, 0)));
                    double cov_h_raw = 50.0 * std::sqrt(std::abs(P_track(1, 1)));

                    int cov_w = std::max(0, cvRound(cov_w_raw));
                    int cov_h = std::max(0, cvRound(cov_h_raw));
                    double cov_angle = x_hat_track(6) * (180.0 / M_PI);

                    // DIAGNOSTIC PRINT: If it crashes on the next line, look at your console output!
                    if (cov_w < 0 || cov_h < 0) {
                        std::cout << "[CRASH PREVENTED] Invalid Covariance Size: " << cov_w << "x" << cov_h << std::endl;
                        continue;
                    }

                    cv::ellipse(
                        cimg,
                        center_pt,
                        cv::Size(cov_w, cov_h),
                        cov_angle,
                        0,
                        360,
                        cv::Scalar(255, 0, 0),
                        1,
                        cv::LINE_AA);
                }
            }
        }
 
        cv::imshow("Video" + std::to_string(camera_id), cimg); // show the reconstructed+annotated frame
        int key = cv::waitKey(1);

        // allow close on ESC key
        if (key == 27) {
            std::cout << "[cam " << int(camera_id) << "] ESC key pressed. Exiting..." << std::endl;
            return true; // signal to the caller that we want to exit
        }
 
        image_count += 1;
        return false; // signal to the caller that we want to continue
    }


    // ---- SETUP() ---- //
    // Runs once, before the first packet arrives.
    bool ProcessingPipeline::setup(const std::string &config_name, frame_queue& frames, TrackUpdateQueue& threadC) {
        frame_output = &frames;
        threadC_output = &threadC;

        // PARSE THE CONFIG //
        try {
            // configs/ is a sister directory of src/ (where this binary is built).
            // See executable_directory()'s comment above if you move the build output elsewhere.
            const std::filesystem::path file_config_path =
                executable_directory() / ".." / "configs" / (config_name + ".yaml");
            if (!std::filesystem::exists(file_config_path)) {
                throw std::runtime_error("config file not found: " + file_config_path.string());
            }
            params = loadParametersFromYAML(file_config_path.string());
        } catch (const std::exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
            return false; // Fail fast if config is missing/invalid
        }

        // Allocate the two per-pixel accumulator images used for the event->intensity reconstruction.
        log_intensity_state = cv::Mat::zeros(params.height, params.width, CV_64FC1);
        ts_array = cv::Mat::zeros(params.height, params.width, CV_64FC1);
        contrast_threshold = params.contrast_threshold;
        int n_state = params.n_state;

        //assign the detector type to use
        use_dt_detector = static_cast<bool>(params.use_dt_detector);

        //assign the show display flag
        show_display = static_cast<bool>(params.show_display);

        // cache these once to avoid sqrt and power functions later - just compare to these squared values
        dist_threshold_sq = params.dist_threshold * params.dist_threshold;
        detector_dist_threshold_sq = params.detector_dist_threshold * params.detector_dist_threshold;

        deleted_IDs_scratch.reserve(8);

        // TrackLogger //
        if (params.save_files) {
            // Create a unique filename for the log file based on the current timestamp
            auto const now = std::chrono::system_clock::now();
            std::time_t const now_time_t = std::chrono::system_clock::to_time_t(now);
            // Convert to local time safely
            std::tm local_tm;
            #if defined(_WIN32)
                localtime_s(&local_tm, &now_time_t);
            #else
                localtime_r(&now_time_t, &local_tm); // Thread-safe POSIX alternative
            #endif
            std::stringstream ss;
            ss << std::put_time(&local_tm, "%d-%m-%Y-%H-%M-%S") << "_cam" << int(camera_id);
            std::string formatted = ss.str();
        
            try {
                track_logger = std::make_unique<TrackLogger>("./track_logs/" + formatted + ".bees"); // Binary Event Evolution Storage
                track_summary_logger = std::make_unique<TrackSummaryLogger>("./track_logs/" + formatted + ".beesum"); // Binary Event Evolution Summary
            } catch (const std::exception &ex) {
                std::cout << "Error: " << ex.what() << std::endl;
                return false;
            }

            // Set up the raw event logger if set in params //
            if (params.create_event_log_file) {
                std::cout << "Raw event logging requested. Generating .aemotevt file alongside the standard files..." << std::endl;
                try {
                    raw_event_logger = std::make_unique<RawEventLogger>(
                        "./track_logs/" + formatted + ".aemotevt");
                } catch (const std::exception &ex) {
                    std::cerr << "Error: " << ex.what() << std::endl;
                    return false;
                }
            }

        } else {
            std::cout << "Logging to file disabled by config. Skipping file creation..." << std::endl;
        }

        // NOTE: display window / video writer / output-image-folder setup
        // used to happen right here. It's now in render_setup(), which
        // runs on the dedicated render thread instead - see processing.hpp
        // for why (keeping every OpenCV GUI/IO call on one thread).

        // INITALISE SAE-DETECTOR //
        detector = std::make_unique<SAEdetector>(
            params.height,
            params.width,
            params.SAE_ksize,
            params.SAE_alpha,
            params.SAE_min_contributions,
            params.SAE_min_active_pixels,
            params.SAE_detection_threshold,
            params.detector_dt_threshold,
            params.SAE_recency_window
        );

        // BUILD THE KALMAN FILTER MATRIX TEMPLATES //
        int n_target_local = 1;
        F = Eigen::MatrixXd::Zero(n_target_local * n_state, n_state);
        C = Eigen::MatrixXd::Zero(3, n_state);        // maps state -> the 3 measured quantities (x, y, ts-related)
        R = Eigen::MatrixXd::Identity(3, 3);          // measurement noise covariance
        P = Eigen::MatrixXd::Zero(n_target_local * n_state, n_state); // initial estimate error covariance
        Q = Eigen::MatrixXd::Zero(n_state, n_state);  // process noise covariance
        A = Eigen::MatrixXd::Zero(2, 2);              // Sigma^(1/2) - used for shape/orientation estimation elsewhere
        x0 = Eigen::MatrixXd::Zero(n_target_local, n_state);

        for (int j = 0; j < n_target_local; j++)
        {
            for (int i = 0; i < n_state; i++)
            {
                F(j * n_state + i, i) = 1; // start as identity (each state persists by default)
            }
            // Constant-velocity coupling: position += velocity * dt
            F(j * n_state, 2) = params.dt;     // x += vx * dt
            F(j * n_state + 1, 3) = params.dt; // y += vy * dt
            F(j * n_state + 6, 7) = params.dt; // theta += angular_velocity * dt
        }

        Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(n_state, n_state);
        x0.block(0, 4, n_target_local, 1).setConstant(params.lambda_init);  // initial size param 1
        x0.block(0, 8, n_target_local, 2).setConstant(0);                   // initial extra states
        x0.block(0, 5, n_target_local, 1).setConstant(params.lambda_init);  // initial size param 2

        // Diagonal (uncorrelated) initial covariance, one variance value per state dimension.
        P_block.diagonal() << params.var_x, params.var_y, params.var_vx,
            params.var_vy, params.var_lambda_1, params.var_lambda_2, params.var_theta,
            params.var_q, 50, 50;

        // Diagonal process noise - how much we expect each state to drift/be uncertain per step.
        Q.diagonal() << params.q_x, params.q_y, params.q_vx, params.q_vy,
            params.q_lambda_1, params.q_lambda_2, params.q_theta, params.q_q, 5, 5;

        for (int row = 0; row < n_target_local; row++)
        {
            P.block(row * n_state, 0, n_state, n_state) << P_block;
        }

        // SETUP THE TRACK MANAGER //
        track_manager = std::make_unique<TrackManager>(params.dt, F, C, Q, R, P, A, x0, params.ring_buffer_len, n_state, params.pool_size, params.accumulator_count_thresh, params.accumulator_time_thresh);
        track_manager->update_default_dist_threshold(params.dist_threshold);
        track_manager->update_frame_dimensions(params.height, params.width);
        track_manager->update_event_rate_threshold(params.event_rate_threshold);
        track_manager->update_rate_per_area(params.rate_per_area);
        track_manager->store_parameters(
            params.evaluate_ts_age, params.evaluate_dt_terminate, params.evaluate_low_activity_factor
        );
        track_manager->setLogger(track_logger.get(), params.save_files);
        track_manager->setSummaryLogger(track_summary_logger.get());

        // SETUP THE CORROBORATOR //
        corroboration_grid = std::make_unique<DetectionCorroborationGrid>(
            params.width, params.height,
            params.corroboration_cell_size,
            params.corroboration_window,
            params.corroboration_direction_cos_threshold
        );

        return true;
    } // end setup()

    // Runs once, before the first frame, on the RENDER thread. Must not be
    // called until setup() (above) has already returned true - reads
    // params, which setup() is what populates.
    void ProcessingPipeline::render_setup() {
        if (!show_display) {
            std::cout << "[cam " << int(camera_id) << "] Display disabled by config. Skipping window creation..." << std::endl;
            return;
        }
        cv::namedWindow("Video" + std::to_string(camera_id));
        cv::resizeWindow("Video" + std::to_string(camera_id), params.width, params.height);
 
        // RENDER INITIAL EMPTY FRAME //
        frame_job empty_job;
        empty_job.log_intensity_snapshot = log_intensity_state.clone();
        empty_job.ts = 0.0;
        render_frame(empty_job);
    }
 
    // Runs once, on the render thread, after the frame_queue is stopped and drained.
    void ProcessingPipeline::render_teardown() {
        if (show_display) {
            cv::destroyWindow("Video" + std::to_string(camera_id));
        }
    }


    // ---- EVENT PROCESSOR LOGIC ---- //
    // Runs once for every paclet pulled from the queue.
    void ProcessingPipeline::process_batch(const std::vector<sepia::dvs_event>& events) {
        if (events.empty()) return;

        // TRIGGER PULSE HANDLER //
        const uint32_t new_pulses = pending_trigger_pulse_count.exchange(0, std::memory_order_acquire);
        if (new_pulses > 0) {
            trigger_pulse_count += new_pulses; // update the global synced seconds counter
            const int64_t raw_t = pending_internal_timestamp_of_last_trigger_pulse.load(std::memory_order_acquire);
            if (have_first_ts) {
                internal_timestamp_of_last_trigger_pulse = (static_cast<double>(raw_t) - t1) * 1e-6; // update the internal reference for the last global synced second marker
            }
        }
        // BACK TO STANDARD PROCESSING
        
        // `events` is one batch (one USB packet's worth), events are in timestamp order within the batch and across batches.
        total_events += events.size();
        
        // iterate over every event in the packet:
        for (const auto& event : events) {
            // event.t  -> microseconds
            // event.x  -> width pixel coord
            // event.y  -> height pixel coord
            // event.on -> true = ON, false = OFF

            if (!have_first_ts) // assign t1 on first event
            {
                t1 = static_cast<double>(event.t);
                have_first_ts = true;

                //  retroactively fix the reference point is a pulse landed before the first event somehow
                if (new_pulses > 0) {
                    const int64_t raw_t = pending_internal_timestamp_of_last_trigger_pulse.load(std::memory_order_relaxed);
                    internal_timestamp_of_last_trigger_pulse = (static_cast<double>(raw_t) - t1) * 1e-6; // keep this relative to the first event, too, to keep it consistent with the main processing loop's time handling
                }
            }
            // convert microseconds to seconds, relative to the first event
            double ts = (static_cast<double>(event.t) - t1) * 1e-6;

            //convert variables for AEMOT code compatibility
            uint16_t c = static_cast<uint16_t>(event.x);
            uint16_t r = static_cast<uint16_t>(event.y);
            uint8_t p = static_cast<uint8_t>(event.on);

            // pack the event into an Eigen vector for downstream Kalman update calls
            e << c, r, ts;
            if (t_last == -1.0){
                t_last = ts; // initialise on first iteration
            }
            
            // update the reconstructed-image pixel this event touched
            if (show_display){
                AEMOT_TIMED_SCOPE("high_pass");
                high_pass(ts, c, r, p, params.alpha);
            }

            bool f_event_associated = false; // was this specific event matched to an existing track?

            // NEAREST NEIGHBOUR DATA ASSOCIATION //
            //Find which existing track's predicted position is closest to this event's location
            double dist_min_sq = 1e18; // a very large starting squared distance (distance = 1e6)
            
            // FAST PATH: check the MRU list first, most-recent-first - more likely to find a match here, and then move on for speed
            {
                AEMOT_TIMED_SCOPE("nearest_neighbour_MRU");

                for (int k = 0; k < mru_len; k++) {
                    int slot = mru_slots[k];
                    KalmanFilter* trk = track_manager->getTrackUnchecked(slot);
                    if (!trk->active) continue;
                    double pred_x, pred_y;
                    trk->predicted_position(ts, pred_x, pred_y);
                    const double dx = static_cast<double>(c) - pred_x;
                    const double dy = static_cast<double>(r) - pred_y;
                    //double dx = trk->pos_x();
                    //double dy = trk->pos_y();
                    const double d_sq = dx * dx + dy * dy;
                    if (d_sq < dist_threshold_sq) {
                        id = slot;
                        dist_min_sq = d_sq;
                        f_event_associated = true;
                        mru_hit_count++;
                        break;
                    }
                }
                if (!f_event_associated) mru_miss_count++;
            }

            // FALLBACK: full scan, only on an MRU miss
            if (!f_event_associated) {
                AEMOT_TIMED_SCOPE("nearest_neighbour_fallback");
                dist_min_sq = 1e18;
                const int n_tracks = track_manager->len();
                // itterate through the track manager's current tracks to find the closest
                for (int i = 0; i < n_tracks; i++)
                {
                    KalmanFilter* trk = track_manager->getTrackUnchecked(i);
                    // if the track isn't active (just a waiting container), skip it
                    if (!trk->active) continue;
                    double pred_x, pred_y;
                    trk->predicted_position(ts, pred_x, pred_y);
                    const double dx = static_cast<double>(c) - pred_x;
                    const double dy = static_cast<double>(r) - pred_y;
                    const double d_sq = dx * dx + dy * dy;
                    
                    if (d_sq < dist_min_sq)
                    {
                        dist_min_sq = d_sq;
                        id = i; // remember the index of the closest track
                    }
                }
                f_event_associated = (dist_min_sq < dist_threshold_sq);
            }

            // Kalman update and logging
            if (f_event_associated) {
                AEMOT_TIMED_SCOPE("kalman_update_and_logging");
                mru_touch(id); // remember this slot as most-recently-active

                KalmanFilter* trk = track_manager->getTrackUnchecked(id);
                
                // raw event logging if requested via config
                if (raw_event_logger) {
                    raw_event_logger->log(ts, c, r, p, trk->getID());
                }
                // --
                
                const bool should_flush = trk->accumulate_event(c, r, ts, p);

                if (should_flush) {
                    Eigen::Vector3d flush_e;
                    int flush_p;
                    trk->get_accumulated_measurement(flush_e, flush_p);
                    trk->reset_accumulator();
                    trk->update(flush_e, 0, flush_p);

                    // log the post-update state, buffered until validated
                    track_manager->logTrackUpdate(id, ts, trk->state_data());

                    // EMIT ONTO SHARED CROSS-CAMERA THREAD C OUTPUT QUEUE //
                    if (threadC_output != nullptr && trk->validated) {
                        // SYNC INTERNAL TIMESTAMP TO GLOBAL TIME BEFORE SENDING IT OUT INTO THE OUTSIDE WORLD //
                        double tg = static_cast<double>(trigger_pulse_count) + (ts - internal_timestamp_of_last_trigger_pulse);
                                                        // seconds          // fraction of current second
                        const double* state = trk->state_data();
                        RawState msg;
                        msg.cam_id = camera_id;
                        msg.track_id = static_cast<uint32_t>(trk->getID());
                        msg.tg = tg;
                        msg.x = state[0];
                        msg.y = state[1];
                        msg.vx = state[2];
                        msg.vy = state[3];
                        threadC_output->push(msg);
                    }
                }
            }
            
            int detector_output = -1;
            int detector_output_dt = -1;
            
            if (!f_event_associated) // if the event hasn't been associated yet
            {   
                if (use_dt_detector) { // dt_detector route

                    // add event to detector's stash
                    detector->addEvent({(double)c, (double)r, ts, (double)p});
                    detection_event_count++;

                    if ((track_manager->hasAvailableSlot()) &
                        (dist_min_sq > detector_dist_threshold_sq || track_manager->activeCount() == 0) &
                        (detection_event_count > params.SAE_operation_rate)) 
                    {
                        detection_event_count = 0;
                        // perform the dt detection on this event
                        detector_output_dt = detector->performDetection_dt({(double)c, (double)r, ts, (double)p});
                    }

                    if (detector_output_dt == 1) {
                        const bool corroborated = corroboration_grid->check_and_update(c, r, ts, 0.0, 0.0);
                        if (corroborated) {
                            NewTrackResult new_trk = track_manager->createNewTrack({(double)c, (double)r, ts});
                            mru_touch(new_trk.slot);
                        }
                    }

                } else { // standard detector route

                    //debug
                    //std::cout << "Event NOT associated to existing track. Adding to detector.\n";
                    
                    // add this event to the SAE
                    {
                        AEMOT_TIMED_SCOPE("detection_addevent");
                        detector->addEvent({(double)c, (double)r, ts, (double)p});
                        detection_event_count++;
                    }
                    
                    // if we're able to perform an SAE detection, then do it
                    if ((track_manager->hasAvailableSlot()) &
                        (dist_min_sq > detector_dist_threshold_sq || track_manager->activeCount() == 0) &
                        (detection_event_count > params.SAE_operation_rate)) 
                    {
                        
                        detection_event_count = 0;
                        //perform the SAE detection on this event
                        {
                            AEMOT_TIMED_SCOPE("detection_performdetection");
                            detector_output = detector->performDetection({(double)c, (double)r, ts, (double)p});
                        }
                        
                        // check if the detector returned a positive result
                        if (detector_output == 1){
                            bool corroborated = false;
                            {
                                AEMOT_TIMED_SCOPE("detection_corroboration");
                                double lx_val, ly_val;
                                detector->getLastDirection(c, r, lx_val, ly_val);
                                corroborated = corroboration_grid->check_and_update(c, r, ts, lx_val, ly_val);
                            }
                            
                            if (corroborated) {
                                AEMOT_TIMED_SCOPE("detection_createnewtrack");
                                NewTrackResult new_trk = track_manager->createNewTrack({(double)c, (double)r, ts});
                                mru_touch(new_trk.slot);
                            }

                        }
                        
                    }
                }

            }

            // TRACK EVALUATION AND PRUNING //
            // Periodically ask the TrackManager to check whether any tracks should be deleted.
            // How we do this (one of three ways) is based on the config and how many events have passed:
            {
                AEMOT_TIMED_SCOPE("track_evaluation");
                deleted_IDs_scratch.clear();

                double_track_evaluation_counter++;
                full_evaluation_counter++;

                // (a) Duplicate-track check: runs only every 100 events, and only if there's more than 1 track.
                if ((track_manager->activeCount() > 1) & (params.f_evaluate == 1) & (double_track_evaluation_counter > 100))
                {  
                    //debug
                    //std::cout << "[track eval] Checking duplicate tracks...\n";

                    double_track_evaluation_counter = 0;
                    auto ids = track_manager->evaluateDoubleTracks();
                    deleted_IDs_scratch.insert(deleted_IDs_scratch.end(), ids.begin(), ids.end());
                }
                // (b) Full evaluation (age/activity/etc.) - runs every 200 events.
                if ((track_manager->activeCount() > 0) & (params.f_evaluate == 1) & (full_evaluation_counter > 200)){
                    //debug
                    //std::cout << "[track eval] Performing full eval...\n";

                    full_evaluation_counter = 0;
                    auto ids = track_manager->evaluateTracks(ts);
                    deleted_IDs_scratch.insert(deleted_IDs_scratch.end(), ids.begin(), ids.end());  
                }
                // (c) Fallback: only delete tracks that have left the frame (position-only check).
                if ((track_manager->activeCount() > 0) & (params.f_evaluate == 0)){
                    //debug
                    //std::cout << "[track eval] Checking position (frame edge) only...\n";

                    auto ids = track_manager->evaluateTracksPosition();
                    deleted_IDs_scratch.insert(deleted_IDs_scratch.end(), ids.begin(), ids.end());
                }

                // If anything was deleted, the same-timestamp event buffer is now stale, so clear it.
                if (!deleted_IDs_scratch.empty()) {
                    while(!same_ts_e_buffer.empty()){
                        same_ts_e_buffer.pop();
                    }
                }

                ts_kf_last = ts;
                t_last = ts; // advance the previous timestep marker, ready for the next event to come in!
            }

            // PERIODIC DISPLAY REFRESH //
            // refresh at most once every 1/publish_framerate seconds of event time, on this same thread
            if (show_display && params.publish_framerate > 0 && ts > t_next_publish) {
                AEMOT_TIMED_SCOPE("publish_frame");
                //high_pass_global(ts, params.alpha); // COULD move this to the render thread too (currently on processing thread)
                publish_frame(ts); // pass this off to the dedicated thread that handles all OpenCV GUI/video-writer calls, so we don't block the event-processing thread
                t_next_publish = ts + (1.0 / params.publish_framerate);
            }
        }
    }

    void ProcessingPipeline::teardown() {
        // write out any summaries for tracks that are still alive at teardown
        if (track_manager) {
            track_manager->flushAllSummaries();
        }

        // flush any remaining logs from the logger's buffer onto the writer thread before shutdown!
        if (track_logger) {
            track_logger->close();
        }
        if (track_summary_logger) {
            track_summary_logger->close();
        }
        if (raw_event_logger) {
            raw_event_logger->close();
        }

        Profiler::instance().report();

        if (mru_hit_count + mru_miss_count > 0) {
            std::cout << "[cam " << int(camera_id) << "] MRU hit rate: " << mru_hit_count << "/" << (mru_hit_count + mru_miss_count)
                    << " (" << (100.0 * mru_hit_count / (mru_hit_count + mru_miss_count)) << "%)\n";
        }

        std::cout << "[cam " << int(camera_id) << "] processed " << total_events << " events total\n";
    }

} // namespace processing
