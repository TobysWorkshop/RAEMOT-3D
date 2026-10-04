// include companion file
#include "threadC.hpp"

// other local file includes
#include "track_update_queue.hpp"
#include "threadC_params.h"
#include "registry.hpp"

// other includes
#include <array>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

// custom matrix defs for simplicity
using Mat3 = std::array<double, 9>; //row-major 3x3 (for fundamental F matrix)
using Mat34 = std::array<double, 12>; // row-major 3x4 (for projection P matricies)

namespace threadC {

    // forward-declared structs for use in global namespace after
    struct MatchResult {
        bool suitable = false;
        double compatibility_score = 1e18;
        State3D triangulation{};
    };

    struct RejectEntry { 
        uint64_t key = 0; 
        double retry_at = 0.0; 
    };

    namespace { // global variables for this namespace

        // --- Resistry pointer -- //
        constexpr size_t POOL_SIZE = 128;
        Registry<POOL_SIZE> reg;

        // --- Full sweep 3D track evaluation -- //
        constexpr double SWEEP_INTERVAL = 0.1; // seconds of global tg time between eval sweeps
        constexpr double WAITING_MAX_STALE = 1.0; // Drop A-only tracks if they never pair up after this long (in seconds)
        constexpr double PAIR_MAX_STALE = 0.5; // End a track if either side (A or B) stops giving updates to it for this long (seconds)
        constexpr double CANDIDATE_MAX_AGE = 2.0; // A CANDIDATE track must validate within this many seconds, otherwise it dissolves
        double last_sweep_tg = 0.0;

        // --- CANDIDATE validation -- //
        constexpr uint16_t CANDIDATE_PASS_THRESHOLD = 5; // how many consecutive pairs need to pass to validate this track
        static_assert(CANDIDATE_PASS_THRESHOLD <= MAX_CANDIDATE_BUFFER);
        constexpr uint16_t CANDIDATE_FAIL_THRESHOLD = 3; // how many consecutive pairs need to fail to remove this track from consideration
        constexpr double COOLDOWN_SECONDS = 0.5; // how long (in seconds) do we place a failed pair on cooldown before they can pair up again later

        // --- VALIDATED divergence -- //
        constexpr uint16_t VALIDATED_FAIL_THRESHOLD = 9; // how many consecutive pairs need to fail to destroy a validated track 

        // --- Writer queue -- //
        WriterQueue writer;

        //Epipolar and velocity checks
        constexpr double EPIPOLAR_THRESH_SQ = 9.0; // ~3 px sampson distance
        constexpr double VEL_PROJ_THRESH_SQ = 25.0; // ~5 px/s combined residual

        // Time alignment for the checks
        constexpr double MAX_ALIGN_DT = 0.1; // how long will we allow two updates from different cameras to be and still time-slign them for epipolar and velocity checks

        // B reject cache stuff
        constexpr double REJECT_RETRY_DT = 0.05; // allow re-scanning of an unmatched B id after this many seconds
        std::array<RejectEntry, 64> reject_cache{}; // allows for 64 concurrent rejected B ids
        size_t reject_next = 0;

        // Camera calibration info
        Mat3 F;
        Mat34 P_A;
        Mat34 P_B;

        // performance counters for final log on teardown
        uint64_t n_validated = 0;
        uint64_t n_dissolved = 0;
        uint64_t pool_highest_load = 0;

        // parameters def
        ThreadCParameters params;

    } // end namespace for global variables

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

    // --- Full evaluation sweep --- //
    void sweep(double now) {
        reg.for_each_active([&](uint16_t i, Track& track) {
            switch (track.status) {
            case TrackStatus::WAITING:
                if (now - track.last_a_time > WAITING_MAX_STALE) reg.close(i);
                break;
            case TrackStatus::CANDIDATE:
                if (now - track.last_a_time > PAIR_MAX_STALE) reg.close(i); // A has stopped, nothing left to hold
                else if (now - track.last_b_time > PAIR_MAX_STALE || now - track.status_since > CANDIDATE_MAX_AGE) {
                    reg.dissolve(i, now, COOLDOWN_SECONDS); // A survives as WAITING
                    n_dissolved++;
                }
                break;
            case TrackStatus::VALIDATED:
                if (now - track.last_a_time > PAIR_MAX_STALE ||
                    now - track.last_b_time > PAIR_MAX_STALE) {
                    writer.push_close_marker(track.global_id, now); // one side has stopped contributing, close the track
                    reg.close(i);
                }
                break;
            default: break;
            }
        });
        // for output log at the end - can remove if unneeded:
        pool_highest_load = std::max<uint64_t>(pool_highest_load, reg.in_use());
    }

    // --- B id reject cache things --- //
    inline bool recently_rejected(uint64_t key, double now) {
        for (const auto& e : reject_cache) if (e.key == key) return now < e.retry_at;
        return false;
    }
    inline void mark_rejected(uint64_t key, double now) {
        for (auto& e : reject_cache) if (e.key == key) { 
            e.retry_at = now + REJECT_RETRY_DT;
            return; 
        }
        reject_cache[reject_next] = {key, now + REJECT_RETRY_DT};
        reject_next = (reject_next + 1) % reject_cache.size(); // overwrite the oldest entry
    }

    // --- Epipolar check: squared Sampson distance between A and B points --- //
    // Returns squared distances for speed - compare against squared threshold below
    inline double sampson_distance_sq(double xa, double ya, double xb, double yb) {
        double l1 = F[0]*xa + F[1]*ya + F[2]; // epipolar line in B
        double l2 = F[3]*xa + F[4]*ya + F[5];
        double l3 = F[6]*xa + F[7]*ya + F[8];

        double m1 = F[0]*xb + F[3]*yb + F[6]; // epipolar line in A
        double m2 = F[1]*xb + F[4]*yb + F[7];

        double numerator = xb*l1 + yb*l2 + l3; // = pB^T F pA
        numerator *= numerator;

        double denominator = l1*l1 + l2*l2 + m1*m1 + m2*m2;
        // optimsied return call for compiler
        return (denominator < 1e-12) ? std::numeric_limits<double>::max() : (numerator / denominator);
    }

    // --- Velocity check: reprojects a 3D point+velocity into one camera and compares --- //
    // against the observed 2D velocity in that camera.
    // Uses the analytical derivative of perspective projection (quotient rule) rather than differentiating w.r.t some dt
    // Returns squared error. Call this once per camera and sum the two errors.
    inline double velocity_reprojection_error_sq(double X, double Y, double Z,
                                                 double Vx, double Vy, double Vz,
                                                 double obs_vx, double obs_vy,
                                                 const Mat34& P, double w) {
        // fail early if the precalculated homogenous denominator doesn't pass
        const double w2 = w * w;
        if (w2 < 1e-12) return std::numeric_limits<double>::max();

        // Projected point
        double u = P[0]*X + P[1]*Y + P[2]*Z + P[3];
        double v = P[4]*X + P[5]*Y + P[6]*Z + P[7];

        // Projected velocity
        double up = P[0]*Vx + P[1]*Vy + P[2]*Vz;
        double vp = P[4]*Vx + P[5]*Vy + P[6]*Vz;
        double wp = P[8]*Vx + P[9]*Vy + P[10]*Vz;

        double pred_vx = (up*w - u*wp) / w2;
        double pred_vy = (vp*w - v*wp) / w2;

        double dx = pred_vx - obs_vx;
        double dy = pred_vy - obs_vy;

        return dx*dx + dy*dy;
    }

    // --- Triangulation to 3D point and velocity --- //
    inline bool triangulate(const RawState& a, const RawState& b, State3D& out, double& out_wA, double& out_wB) {
        Eigen::Matrix<double,4,3> M;
        Eigen::Matrix<double,4,1> rp, rv;

        M(0, 0) = P_A[0] - a.x * P_A[8];
        M(0, 1) = P_A[1] - a.x * P_A[9];
        M(0, 2) = P_A[2] - a.x * P_A[10];
        rp(0)   = a.x * P_A[11] - P_A[3];

        M(1, 0) = P_A[4] - a.y * P_A[8];
        M(1, 1) = P_A[5] - a.y * P_A[9];
        M(1, 2) = P_A[6] - a.y * P_A[10];
        rp(1)   = a.y * P_A[11] - P_A[7];

        M(2, 0) = P_B[0] - b.x * P_B[8];
        M(2, 1) = P_B[1] - b.x * P_B[9];
        M(2, 2) = P_B[2] - b.x * P_B[10];
        rp(2)   = b.x * P_B[11] - P_B[3];

        M(3, 0) = P_B[4] - b.y * P_B[8];
        M(3, 1) = P_B[5] - b.y * P_B[9];
        M(3, 2) = P_B[6] - b.y * P_B[10];
        rp(3)   = b.y * P_B[11] - P_B[7];

        auto qr = M.colPivHouseholderQr();
        Eigen::Vector3d X = qr.solve(rp);

        // depth in each camera at the solved point
        out_wA = P_A[8]*X[0] + P_A[9]*X[1] + P_A[10]*X[2] + P_A[11];
        out_wB = P_B[8]*X[0] + P_B[9]*X[1] + P_B[10]*X[2] + P_B[11];
        if (out_wA <= 0.0 || out_wB <= 0.0) return false; // behind the camera, bad result

        // solve for the 3D velocity vector
        rv << a.vx * out_wA, a.vy * out_wA, b.vx * out_wB, b.vy * out_wB;
        Eigen::Vector3d V = qr.solve(rv);

        // assign the output triangulation to its place and return true
        out = State3D{a.tg, X[0], X[1], X[2], V[0], V[1], V[2]};
        return true;
    }


    // --- Combined epipolar and velocity projection check between an A and B state pair --- //
    // Returns {success, cost}
    inline bool evaluate_pair(const RawState& a_in, const RawState& b_in, MatchResult& result) {
        result.suitable = false;

        // Step 1: Time-align. Advance the older states to the newer one's timestamp
        const double t_common = std::max(a_in.tg, b_in.tg);
        const double dt_a = t_common - a_in.tg; // >= 0, nonzero only if A is older than B
        const double dt_b = t_common - b_in.tg; // >= 0, nonzero only if B is older than A
        if (dt_a + dt_b > MAX_ALIGN_DT) return false; // one of them is always zero

        RawState a = a_in, b = b_in;

        // maths to align the older time to the newer one
        a.x += a.vx * dt_a;
        a.y += a.vy * dt_a;
        a.tg = t_common;
        b.x += b.vx * dt_b;
        b.y += b.vy * dt_b;
        b.tg = t_common;

        // Step 2: Check the epipolar distance and gate on this before continuing
        const double epipolar = sampson_distance_sq(a.x, a.y, b.x, b.y);
        if (epipolar > EPIPOLAR_THRESH_SQ) return false; // if this is too large, don't bother running the more expensive velocity check

        // Step 3: Velocity reprojection error. If the epipolar check is good, then run this
        double wA = 0.0, wB = 0.0;
        if (!triangulate(a, b, result.triangulation, wA, wB)) return false;

        const State3D& t = result.triangulation;
        const double vel_error_sq = velocity_reprojection_error_sq(t.x, t.y, t.z, t.vx, t.vy, t.vz, a.vx, a.vy, P_A, wA)
                                  + velocity_reprojection_error_sq(t.x, t.y, t.z, t.vx, t.vy, t.vz, b.vx, b.vy, P_B, wB);
        if (vel_error_sq > VEL_PROJ_THRESH_SQ) return false; // failed velocity projection

        result.compatibility_score = (epipolar / EPIPOLAR_THRESH_SQ) + (vel_error_sq / VEL_PROJ_THRESH_SQ); // normalised for comparison
        result.suitable = true;
        return true;
    }


    // --- SETUP -- //
    bool setup(const std::string &config_name) {
        // PARSE THE CONFIG //
        try {
            // configs/ is a sister directory of src/ (where this binary is built).
            // See executable_directory()'s comment above if you move the build output elsewhere.
            const std::filesystem::path file_config_path =
                executable_directory() / ".." / "configs" / (config_name + ".yaml");
            if (!std::filesystem::exists(file_config_path)) {
                throw std::runtime_error("config file not found: " + file_config_path.string());
            }
            params = loadThreadCParametersFromYAML(file_config_path.string());
        } catch (const std::exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
            return false; // Fail fast if config is missing/invalid
        }

        F = params.F;
        P_A = params.P_A;
        P_B = params.P_B;


        // Start the writer! //
        if (params.save_file) {
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
            ss << std::put_time(&local_tm, "%d-%m-%Y-%H-%M-%S");
            std::string formatted = ss.str();
        
            if(!writer.start("./track_logs/" + formatted + ".raemot3d")){
                std::cerr << "[Thread C] the writer failed to start ._. .-. ._.\n";
                return false;
            }

        } else {
            std::cout << "[Thread C] Logging to file disabled by config. Skipping raemot3d file creation..." << std::endl;
        }

        /// etc etc

        return true;
    }

    // --- PROCESS TRACK UPDATE LOOP --- //
    void process_track_update(const RawState& state) {
        // Perform a full eval sweep if it's time to do so
        if (state.tg - last_sweep_tg >= SWEEP_INTERVAL) {
            last_sweep_tg = state.tg;
            sweep(state.tg);
        }

        // look up this camera_id and track_id combo in the registry
        uint16_t idx = reg.find(state.cam_id, state.track_id);

        // ---- BRANCH 1: NEW COMBO WE'VE NEVER SEEN ---- //
        if (idx == INVALID_SLOT) {

            // -- If this update was from Camera A: -- //
            if (state.cam_id == CamId::A) {
                // create a new WAITING slot for it
                idx = reg.create_waiting(state.cam_id, state.track_id);
                
                // if the registry is actually full, alert this problem to the user - will need to be fixed!
                if (idx == INVALID_SLOT) {
                    std::cerr << "[Thread C] OVERFLOW! Registry pool is exhausted! -- raise POOL_SIZE!!\n";
                    return;
                }

                // access the new track slot that the registry assigned for us
                Track& track = reg.get(idx);

                // update the track's initial A state info
                track.last_a = state;
                track.last_a_time = state.tg;
            } else {
            // -- If this update was from Camera B: -- //

                // New B: scan all unassigned WAITING A-only tracks for a match

                // And HERE we begin the age-old courting process as a B track speed dates a selection of potential A track suitors
            
                // first, check if this B track id is on the no-date black list
                const uint64_t b_key = make_key(state.cam_id, state.track_id);
                if (recently_rejected(b_key, state.tg)) return;
                // onwards!
                uint16_t best = INVALID_SLOT;
                MatchResult best_match; // starts with a very poor compatibility score

                reg.for_each_active([&](uint16_t suitor_idx, Track& suitor) {
                    // make sure the suitor isn't itself (you can't date yourself)
                    //if (suitor_idx == idx) return; // now handled implicitly
                    // make sure the suitor has status: single
                    if (suitor.status != TrackStatus::WAITING) return;
                    // make sure the suitor is an A, not a B, and also double check it didn't somehow sneak in a partner since that previous check (monogamous bee tracks, thank you very much)
                    //if (!suitor.has_a || suitor.has_b) return; // now handled implicitly
                    // make sure the suitor isn't too old for it (appropriate age gaps are all the rage. They're also very wise)
                    //const double age = state.tg - suitor.last_a_time;
                    //if (age > WAITING_MAX_AGE){
                    //    too_old_suitors.push_back(suitor_idx); // defer removal of this suitor till after the full loop
                    //    return;
                    //} // now handled below

                    // make sure it didn't just date this suitor, and therefore is in a cooldown period (no rebounds back to exes thank you very much!)
                    if (suitor.has_cooldown && suitor.cooldown_key == b_key && state.tg < suitor.cooldown_until) return;
                    
                    // Let's do a compatibility test!
                    MatchResult match_result;
                    // If these two are suitable, and have a better compatibility score than previous suitors, take a note and update their standards before moving on
                    if (evaluate_pair(suitor.last_a, state, match_result) && match_result.compatibility_score < best_match.compatibility_score) {
                        best = suitor_idx;
                        best_match = match_result;
                    }
                });

                // See if anyone in the lineup was actually suitable
                if (best != INVALID_SLOT) {
                    // Pair them up!
                    reg.merge_into_candidate(best, state.track_id, state);
                    Track& track = reg.get(best);
                    track.pass_count = 1; // the match itself counts as an initial success
                    track.push_buffer(best_match.triangulation);
                    track.fresh_a = track.fresh_b = false; // both observations consumed by the match
                } else {
                    // add this B id to the black list for just a lil' bit
                    mark_rejected(b_key, state.tg);
                    return;
                }
                // If it wasn't matched, the data is dropped and we move on to the next one coming in
            }

            return;
        } // END BRANCH 1 (new combo we've never seen)

        // ---- BRANCH 2: EXISTING COMBO WE'VE SEEN BEFORE ---- //
        // we need to update the relevant side, do CANDIDATE buffering or VALIDATED stream to file,
        // and update counters for pass/fail vailidation steps

        // access the track that the registry assigned to this existing combo
        Track& track = reg.get(idx);

        // update the relevant side's last-seen state data
        if (state.cam_id == CamId::A) {
            // Update was from Camera A
            if (state.tg <= track.last_a_time) return; // remove update that are out of time order
            track.last_a = state;
            track.last_a_time = state.tg;
            track.fresh_a = true;
        } else {
            // Update was from Camera B
            if (state.tg <= track.last_b_time) return; // remove update that are out of time order
            track.last_b = state;
            track.last_b_time = state.tg;
            track.fresh_b = true;
        }

        if (track.status == TrackStatus::WAITING) return;
        if (!(track.fresh_a && track.fresh_b)) return; // needs a new observation from both cameras before we evaluate and write out, etc.
        track.fresh_a = track.fresh_b = false;

        // Allocate a single MatchResult here to capture evaluation and triangulation
        MatchResult eval_result;
        const bool passed = evaluate_pair(track.last_a, track.last_b, eval_result);

        // -- Track is CANDIDATE -- //
        if (track.status == TrackStatus::CANDIDATE) {
            // Check if this new state passes or fails the matching test
            if (passed) {
                // YES, it passed!
                track.pass_count++;
                track.fail_count = 0;

                track.push_buffer(eval_result.triangulation);

                // VALIDATION CHECK //
                if (track.pass_count >= CANDIDATE_PASS_THRESHOLD) {
                    reg.promote_to_validated(idx, writer, state.tg);
                    n_validated++;
                }
            } else {
                // NO, it didn't pass.
                track.fail_count++;
                track.pass_count = 0;
                track.buffer_count = 0;

                // REMOVAL CHECK //
                if (track.fail_count >= CANDIDATE_FAIL_THRESHOLD) {
                    reg.dissolve(idx, state.tg, COOLDOWN_SECONDS);
                    n_dissolved++;
                }
            }
            
        } // end candidate track branch
        else if (track.status == TrackStatus::VALIDATED) {
        // -- Track is VALIDATED -- //
            
            // If it passed evaluation, pass the state to the writer
            if (passed) {
                writer.push(track.global_id, eval_result.triangulation);
                track.fail_count = 0;
            } else if (++track.fail_count >= VALIDATED_FAIL_THRESHOLD) {
                writer.push_close_marker(track.global_id, state.tg);
                reg.close(idx); // end and clear this track slot
            }

        } // end validated track branch

        // END BRANCH 2 (existing combo we've seen before)

    }

    // --- TEARDOWN --- //
    void teardown() {
        // drain any VALIDATED tracks on teardown
        reg.for_each_active([&](uint16_t i, Track& track) {
            if (track.status == TrackStatus::VALIDATED) {
                writer.push_close_marker(track.global_id, std::max(track.last_a_time, track.last_b_time));
            }
            reg.close(i);
        });

        // stop the writer
        writer.stop();
        std::cerr << "[Thread C writer] wrote " << writer.written_records() << ", dropped " << writer.dropped_records() << "\n";

        // output some performance notes to terminal log
        std::cerr << "[Thread C writer] wrote " << writer.written_records()
                  << ", dropped " << writer.dropped_records() << "\n";
        std::cerr << "[Thread C] validated=" << n_validated
                  << " dissolved=" << n_dissolved
                  << " pool_highest_load=" << pool_highest_load
                  << "/" << POOL_SIZE << "\n";
    }   
} // namespace threadC