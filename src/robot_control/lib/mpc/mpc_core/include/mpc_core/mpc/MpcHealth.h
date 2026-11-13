#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <optional>
#include <string>

namespace ocs2::mpc_core
{
    /**
     * Solve-time and failure counters of an MPC update thread, summarised once per window.
     *
     * Only for the MPC thread (not thread safe and never called from the realtime control loop), so it
     * adds two clock reads per solve and one log line per window. A window with an overrun (a solve slower
     * than the MPC period) or a failure is reported as a warning, which is what to look for after an incident.
     */
    class MpcHealth
    {
    public:
        using Clock = std::chrono::steady_clock;

        struct Report
        {
            std::string text;
            bool healthy;
        };

        explicit MpcHealth(double period_s, double window_s = 10.0)
            : period_s_(period_s), window_s_(window_s), window_start_(Clock::now())
        {
        }

        void recordSolve(double solve_s)
        {
            ++solves_;
            total_s_ += solve_s;
            max_s_ = std::max(max_s_, solve_s);
            if (period_s_ > 0.0 && solve_s > period_s_)
            {
                ++overruns_;
            }
        }

        void recordFailure() { ++failures_; }

        /// The summary of the window that just ended, or nothing while the window is still running.
        std::optional<Report> report(Clock::time_point now)
        {
            const double elapsed_s = std::chrono::duration<double>(now - window_start_).count();
            if (elapsed_s < window_s_)
            {
                return std::nullopt;
            }
            char text[208];
            const double average_ms = solves_ > 0 ? 1000.0 * total_s_ / static_cast<double>(solves_) : 0.0;
            const bool healthy = overruns_ == 0 && failures_ == 0;
            // A healthy window is routine; only a window with an overrun or a failure carries the fault code.
            std::snprintf(text, sizeof(text),
                          "%sMPC health (%.1f s): %zu solves, avg %.2f ms, max %.2f ms, period %.2f ms, overruns %zu, failures %zu",
                          healthy ? "" : "[CTL-002] ",
                          elapsed_s, solves_, average_ms, 1000.0 * max_s_, 1000.0 * period_s_, overruns_, failures_);
            const Report result{text, healthy};
            solves_ = overruns_ = failures_ = 0;
            total_s_ = max_s_ = 0.0;
            window_start_ = now;
            return result;
        }

    private:
        double period_s_;
        double window_s_;
        Clock::time_point window_start_;
        std::size_t solves_{0};
        std::size_t overruns_{0};
        std::size_t failures_{0};
        double total_s_{0.0};
        double max_s_{0.0};
    };
} // namespace ocs2::mpc_core
