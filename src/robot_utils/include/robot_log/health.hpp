#pragma once

// Per-hop health of the chain, cheap enough to leave on all the time.
//
// A component counts what it does in a CycleStats or a RateStats: a few relaxed atomic operations per event, no lock, no
// allocation, no logging, so it is safe inside a real-time loop. A timer on the component's node (not the real-time
// thread) takes the numbers once per window and writes one line:
//
//   [HEALTH] arm_controller window=10.0s cycles=5000 period_avg_ms=2.00 period_max_ms=2.31 late=0 duration_avg_us=120 duration_max_us=310
//
// The window is 10 s in normal mode and 1 s in debug. The line is plain INFO while the hop is healthy; when it is not
// (late cycles, a gap in the data) it goes out as WARN with the fault code of that condition, which is what the incident
// watcher and a reader of the log look for.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "robot_log/log_mode.hpp"

namespace robot_log
{
    /// Length of a health window: 10 s in normal mode, 1 s in debug.
    inline double healthWindowSeconds()
    {
        return debugMode() ? 1.0 : 10.0;
    }

    inline std::uint64_t steadyNowNs()
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    namespace detail
    {
        inline void atomicMax(std::atomic<std::uint64_t>& target, std::uint64_t value)
        {
            std::uint64_t seen = target.load(std::memory_order_relaxed);
            while (value > seen && !target.compare_exchange_weak(seen, value, std::memory_order_relaxed))
            {
            }
        }
    } // namespace detail

    // --- a loop that runs on a period: a control cycle, a read/write cycle ---------------------------------------
    struct CycleSnapshot
    {
        std::uint64_t cycles{0};
        std::uint64_t late{0};
        double period_avg_ms{0.0};
        double period_max_ms{0.0};
        double duration_avg_us{0.0};
        double duration_max_us{0.0};
    };

    /// One writer (the loop), one reader (the reporter). record() is safe in a real-time thread.
    class CycleStats
    {
    public:
        /// period_ns: time since the previous cycle began; duration_ns: how long this cycle took;
        /// late_period_ns: a period longer than this counts as a late cycle.
        void record(std::uint64_t period_ns, std::uint64_t duration_ns, std::uint64_t late_period_ns)
        {
            cycles_.fetch_add(1, std::memory_order_relaxed);
            period_sum_ns_.fetch_add(period_ns, std::memory_order_relaxed);
            duration_sum_ns_.fetch_add(duration_ns, std::memory_order_relaxed);
            detail::atomicMax(period_max_ns_, period_ns);
            detail::atomicMax(duration_max_ns_, duration_ns);
            if (period_ns > late_period_ns)
            {
                late_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        /// The numbers since the last take(), and start a new window.
        CycleSnapshot take()
        {
            CycleSnapshot out;
            out.cycles = cycles_.exchange(0, std::memory_order_relaxed);
            out.late = late_.exchange(0, std::memory_order_relaxed);
            const std::uint64_t period_sum = period_sum_ns_.exchange(0, std::memory_order_relaxed);
            const std::uint64_t duration_sum = duration_sum_ns_.exchange(0, std::memory_order_relaxed);
            out.period_max_ms = static_cast<double>(period_max_ns_.exchange(0, std::memory_order_relaxed)) / 1.0e6;
            out.duration_max_us = static_cast<double>(duration_max_ns_.exchange(0, std::memory_order_relaxed)) / 1.0e3;
            if (out.cycles > 0)
            {
                out.period_avg_ms = static_cast<double>(period_sum) / static_cast<double>(out.cycles) / 1.0e6;
                out.duration_avg_us = static_cast<double>(duration_sum) / static_cast<double>(out.cycles) / 1.0e3;
            }
            return out;
        }

    private:
        std::atomic<std::uint64_t> cycles_{0};
        std::atomic<std::uint64_t> late_{0};
        std::atomic<std::uint64_t> period_sum_ns_{0};
        std::atomic<std::uint64_t> period_max_ns_{0};
        std::atomic<std::uint64_t> duration_sum_ns_{0};
        std::atomic<std::uint64_t> duration_max_ns_{0};
    };

    /// Times one call of a loop. Make one at the top of the function; whichever way it returns, the destructor records
    /// the period since the previous call began and how long this call took. last_begin_ns is the loop's own memory of
    /// when the previous call began (it starts at 0: the first call has nothing to measure from). A late_period_ns of 0
    /// switches the recording off (the nominal period is not known yet).
    class CycleTimer
    {
    public:
        CycleTimer(CycleStats& stats, std::uint64_t& last_begin_ns, std::uint64_t late_period_ns)
            : stats_(stats), last_begin_ns_(last_begin_ns), late_period_ns_(late_period_ns), begin_ns_(steadyNowNs())
        {
        }

        ~CycleTimer()
        {
            if (late_period_ns_ != 0 && last_begin_ns_ != 0)
            {
                stats_.record(begin_ns_ - last_begin_ns_, steadyNowNs() - begin_ns_, late_period_ns_);
            }
            last_begin_ns_ = begin_ns_;
        }

        CycleTimer(const CycleTimer&) = delete;
        CycleTimer& operator=(const CycleTimer&) = delete;

    private:
        CycleStats& stats_;
        std::uint64_t& last_begin_ns_;
        std::uint64_t late_period_ns_;
        std::uint64_t begin_ns_;
    };

    // --- data that arrives at some rate: headset frames, published targets --------------------------------------
    struct RateSnapshot
    {
        std::uint64_t count{0};
        double rate_hz{0.0};
        double gap_max_ms{0.0};
    };

    /// One writer (whoever receives or publishes), one reader (the reporter).
    class RateStats
    {
    public:
        /// An item arrived / was sent now.
        void record() { recordAt(steadyNowNs()); }

        void recordAt(std::uint64_t now_ns)
        {
            const std::uint64_t previous = last_ns_.exchange(now_ns, std::memory_order_relaxed);
            count_.fetch_add(1, std::memory_order_relaxed);
            if (previous != 0 && now_ns > previous)
            {
                detail::atomicMax(gap_max_ns_, now_ns - previous);
            }
        }

        /// The numbers since the last take(), over a window of window_s seconds, and start a new window.
        RateSnapshot take(double window_s)
        {
            RateSnapshot out;
            out.count = count_.exchange(0, std::memory_order_relaxed);
            out.gap_max_ms = static_cast<double>(gap_max_ns_.exchange(0, std::memory_order_relaxed)) / 1.0e6;
            out.rate_hz = window_s > 0.0 ? static_cast<double>(out.count) / window_s : 0.0;
            return out;
        }

    private:
        std::atomic<std::uint64_t> count_{0};
        std::atomic<std::uint64_t> last_ns_{0};
        std::atomic<std::uint64_t> gap_max_ns_{0};
    };

    // --- the lines ---------------------------------------------------------------------------------------------
    inline std::string formatCycleLine(const std::string& name, double window_s, const CycleSnapshot& s)
    {
        char text[256];
        std::snprintf(text, sizeof(text),
                      "[HEALTH] %s window=%.1fs cycles=%llu period_avg_ms=%.2f period_max_ms=%.2f late=%llu "
                      "duration_avg_us=%.0f duration_max_us=%.0f",
                      name.c_str(), window_s, static_cast<unsigned long long>(s.cycles), s.period_avg_ms, s.period_max_ms,
                      static_cast<unsigned long long>(s.late), s.duration_avg_us, s.duration_max_us);
        return text;
    }

    inline std::string formatRateLine(const std::string& name, double window_s, const RateSnapshot& s,
                                      const std::string& extra = "")
    {
        char text[256];
        std::snprintf(text, sizeof(text), "[HEALTH] %s window=%.1fs count=%llu rate_hz=%.1f gap_max_ms=%.1f%s%s",
                      name.c_str(), window_s, static_cast<unsigned long long>(s.count), s.rate_hz, s.gap_max_ms,
                      extra.empty() ? "" : " ", extra.c_str());
        return text;
    }

    /// Writes the cycle line: INFO when every cycle was on time, WARN with the fault code when some were late.
    inline void logCycleHealth(const rclcpp::Logger& logger, const std::string& name, double window_s,
                               const CycleSnapshot& s, const char* late_fault_code)
    {
        const std::string line = formatCycleLine(name, window_s, s);
        if (s.late > 0)
        {
            RCLCPP_WARN(logger, "[%s] %s", late_fault_code, line.c_str());
        }
        else
        {
            RCLCPP_INFO(logger, "%s", line.c_str());
        }
    }
} // namespace robot_log
