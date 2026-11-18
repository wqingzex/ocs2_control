#include "end_effector_hardware/bus/bus_probe.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>

namespace end_effector_hardware
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        Clock::duration seconds(double s)
        {
            return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
        }

        double millisBetween(Clock::time_point from, Clock::time_point to)
        {
            return std::chrono::duration<double, std::milli>(to - from).count();
        }

        struct Outcomes
        {
            uint64_t ok = 0, timeouts = 0, errors = 0;
            uint64_t total() const { return ok + timeouts + errors; }
        };

        Outcomes outcomesOf(const robot_comm::BusStats& s) { return Outcomes{s.ok, s.timeouts, s.errors}; }
    }  // namespace

    std::vector<std::string> defaultJoints(const std::string& device, const std::string& side)
    {
        const std::string prefix = side + "_hand_";
        std::vector<std::string> names;
        if (device == "jodell_rg75")
        {
            names.push_back(side + "_gripper_joint");
        }
        else if (device == "linkerhand_l6")
        {
            for (const char* joint : {"thumb_joint1", "thumb_joint2", "index_joint", "middle_joint", "ring_joint", "pinky_joint"})
                names.push_back(prefix + joint);
        }
        else if (device == "xhand1")
        {
            for (const char* joint : {"thumb_joint1", "thumb_joint2", "thumb_joint3", "index_joint1", "index_joint2",
                                      "index_joint3", "middle_joint1", "middle_joint2", "ring_joint1", "ring_joint2",
                                      "pinky_joint1", "pinky_joint2"})
                names.push_back(prefix + joint);
        }
        return names;
    }

    Params defaultParams(const std::string& /*device*/, const std::string& side) { return Params{{"side", side}}; }

    int defaultBaud(const std::string& device)
    {
        if (device == "linkerhand_l6") return 115200;  // external_ee_systems.xacro
        if (device == "xhand1") return 3000000;        // xhand1_description side_systems.xacro
        return 0;
    }

    double percentile(const std::vector<double>& sorted, double p)
    {
        if (sorted.empty()) return 0.0;
        const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(sorted.size())));
        return sorted[std::min(sorted.size() - 1, rank == 0 ? 0 : rank - 1)];
    }

    ProbeReport runProbe(BusDevice& device, const std::shared_ptr<robot_comm::BusClient>& bus, const ProbeOptions& options,
                         const std::atomic<bool>* stop)
    {
        ProbeReport report;
        device.attach(bus, options.io_mode);
        device.start();

        const auto begin = Clock::now();
        const auto measure_from = begin + seconds(options.warmup_s);
        const auto end = measure_from + seconds(options.duration_s);
        const auto period = options.cycle_hz > 0.0 ? seconds(1.0 / options.cycle_hz) : Clock::duration::zero();

        bool measuring = false;
        Clock::time_point measure_start = measure_from;
        Clock::time_point last_done = begin;
        bool has_last_done = false;
        Outcomes seen = outcomesOf(bus->stats());
        bool pending = false;  // a request of the device is on the bus, its outcome not seen yet
        Clock::time_point sent_at = begin;

        // Look at what the bus finished since the last look; the time of a finished request is "now".
        const auto observe = [&] {
            const Outcomes now_outcomes = outcomesOf(bus->stats());
            if (now_outcomes.total() == seen.total()) return;
            const auto now = Clock::now();
            const Outcomes delta{now_outcomes.ok - seen.ok, now_outcomes.timeouts - seen.timeouts,
                                 now_outcomes.errors - seen.errors};
            seen = now_outcomes;
            if (measuring)
            {
                report.ok += delta.ok;
                report.timeouts += delta.timeouts;
                report.errors += delta.errors;
                report.requests += delta.total();
                if (delta.ok > 0 && pending) report.round_trip_ms.push_back(millisBetween(sent_at, now));
                if (has_last_done && millisBetween(last_done, now) > options.max_gap_ms) ++report.gaps;
                last_done = now;
                has_last_done = true;
            }
            pending = false;
        };

        auto next_step = begin;
        while (true)
        {
            const auto now = Clock::now();
            if (now >= end || (stop && stop->load()) || device.failed()) break;
            if (!measuring && now >= measure_from)
            {
                measuring = true;
                measure_start = now;
                has_last_done = false;
            }
            if (now >= next_step)
            {
                next_step = period == Clock::duration::zero() ? now : std::max(next_step + period, now - period);
                const auto before = Clock::now();
                const uint64_t sent_before = bus->stats().sent;
                const uint64_t finished_before = seen.total();
                device.step();
                // A new request is on the bus (or already finished) when the counters moved or the bus is busy.
                if (!pending && (bus->stats().sent > sent_before || bus->state() == robot_comm::BusState::Busy ||
                                 outcomesOf(bus->stats()).total() > finished_before))
                {
                    pending = true;
                    sent_at = before;
                }
                observe();
            }
            device.collect();
            observe();
            std::this_thread::yield();
        }

        const auto stopped = Clock::now();
        report.seconds = measuring ? std::chrono::duration<double>(stopped - measure_start).count() : 0.0;
        report.hz = report.seconds > 0.0 ? static_cast<double>(report.requests) / report.seconds : 0.0;
        std::vector<double> sorted = report.round_trip_ms;
        std::sort(sorted.begin(), sorted.end());
        report.p50_ms = percentile(sorted, 0.50);
        report.p95_ms = percentile(sorted, 0.95);
        report.p99_ms = percentile(sorted, 0.99);
        report.max_ms = sorted.empty() ? 0.0 : sorted.back();
        report.ready = device.ready();
        report.failed = device.failed();
        report.failure = device.failure();
        report.online = device.online();
        return report;
    }

    std::string formatReport(const ProbeReport& r, const std::string& title)
    {
        char line[256];
        std::string out = "\n=== " + title + " ===\n";
        std::snprintf(line, sizeof line, "device: %s%s%s; online at the end: %s\n", r.ready ? "ready" : "NOT ready",
                      r.failed ? ", FAILED: " : "", r.failed ? r.failure.c_str() : "", r.online ? "yes" : "no");
        out += line;
        std::snprintf(line, sizeof line,
                      "requests %llu in %.1f s (%.1f Hz): answered %llu, timeouts %llu, errors %llu, gaps > limit %llu\n",
                      static_cast<unsigned long long>(r.requests), r.seconds, r.hz, static_cast<unsigned long long>(r.ok),
                      static_cast<unsigned long long>(r.timeouts), static_cast<unsigned long long>(r.errors),
                      static_cast<unsigned long long>(r.gaps));
        out += line;
        if (!r.round_trip_ms.empty())
        {
            std::snprintf(line, sizeof line, "round trip ms: p50 %.2f  p95 %.2f  p99 %.2f  max %.2f\n", r.p50_ms, r.p95_ms,
                          r.p99_ms, r.max_ms);
            out += line;
        }
        return out;
    }
}  // namespace end_effector_hardware
