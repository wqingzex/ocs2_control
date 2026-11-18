#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "end_effector_hardware/bus/bus_device.h"
#include "robot_comm/bus_hub.h"

namespace end_effector_hardware
{
    /**
     * Measures a bus through the device that will really use it: the device's own state machine (BusDevice) runs
     * against the bus at a controller-like rate, so the requests are the ones that device sends (L6 reads, XHand1
     * zero-torque probe frames, RG75 activation and status reads), not a made-up frame.
     *
     * A request is timed from the step() that put it on the bus to the step() / collect() after which the bus reports
     * it finished, so the round trip has the resolution of the polling loop (tens of microseconds).
     */
    struct ProbeOptions
    {
        double cycle_hz = 0.0;       ///< controller rate to emulate; 0 = as fast as the bus answers
        double duration_s = 10.0;
        double max_gap_ms = 100.0;   ///< a pause between two finished requests longer than this counts as a gap
        double warmup_s = 2.0;       ///< not measured: lets the device finish its initialization
        robot_comm::IoMode io_mode = robot_comm::IoMode::Async;
    };

    struct ProbeReport
    {
        bool ready = false;          ///< the device read its first state
        bool failed = false;
        std::string failure;
        bool online = true;          ///< at the end
        uint64_t requests = 0;       ///< finished requests in the measured time
        uint64_t ok = 0;
        uint64_t timeouts = 0;
        uint64_t errors = 0;
        double seconds = 0.0;
        double hz = 0.0;             ///< finished requests per second
        double p50_ms = 0.0, p95_ms = 0.0, p99_ms = 0.0, max_ms = 0.0;  ///< round trip of the answered requests
        uint64_t gaps = 0;
        std::vector<double> round_trip_ms;  ///< every answered request, in order
    };

    /** The joints a device needs to be configured without a URDF (names follow the description packages). */
    std::vector<std::string> defaultJoints(const std::string& device, const std::string& side);
    /** The URDF parameters the device needs besides the bus (side, ...). */
    Params defaultParams(const std::string& device, const std::string& side);
    /** The baud rate the device's description package opens the adapter with; 0 for an unknown device. */
    int defaultBaud(const std::string& device);

    /** Ascending `sorted`, p in [0, 1]. */
    double percentile(const std::vector<double>& sorted, double p);

    /**
     * Run `device` on `bus` for warmup + duration. The device must already be configured; this attaches it and starts
     * it. `stop` (optional) ends the run early (Ctrl+C).
     */
    ProbeReport runProbe(BusDevice& device, const std::shared_ptr<robot_comm::BusClient>& bus, const ProbeOptions& options,
                         const std::atomic<bool>* stop = nullptr);

    std::string formatReport(const ProbeReport& report, const std::string& title);
}  // namespace end_effector_hardware
