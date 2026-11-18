/**
 * Measures a robot_comm bus through the end effector that uses it: round trip, loss, request rate and gaps.
 * Usually started through `./scripts/start.sh bus-probe`, which takes the device and port from robot.local.yaml.
 *
 *   bus_probe --device linkerhand_l6|xhand1 --side left|right (--port /dev/serial/by-id/... | --bus URI) [options]
 *
 * The device runs its own state machine on the bus (the same initialization and polling as under the controller),
 * so it receives the frames it receives at start-up: it holds its position, nothing commands a movement. Buses of an
 * arm's flange (arm_rs485://, arm_canfd://) exist only inside the arm driver and are not supported here yet.
 */
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <rclcpp/logger.hpp>

#include "end_effector_hardware/bus/bus_device.h"
#include "end_effector_hardware/bus/bus_probe.h"
#include "robot_comm/bus_name.h"
#include "robot_comm_serial/serial_bus.h"

namespace
{
    std::atomic<bool> g_stop{false};
    void onSignal(int) { g_stop.store(true); }

    struct Options
    {
        std::string device;
        std::string side = "left";
        std::string bus;
        std::string port;
        std::string csv;
        end_effector_hardware::ProbeOptions probe;
        bool yes = false;
    };

    void usage()
    {
        std::puts(
            "bus_probe --device linkerhand_l6|xhand1 [--side left|right] (--port DEVICE | --bus URI) [options]\n"
            "  --port DEVICE        serial device, /dev/serial/by-id/...; the bus is usb_rs485://DEVICE?baud=<device default>\n"
            "  --bus URI            a bus name instead of --port (usb_rs485://..., usb_serial://...)\n"
            "  --cycle-hz N         controller rate to emulate (default 0 = as fast as the bus answers)\n"
            "  --duration S         seconds measured (default 10)\n"
            "  --warmup S           seconds before measuring, for the device's initialization (default 2)\n"
            "  --max-gap-ms N       a pause longer than this between two finished requests is a gap (default 100)\n"
            "  --io-mode async|sync how the device waits for replies (default async)\n"
            "  --csv FILE           write every answered round trip (ms) to FILE\n"
            "  --yes                do not ask for confirmation");
    }

    bool parse(int argc, char** argv, Options& o)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string key = argv[i];
            if (key == "--yes")
            {
                o.yes = true;
                continue;
            }
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "missing value for %s\n", key.c_str());
                return false;
            }
            const std::string v = argv[++i];
            if (key == "--device") o.device = v;
            else if (key == "--side") o.side = v;
            else if (key == "--bus") o.bus = v;
            else if (key == "--port") o.port = v;
            else if (key == "--csv") o.csv = v;
            else if (key == "--cycle-hz") o.probe.cycle_hz = std::atof(v.c_str());
            else if (key == "--duration") o.probe.duration_s = std::atof(v.c_str());
            else if (key == "--warmup") o.probe.warmup_s = std::atof(v.c_str());
            else if (key == "--max-gap-ms") o.probe.max_gap_ms = std::atof(v.c_str());
            else if (key == "--io-mode")
            {
                if (v != "async" && v != "sync")
                {
                    std::fprintf(stderr, "--io-mode must be async or sync\n");
                    return false;
                }
                o.probe.io_mode = v == "sync" ? robot_comm::IoMode::Sync : robot_comm::IoMode::Async;
            }
            else
            {
                std::fprintf(stderr, "unknown option %s\n", key.c_str());
                return false;
            }
        }
        if (o.device.empty() || (o.side != "left" && o.side != "right")) return false;
        if (o.bus.empty() == o.port.empty())
        {
            std::fprintf(stderr, "give exactly one of --port and --bus\n");
            return false;
        }
        return o.probe.cycle_hz >= 0.0 && o.probe.duration_s > 0.0 && o.probe.warmup_s >= 0.0 && o.probe.max_gap_ms > 0.0;
    }

    bool confirmed(const Options& o, const std::string& bus)
    {
        std::printf("\nprobe %s (%s hand) on %s for %.0f s.\n", o.device.c_str(), o.side.c_str(), bus.c_str(),
                    o.probe.duration_s);
        std::puts("The device gets the frames it gets when the controller starts (it holds its position); nothing");
        std::puts("commands a movement. Stop the controller first: only one process may use the serial port.");
        if (o.yes) return true;
        std::printf("Type YES to continue: ");
        std::string answer;
        std::getline(std::cin, answer);
        return answer == "YES";
    }
}  // namespace

int main(int argc, char** argv)
{
    using namespace end_effector_hardware;
    Options options;
    if (!parse(argc, argv, options))
    {
        usage();
        return 2;
    }

    if (options.bus.empty())
    {
        const int baud = defaultBaud(options.device);
        if (baud == 0)
        {
            std::fprintf(stderr, "%s has no USB-RS485 default baud rate: pass --bus\n", options.device.c_str());
            return 2;
        }
        options.bus = "usb_rs485://" + options.port + "?baud=" + std::to_string(baud);
    }
    std::string error;
    const auto name = robot_comm::parseBusName(options.bus, error);
    if (!name)
    {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    if (name->isArm())
    {
        std::fprintf(stderr,
                     "%s: a bus at an arm's flange exists only while the arm driver runs; this tool opens USB buses only\n",
                     options.bus.c_str());
        return 2;
    }

    auto device = makeBusDevice(options.device, rclcpp::get_logger("bus_probe"), error);
    if (!device || !device->configure(defaultJoints(options.device, options.side),
                                      defaultParams(options.device, options.side), error))
    {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    if (!confirmed(options, name->text()))
    {
        std::puts("not confirmed, nothing sent");
        return 1;
    }

    robot_comm::registerSerialBuses();
    const auto client = robot_comm::BusHub::instance().connect(options.bus, error);
    if (!client)
    {
        std::fprintf(stderr, "cannot open %s: %s\n", options.bus.c_str(), error.c_str());
        return 1;
    }

    std::printf("measuring %.0f s (+ %.0f s warm-up)...\n", options.probe.duration_s, options.probe.warmup_s);
    const ProbeReport report = runProbe(*device, client, options.probe, &g_stop);
    std::printf("%s", formatReport(report, options.device + " " + options.side + " on " + name->text()).c_str());

    if (!options.csv.empty())
    {
        std::ofstream out(options.csv);
        if (!out)
        {
            std::fprintf(stderr, "cannot write %s\n", options.csv.c_str());
            return 1;
        }
        out << "index,rtt_ms\n";
        for (std::size_t i = 0; i < report.round_trip_ms.size(); ++i) out << i << ',' << report.round_trip_ms[i] << '\n';
        std::printf("wrote %s\n", options.csv.c_str());
    }
    return report.ready && !report.failed ? 0 : 1;
}
