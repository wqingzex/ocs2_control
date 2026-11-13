#include "robot_comm_serial/usb_latency.h"

#include <filesystem>
#include <fstream>

namespace robot_comm
{
    std::vector<Notice> configureUsbLatency(const std::string& device)
    {
        namespace fs = std::filesystem;
        std::vector<Notice> notices;
        const auto warn = [&](std::string text) { notices.push_back({Notice::Level::Warn, std::move(text)}); };

        std::error_code ec;
        const auto resolved_port = fs::canonical(device, ec);
        if (ec)
        {
            warn("Cannot resolve serial port '" + device + "' while checking FTDI latency_timer: " + ec.message());
            return notices;
        }

        const auto latency_path = fs::path("/sys/bus/usb-serial/devices") / resolved_port.filename() / "latency_timer";
        if (!fs::exists(latency_path, ec) || ec)
        {
            return notices; // not a USB serial adapter with a latency timer
        }

        int previous = -1;
        {
            std::ifstream input(latency_path);
            input >> previous;
        }
        if (previous == 1)
        {
            return notices;
        }

        {
            std::ofstream output(latency_path);
            if (!output || !(output << 1 << '\n' << std::flush))
            {
                warn("Unable to set '" + latency_path.string() + "' to 1 ms; continuing with latency_timer=" +
                     std::to_string(previous) + " ms (check sysfs/udev permissions)");
                return notices;
            }
        }

        int verified = -1;
        {
            std::ifstream input(latency_path);
            input >> verified;
        }
        if (verified == 1)
        {
            notices.push_back({Notice::Level::Info, "FTDI latency_timer changed from " + std::to_string(previous) +
                                                        " ms to 1 ms for '" + resolved_port.string() + "'"});
        }
        else
        {
            warn("FTDI latency_timer verification failed for '" + resolved_port.string() + "' (read back " +
                 std::to_string(verified) + " ms)");
        }
        return notices;
    }
} // namespace robot_comm
