#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>

#include "robot_comm/interface_name.h"

namespace robot_comm
{
    /**
     * The name of a bus, as written in the URDF (`<param name="bus">`):
     *
     *   usb_rs485:///dev/serial/by-id/usb-FTDI_..?baud=115200   USB-RS485 adapter (Modbus RTU devices)
     *   usb_serial:///dev/ttyUSB1?baud=2500000                USB serial adapter (vendor protocols)
     *   arm_rs485://192.168.1.190/left/port1                  RS485 port 1 at the left arm's flange
     *   arm_canfd://192.168.1.190/right                       CAN FD port at the right arm's flange
     *   usb_can://can0                                        USB-CAN interface
     *
     * Options (after '?', usb_rs485 / usb_serial only): baud (default 115200), low_latency (1 / 0, default 1).
     */
    struct BusName
    {
        std::string scheme;  ///< usb_rs485, usb_serial, arm_rs485, arm_canfd, usb_can
        std::string device;  ///< usb_rs485 / usb_serial: device path; usb_can: interface name
        std::string host;    ///< arm_*: address of the arm controller
        std::string side;    ///< arm_*: left / right
        std::string port;    ///< arm_rs485: port1 / port2
        std::map<std::string, std::string> options;

        bool isUsbSerial() const { return scheme == "usb_rs485" || scheme == "usb_serial"; }
        bool isArm() const { return scheme == "arm_rs485" || scheme == "arm_canfd"; }

        /** The physical line: two names with the same key are the same bus (a USB port is one line whatever the scheme). */
        std::string key() const
        {
            if (isUsbSerial()) return "usb:" + device;
            if (isArm()) return scheme + "://" + host + "/" + side + (port.empty() ? "" : "/" + port);
            return scheme + "://" + device;
        }

        /** The name with its options, in a fixed order. */
        std::string text() const
        {
            std::string out = isArm() ? key() : scheme + "://" + device;
            char sep = '?';
            for (const auto& [k, v] : options)
            {
                out += sep + k + "=" + v;
                sep = '&';
            }
            return out;
        }

        int baud() const { return std::atoi(options.at("baud").c_str()); }
        bool lowLatency() const { return options.at("low_latency") == "1"; }
    };

    namespace detail
    {
        inline bool isHostName(const std::string& s)
        {
            if (s.empty() || s.size() > 253) return false;
            return std::all_of(s.begin(), s.end(), [](char c) {
                return std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-';
            });
        }

        inline bool isPositiveInt(const std::string& s)
        {
            if (s.empty() || s.size() > 9) return false;
            if (!std::all_of(s.begin(), s.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
            {
                return false;
            }
            return std::atoi(s.c_str()) > 0;
        }

        inline bool parseOptions(const std::string& query, std::map<std::string, std::string>& out, std::string& error)
        {
            std::size_t start = 0;
            while (start <= query.size())
            {
                const std::size_t end = std::min(query.find('&', start), query.size());
                const std::string item = query.substr(start, end - start);
                const std::size_t eq = item.find('=');
                if (item.empty() || eq == std::string::npos || eq == 0)
                {
                    error = "malformed option '" + item + "' (expected key=value)";
                    return false;
                }
                const std::string key = item.substr(0, eq);
                const std::string value = item.substr(eq + 1);
                if (key == "baud")
                {
                    if (!isPositiveInt(value))
                    {
                        error = "baud must be a positive integer, got '" + value + "'";
                        return false;
                    }
                }
                else if (key == "low_latency")
                {
                    if (value != "0" && value != "1")
                    {
                        error = "low_latency must be 0 or 1, got '" + value + "'";
                        return false;
                    }
                }
                else
                {
                    error = "unknown option '" + key + "' (known: baud, low_latency)";
                    return false;
                }
                out[key] = value;
                start = end + 1;
            }
            return true;
        }
    }  // namespace detail

    /** Parse a bus name; on failure returns nullopt and a message that names the offending part. */
    inline std::optional<BusName> parseBusName(const std::string& text, std::string& error)
    {
        const auto fail = [&](const std::string& why) -> std::optional<BusName> {
            error = "bus name '" + text + "': " + why;
            return std::nullopt;
        };
        const std::size_t sep = text.find("://");
        if (sep == std::string::npos) return fail("no '<kind>://' prefix");

        BusName name;
        name.scheme = text.substr(0, sep);
        std::string rest = text.substr(sep + 3);
        std::string query;
        if (const std::size_t q = rest.find('?'); q != std::string::npos)
        {
            query = rest.substr(q + 1);
            rest = rest.substr(0, q);
        }

        if (name.isUsbSerial())
        {
            if (rest.empty() || rest.front() != '/') return fail("expected an absolute device path, e.g. /dev/ttyUSB0");
            name.device = rest;
            name.options = {{"baud", "115200"}, {"low_latency", "1"}};
            if (!query.empty())
            {
                std::string why;
                if (!detail::parseOptions(query, name.options, why)) return fail(why);
            }
            return name;
        }
        if (!query.empty()) return fail("options are only allowed for usb_rs485 / usb_serial");
        if (name.scheme == "usb_can")
        {
            if (!isValidInterfaceName(rest)) return fail("'" + rest + "' is not a valid interface name");
            name.device = rest;
            return name;
        }
        if (name.isArm())
        {
            const std::size_t a = rest.find('/');
            if (a == std::string::npos) return fail("expected <arm address>/<left|right>");
            name.host = rest.substr(0, a);
            const std::string tail = rest.substr(a + 1);
            const std::size_t b = tail.find('/');
            name.side = tail.substr(0, b);
            name.port = (b == std::string::npos) ? "" : tail.substr(b + 1);
            if (!detail::isHostName(name.host)) return fail("'" + name.host + "' is not an address");
            if (name.side != "left" && name.side != "right")
            {
                return fail("side must be left or right, got '" + name.side + "'");
            }
            if (name.scheme == "arm_rs485" && name.port != "port1" && name.port != "port2")
            {
                return fail("arm_rs485 needs a port: port1 or port2, got '" + name.port + "'");
            }
            if (name.scheme == "arm_canfd" && !name.port.empty()) return fail("arm_canfd has no port");
            return name;
        }
        return fail("unknown kind '" + name.scheme + "' (known: usb_rs485, usb_serial, arm_rs485, arm_canfd, usb_can)");
    }
}  // namespace robot_comm
