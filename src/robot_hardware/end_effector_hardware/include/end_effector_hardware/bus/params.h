#pragma once

#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <system_error>

#include "end_effector_hardware/bus/bus_device.h"

// Hardware parameters from the URDF. Parsed without exceptions: the whole text must be the number, anything else is
// reported (with the parameter name) and fails on_init, never replaced by a default.
namespace end_effector_hardware::params
{
    /** The whole of `text` as a finite double; nullopt otherwise ("0.5x", "", "nan"). */
    inline std::optional<double> toDouble(const std::string& text)
    {
        double v = 0.0;
        const char* end = text.data() + text.size();
        const auto [ptr, ec] = std::from_chars(text.data(), end, v);
        if (ec != std::errc() || ptr != end || !std::isfinite(v)) return std::nullopt;
        return v;
    }

    /** The whole of `text` as an integer, decimal (optionally negative) or 0x hex; nullopt otherwise. */
    inline std::optional<long> toLong(const std::string& text)
    {
        const bool hex = text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X');
        const char* begin = text.data() + (hex ? 2 : 0);
        const char* end = text.data() + text.size();
        long v = 0;
        const auto [ptr, ec] = std::from_chars(begin, end, v, hex ? 16 : 10);
        if (ec != std::errc() || ptr != end || begin == end) return std::nullopt;
        return v;
    }

    /** A finite number; keeps `out` when the parameter is absent. */
    inline bool number(const Params& p, const std::string& key, double& out, std::string& error)
    {
        const auto it = p.find(key);
        if (it == p.end()) return true;
        const auto v = toDouble(it->second);
        if (!v)
        {
            error = key + " must be a number, got '" + it->second + "'";
            return false;
        }
        out = *v;
        return true;
    }

    /** A ratio in [0, 1]; keeps `out` when the parameter is absent. */
    inline bool ratio(const Params& p, const std::string& key, double& out, std::string& error)
    {
        const auto it = p.find(key);
        if (it == p.end()) return true;
        const auto v = toDouble(it->second);
        if (!v || *v < 0.0 || *v > 1.0)
        {
            error = key + " must be a number in [0, 1], got '" + it->second + "'";
            return false;
        }
        out = *v;
        return true;
    }

    /** An integer in [lo, hi] (decimal or 0x hex); keeps `out` when absent. */
    inline bool integer(const Params& p, const std::string& key, long lo, long hi, long& out, std::string& error)
    {
        const auto it = p.find(key);
        if (it == p.end()) return true;
        const auto v = toLong(it->second);
        if (!v || *v < lo || *v > hi)
        {
            error = key + " must be an integer in [" + std::to_string(lo) + ", " + std::to_string(hi) + "], got '" +
                    it->second + "'";
            return false;
        }
        out = *v;
        return true;
    }
}  // namespace end_effector_hardware::params
