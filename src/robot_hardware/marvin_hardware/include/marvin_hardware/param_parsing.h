#pragma once

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <string>

namespace marvin_hardware::param_parsing
{
    // Whole-string conversions for the values the ros2_control xacro hands to the plugin as text.
    // std::stoi("12abc") quietly returns 12 and std::stod("1e999") throws, so the plugin used to either accept
    // garbage or fall back to a default without saying so. These return false for anything that is not exactly
    // one number (surrounding whitespace is allowed), and the caller reports it.

    inline std::string trimmed(const std::string& text)
    {
        const auto first = text.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            return "";
        }
        const auto last = text.find_last_not_of(" \t\r\n");
        return text.substr(first, last - first + 1);
    }

    inline bool parseInt(const std::string& text, int& out)
    {
        const std::string s = trimmed(text);
        if (s.empty())
        {
            return false;
        }
        errno = 0;
        char* end = nullptr;
        const long value = std::strtol(s.c_str(), &end, 10);
        if (end == s.c_str() || *end != '\0' || errno == ERANGE || value < INT_MIN || value > INT_MAX)
        {
            return false;
        }
        out = static_cast<int>(value);
        return true;
    }

    inline bool parseDouble(const std::string& text, double& out)
    {
        const std::string s = trimmed(text);
        if (s.empty())
        {
            return false;
        }
        errno = 0;
        char* end = nullptr;
        const double value = std::strtod(s.c_str(), &end);
        if (end == s.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(value))
        {
            return false;
        }
        out = value;
        return true;
    }
} // namespace marvin_hardware::param_parsing
