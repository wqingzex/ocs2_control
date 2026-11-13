#pragma once

#include <cstddef>
#include <string>

namespace robot_comm
{
    /// Linux network interface names are at most 15 characters (IFNAMSIZ - 1).
    constexpr std::size_t kMaxInterfaceNameLength = 15;

    /**
     * A CAN interface name is put into `sudo ip link set <name> ...` and run through the shell, and it has to match
     * the sudoers rule that scripts/setup.sh writes. Accept only plain names: 1 to 15 characters of [A-Za-z0-9_.-],
     * not starting with '-' (which `ip` would read as an option).
     */
    inline bool isValidInterfaceName(const std::string& name)
    {
        if (name.empty() || name.size() > kMaxInterfaceNameLength || name.front() == '-')
        {
            return false;
        }
        for (const char c : name)
        {
            const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
                c == '.' || c == '-';
            if (!ok)
            {
                return false;
            }
        }
        return true;
    }
} // namespace robot_comm
