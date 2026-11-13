#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <unordered_map>

#include "robot_comm/interface_name.h"

namespace robot_comm
{
    // ---- configuration ------------------------------------------------------------------------------------------

    /**
     * One CAN bus as a device driver sees it. Every CAN driver reads the same parameter names (see parseCanConfig),
     * so a new CAN device needs no new parameters: <prefix>can_interface, <prefix>can_bitrate,
     * <prefix>can_data_bitrate, <prefix>can_restart_ms, <prefix>can_manage_interface.
     */
    struct CanConfig
    {
        std::string interface = "can0";
        uint32_t bitrate = 1'000'000;    ///< arbitration phase, bit/s
        uint32_t data_bitrate = 0;       ///< CAN FD data phase, bit/s; 0 = classic CAN
        int restart_ms = 100;            ///< automatic bus-off recovery; 0 = off
        bool manage_interface = true;    ///< false: the interface is already up (systemd-networkd, ...), do not touch it

        bool fd() const { return data_bitrate > 0; }

        std::string describe() const
        {
            std::string text = interface + " " + std::to_string(bitrate);
            if (fd())
            {
                text += "/" + std::to_string(data_bitrate) + " FD";
            }
            text += manage_interface ? ", configured by the driver" : ", configured externally";
            return text;
        }
    };

    namespace detail
    {
        inline bool parseUnsigned(const std::string& text, uint32_t& out)
        {
            if (text.empty() || text.front() == '-')
            {
                return false;
            }
            char* end = nullptr;
            const unsigned long long value = std::strtoull(text.c_str(), &end, 10);
            if (end == text.c_str() || *end != '\0' || value > 0xFFFFFFFFULL)
            {
                return false;
            }
            out = static_cast<uint32_t>(value);
            return true;
        }

        inline bool parseBool(const std::string& text, bool& out)
        {
            if (text == "true" || text == "1" || text == "True" || text == "TRUE")
            {
                out = true;
                return true;
            }
            if (text == "false" || text == "0" || text == "False" || text == "FALSE")
            {
                out = false;
                return true;
            }
            return false;
        }
    } // namespace detail

    /**
     * Read a CanConfig from hardware parameters (hardware_interface::HardwareInfo::hardware_parameters).
     * Missing keys keep the value in `defaults`; a bad value fails with a message in `error`.
     * `prefix` is "" for a driver with one bus, or e.g. "left_ee_" so the keys read left_ee_can_interface, ...
     */
    inline bool parseCanConfig(const std::unordered_map<std::string, std::string>& params, const std::string& prefix,
                               const CanConfig& defaults, CanConfig& out, std::string& error)
    {
        CanConfig config = defaults;
        const auto find = [&](const char* key) -> const std::string*
        {
            const auto it = params.find(prefix + key);
            return it == params.end() ? nullptr : &it->second;
        };

        if (const auto* value = find("can_interface"))
        {
            if (!isValidInterfaceName(*value))
            {
                error = "invalid " + prefix + "can_interface '" + *value +
                    "': expected 1 to 15 characters of [A-Za-z0-9_.-], not starting with '-'";
                return false;
            }
            config.interface = *value;
        }
        if (!isValidInterfaceName(config.interface))
        {
            error = "invalid default CAN interface '" + config.interface + "'";
            return false;
        }

        const auto read_unsigned = [&](const char* key, uint32_t& target) -> bool
        {
            if (const auto* value = find(key))
            {
                if (!detail::parseUnsigned(*value, target))
                {
                    error = "invalid " + prefix + key + " '" + *value + "': expected a non-negative integer";
                    return false;
                }
            }
            return true;
        };
        uint32_t restart = static_cast<uint32_t>(config.restart_ms);
        if (!read_unsigned("can_bitrate", config.bitrate) || !read_unsigned("can_data_bitrate", config.data_bitrate) ||
            !read_unsigned("can_restart_ms", restart))
        {
            return false;
        }
        if (config.bitrate == 0)
        {
            error = "invalid " + prefix + "can_bitrate '0'";
            return false;
        }
        if (restart > 60'000)
        {
            error = "invalid " + prefix + "can_restart_ms: at most 60000";
            return false;
        }
        config.restart_ms = static_cast<int>(restart);

        if (const auto* value = find("can_manage_interface"))
        {
            if (!detail::parseBool(*value, config.manage_interface))
            {
                error = "invalid " + prefix + "can_manage_interface '" + *value + "': expected true or false";
                return false;
            }
        }
        out = config;
        return true;
    }

    // ---- frames and the bus -------------------------------------------------------------------------------------

    constexpr std::size_t kCanFdMaxLen = 64;

    /// A CAN or CAN FD frame, independent of SocketCAN: the flag bits of the kernel's can_id are separate fields.
    struct CanFrame
    {
        uint32_t id = 0;        ///< 11-bit or 29-bit identifier, without flag bits
        bool extended = false;  ///< 29-bit identifier
        bool rtr = false;       ///< remote transmission request
        bool error = false;     ///< error frame reported by the controller, not sent by a node
        bool fd = false;        ///< CAN FD frame
        bool brs = false;       ///< bit-rate switch of the data phase (FD only)
        uint8_t len = 0;
        std::array<uint8_t, kCanFdMaxLen> data{};

        /// A plain data frame: not extended, not RTR, not an error frame.
        bool isStandardData() const { return !extended && !rtr && !error; }
    };

    enum class CanReceive
    {
        Frame, ///< `frame` holds a received frame
        Empty, ///< nothing to read right now (non-blocking)
        Error  ///< the read failed, see lastError()
    };

    /// Counters a bus keeps for stability checks. Read from any thread; the values only grow.
    struct CanStats
    {
        uint64_t tx_frames = 0;
        uint64_t tx_errors = 0; ///< send() failures (queue full, interface down, ...)
        uint64_t rx_frames = 0;
        uint64_t rx_errors = 0; ///< receive() failures other than "nothing to read"
    };

    /**
     * A CAN bus. Implementations: SocketCanBus (Linux SocketCAN, in robot_comm_can) and MockCanBus (tests).
     * send() and receive() never block, so they are safe in a real-time cycle; waitReadable() is for start-up paths
     * that have to wait for answers.
     */
    class CanBus
    {
    public:
        virtual ~CanBus() = default;

        /// Open the bus on `config.interface`. The interface must already be up (see CanInterfaceConfigurator).
        virtual bool open(const CanConfig& config) = 0;
        virtual void close() = 0;
        virtual bool isOpen() const = 0;

        virtual bool send(const CanFrame& frame) = 0;
        virtual CanReceive receive(CanFrame& frame) = 0;
        virtual bool waitReadable(std::chrono::milliseconds timeout) = 0;

        /// Text of the last failure (open, send or receive).
        virtual std::string lastError() const = 0;

        virtual CanStats stats() const = 0;
    };

    /// Brings the interface up with the bitrates in a CanConfig. Separate from CanBus so a driver can run on an
    /// interface somebody else configured (config.manage_interface = false).
    class CanInterfaceConfigurator
    {
    public:
        virtual ~CanInterfaceConfigurator() = default;
        virtual bool apply(const CanConfig& config) = 0;
        virtual std::string lastError() const = 0;
    };
} // namespace robot_comm
