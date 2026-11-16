#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_comm/bus.h"

/**
 * Modbus RTU frames as plain functions, for devices that talk over a robot_comm::Bus (USB-RS485 or an arm's flange
 * port). Frames are slave id, function code, data, CRC-16 (low byte first). Supported: FC03 / FC04 read registers,
 * FC06 write one register, FC16 write registers.
 */
namespace robot_comm::modbus_rtu
{
    constexpr uint8_t kReadHoldingRegisters = 0x03;
    constexpr uint8_t kReadInputRegisters = 0x04;
    constexpr uint8_t kWriteSingleRegister = 0x06;
    constexpr uint8_t kWriteMultipleRegisters = 0x10;

    inline uint16_t crc16(const uint8_t* data, std::size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (std::size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
            }
        }
        return crc;
    }

    namespace detail
    {
        inline void put16(std::vector<uint8_t>& out, uint16_t value)
        {
            out.push_back(static_cast<uint8_t>(value >> 8));
            out.push_back(static_cast<uint8_t>(value & 0xFF));
        }
        inline uint16_t get16(const std::vector<uint8_t>& in, std::size_t at)
        {
            return static_cast<uint16_t>((in[at] << 8) | in[at + 1]);
        }
        inline std::vector<uint8_t> withCrc(std::vector<uint8_t> frame)
        {
            const uint16_t crc = crc16(frame.data(), frame.size());
            frame.push_back(static_cast<uint8_t>(crc & 0xFF));
            frame.push_back(static_cast<uint8_t>(crc >> 8));
            return frame;
        }
    }  // namespace detail

    /** FC03 / FC04. */
    inline std::vector<uint8_t> readRegisters(uint8_t slave, uint8_t function, uint16_t address, uint16_t count)
    {
        std::vector<uint8_t> frame{slave, function};
        detail::put16(frame, address);
        detail::put16(frame, count);
        return detail::withCrc(std::move(frame));
    }

    /** FC06. */
    inline std::vector<uint8_t> writeRegister(uint8_t slave, uint16_t address, uint16_t value)
    {
        std::vector<uint8_t> frame{slave, kWriteSingleRegister};
        detail::put16(frame, address);
        detail::put16(frame, value);
        return detail::withCrc(std::move(frame));
    }

    /** FC16. */
    inline std::vector<uint8_t> writeRegisters(uint8_t slave, uint16_t address, const std::vector<uint16_t>& values)
    {
        std::vector<uint8_t> frame{slave, kWriteMultipleRegisters};
        detail::put16(frame, address);
        detail::put16(frame, static_cast<uint16_t>(values.size()));
        frame.push_back(static_cast<uint8_t>(values.size() * 2));
        for (const uint16_t v : values) detail::put16(frame, v);
        return detail::withCrc(std::move(frame));
    }

    /**
     * The length of a reply from its first bytes (0 while unknown): an exception reply is 5 bytes, a read reply
     * 5 + byte count, a write reply 8. An unknown function code ends the reply where it is (it will fail to parse).
     */
    inline std::size_t replyLength(const std::vector<uint8_t>& bytes)
    {
        if (bytes.size() < 2) return 0;
        const uint8_t function = bytes[1];
        if (function & 0x80) return 5;
        if (function == kReadHoldingRegisters || function == kReadInputRegisters)
        {
            return bytes.size() < 3 ? 0 : 5u + bytes[2];
        }
        if (function == kWriteSingleRegister || function == kWriteMultipleRegisters) return 8;
        return bytes.size();
    }

    /** What to wait for after sending a Modbus RTU request. */
    inline ReplySpec replySpec(std::chrono::milliseconds timeout)
    {
        ReplySpec spec;
        spec.frame_length = replyLength;
        spec.max_size = 256;
        spec.timeout = timeout;
        return spec;
    }

    struct Result
    {
        bool ok = false;
        uint8_t exception = 0;  ///< Modbus exception code when the device refused the request
        std::string error;
        std::vector<uint16_t> registers;  ///< read replies
    };

    namespace detail
    {
        inline bool checkFrame(const std::vector<uint8_t>& reply, uint8_t slave, uint8_t function, Result& result)
        {
            if (reply.size() < 5)
            {
                result.error = "reply too short (" + std::to_string(reply.size()) + " bytes)";
                return false;
            }
            const uint16_t crc = crc16(reply.data(), reply.size() - 2);
            if (reply[reply.size() - 2] != (crc & 0xFF) || reply[reply.size() - 1] != (crc >> 8))
            {
                result.error = "CRC mismatch";
                return false;
            }
            if (reply[0] != slave)
            {
                result.error = "reply from slave " + std::to_string(reply[0]) + ", expected " + std::to_string(slave);
                return false;
            }
            if (reply[1] == (function | 0x80))
            {
                result.exception = reply[2];
                result.error = "Modbus exception " + std::to_string(reply[2]);
                return false;
            }
            // FC03 and FC04 are accepted for each other: some devices answer a read with the other code (as the
            // Marvin tool code has always accepted).
            const bool read = function == kReadHoldingRegisters || function == kReadInputRegisters;
            const bool same = reply[1] == function ||
                              (read && (reply[1] == kReadHoldingRegisters || reply[1] == kReadInputRegisters));
            if (!same)
            {
                result.error = "reply function code " + std::to_string(reply[1]) + ", expected " + std::to_string(function);
                return false;
            }
            return true;
        }
    }  // namespace detail

    inline Result parseReadReply(const std::vector<uint8_t>& reply, uint8_t slave, uint8_t function, uint16_t count)
    {
        Result result;
        if (!detail::checkFrame(reply, slave, function, result)) return result;
        const std::size_t bytes = reply[2];
        if (bytes != count * 2u || reply.size() != 5 + bytes)
        {
            result.error = "read reply has " + std::to_string(bytes) + " data bytes, expected " + std::to_string(count * 2);
            return result;
        }
        result.registers.reserve(count);
        for (std::size_t i = 0; i < count; ++i) result.registers.push_back(detail::get16(reply, 3 + 2 * i));
        result.ok = true;
        return result;
    }

    /** FC06 echoes address and value; FC16 echoes address and count. */
    inline Result parseWriteReply(const std::vector<uint8_t>& reply, uint8_t slave, uint8_t function, uint16_t address,
                                  uint16_t value_or_count)
    {
        Result result;
        if (!detail::checkFrame(reply, slave, function, result)) return result;
        if (reply.size() != 8 || detail::get16(reply, 2) != address || detail::get16(reply, 4) != value_or_count)
        {
            result.error = "write reply does not echo the request";
            return result;
        }
        result.ok = true;
        return result;
    }
}  // namespace robot_comm::modbus_rtu
