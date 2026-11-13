#pragma once

#include <string>
#include <vector>

#include "robot_comm/serial.h"

namespace robot_comm
{
    /// termios serial port. open() does what the hand and sensor drivers did, in the same order: open the device
    /// (O_RDWR | O_NOCTTY | O_NONBLOCK), raw 8N1 without flow control with VMIN = VTIME = 0, the baudrate, shorten
    /// the USB latency timer (if config.low_latency), flush both directions.
    class PosixSerialPort final : public SerialPort
    {
    public:
        PosixSerialPort() = default;
        ~PosixSerialPort() override;
        PosixSerialPort(const PosixSerialPort&) = delete;
        PosixSerialPort& operator=(const PosixSerialPort&) = delete;

        bool open(const SerialConfig& config) override;
        void close() override;
        bool isOpen() const override { return fd_ >= 0; }

        IoStatus read(uint8_t* buffer, std::size_t capacity, std::size_t& n) override;
        IoStatus write(const uint8_t* data, std::size_t length, std::size_t& n) override;
        IoStatus waitReadable(std::chrono::microseconds timeout) override;
        IoStatus waitWritable(std::chrono::microseconds timeout) override;

        void flushInput() override;
        void flushAll() override;
        void drain() override;

        std::string lastError() const override { return error_; }
        std::vector<Notice> takeNotices() override;

    private:
        IoStatus wait(bool for_read, std::chrono::microseconds timeout);

        int fd_{-1};
        std::string error_;
        std::vector<Notice> notices_;
    };
} // namespace robot_comm
