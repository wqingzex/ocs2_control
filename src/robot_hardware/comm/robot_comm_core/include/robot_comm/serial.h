#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_comm
{
    struct SerialConfig
    {
        std::string device;           ///< /dev/ttyUSB0 or a symlink to it
        int baudrate = 115200;
        bool low_latency = true;      ///< shorten the latency timer of USB serial adapters (FTDI) to 1 ms
        bool strict_baudrate = true;  ///< false: an unsupported baudrate falls back to 115200 (what the hand drivers did)
    };

    enum class IoStatus
    {
        Ok,         ///< `n` bytes were read / written, or the descriptor is ready
        WouldBlock, ///< nothing to do right now (EAGAIN, EINTR, a wait that timed out)
        Error       ///< see lastError()
    };

    /// Something open() wants the driver to log (the USB latency timer was changed, could not be changed, ...).
    struct Notice
    {
        enum class Level { Info, Warn };
        Level level = Level::Info;
        std::string text;
    };

    /**
     * A serial port in raw 8N1 mode, no flow control, non-blocking. Implementations: PosixSerialPort (termios, in
     * robot_comm_serial) and MockSerialPort (tests). read() and write() never block; wait*() block for at most the
     * given time.
     */
    class SerialPort
    {
    public:
        virtual ~SerialPort() = default;

        virtual bool open(const SerialConfig& config) = 0;
        virtual void close() = 0;
        virtual bool isOpen() const = 0;

        /// Ok: `n` > 0 bytes read. WouldBlock: nothing now. Error: lastError().
        virtual IoStatus read(uint8_t* buffer, std::size_t capacity, std::size_t& n) = 0;
        /// Ok: `n` > 0 bytes accepted (possibly fewer than `length`). WouldBlock: the output buffer is full.
        virtual IoStatus write(const uint8_t* data, std::size_t length, std::size_t& n) = 0;

        /// Ok: readable / writable. WouldBlock: not within `timeout`. Error: lastError().
        virtual IoStatus waitReadable(std::chrono::microseconds timeout) = 0;
        virtual IoStatus waitWritable(std::chrono::microseconds timeout) = 0;

        virtual void flushInput() = 0;
        virtual void flushAll() = 0;
        /// Wait until everything written has left the port (tcdrain).
        virtual void drain() = 0;

        virtual std::string lastError() const = 0;
        /// Messages open() collected; returns them and clears the list.
        virtual std::vector<Notice> takeNotices() = 0;
    };

    /**
     * Write all of `data`, waiting for room in 1 ms steps, for at most `timeout`. The text of a failure is
     * "serial write failed: <reason>" or "serial write timeout after <written>/<total> bytes".
     */
    inline bool writeAll(SerialPort& port, const uint8_t* data, std::size_t length, std::chrono::milliseconds timeout,
                         std::string& error)
    {
        std::size_t total = 0;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (total < length && std::chrono::steady_clock::now() <= deadline)
        {
            std::size_t n = 0;
            const IoStatus status = port.write(data + total, length - total, n);
            if (status == IoStatus::Ok)
            {
                total += n;
                continue;
            }
            if (status == IoStatus::WouldBlock)
            {
                port.waitWritable(std::chrono::microseconds(1000));
                continue;
            }
            error = "serial write failed: " + port.lastError();
            return false;
        }
        if (total != length)
        {
            error = "serial write timeout after " + std::to_string(total) + "/" + std::to_string(length) + " bytes";
            return false;
        }
        return true;
    }
} // namespace robot_comm
