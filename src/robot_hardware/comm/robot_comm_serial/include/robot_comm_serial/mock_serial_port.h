#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "robot_comm/serial.h"

namespace robot_comm
{
    /// An in-memory serial port for tests: queue bytes for read(), read back what was written, force failures.
    class MockSerialPort final : public SerialPort
    {
    public:
        bool open(const SerialConfig& config) override;
        void close() override;
        bool isOpen() const override;

        IoStatus read(uint8_t* buffer, std::size_t capacity, std::size_t& n) override;
        IoStatus write(const uint8_t* data, std::size_t length, std::size_t& n) override;
        IoStatus waitReadable(std::chrono::microseconds timeout) override;
        IoStatus waitWritable(std::chrono::microseconds timeout) override;

        void flushInput() override;
        void flushAll() override;
        void drain() override;

        std::string lastError() const override;
        std::vector<Notice> takeNotices() override;

        // ---- test controls ----
        void pushReceived(const std::vector<uint8_t>& bytes);
        std::vector<uint8_t> written() const;
        void clearWritten();
        SerialConfig openedWith() const;
        void setOpenFails(bool fails);
        void setReadFails(bool fails);
        void setWriteFails(bool fails);
        /// At most this many bytes are accepted per write() (0 = all): partial writes.
        void setWriteChunk(std::size_t bytes);
        /// At most this many bytes are returned per read() (0 = all).
        void setReadChunk(std::size_t bytes);
        /// Called after each write(), without the port lock: answer with pushReceived() to model a device.
        void setWriteHook(std::function<void(const std::vector<uint8_t>&)> hook);
        int flushInputCalls() const;
        int flushAllCalls() const;
        int drainCalls() const;

    private:
        mutable std::mutex mutex_;
        std::deque<uint8_t> rx_;
        std::vector<uint8_t> tx_;
        SerialConfig config_;
        std::string error_;
        std::function<void(const std::vector<uint8_t>&)> write_hook_;
        std::size_t write_chunk_{0};
        std::size_t read_chunk_{0};
        int flush_input_calls_{0};
        int flush_all_calls_{0};
        int drain_calls_{0};
        bool open_{false};
        bool open_fails_{false};
        bool read_fails_{false};
        bool write_fails_{false};
    };
} // namespace robot_comm
