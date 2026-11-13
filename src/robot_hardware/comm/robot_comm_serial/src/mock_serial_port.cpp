#include "robot_comm_serial/mock_serial_port.h"

#include <algorithm>

namespace robot_comm
{
    bool MockSerialPort::open(const SerialConfig& config)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (open_fails_)
        {
            error_ = "mock: open failed";
            return false;
        }
        config_ = config;
        open_ = true;
        return true;
    }

    void MockSerialPort::close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = false;
    }

    bool MockSerialPort::isOpen() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_;
    }

    IoStatus MockSerialPort::read(uint8_t* buffer, std::size_t capacity, std::size_t& n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        n = 0;
        if (!open_ || read_fails_)
        {
            error_ = open_ ? "mock: read failed" : "serial port is not open";
            return IoStatus::Error;
        }
        if (rx_.empty())
        {
            return IoStatus::WouldBlock;
        }
        std::size_t take = std::min(capacity, rx_.size());
        if (read_chunk_ > 0)
        {
            take = std::min(take, read_chunk_);
        }
        for (std::size_t i = 0; i < take; ++i)
        {
            buffer[i] = rx_.front();
            rx_.pop_front();
        }
        n = take;
        return IoStatus::Ok;
    }

    IoStatus MockSerialPort::write(const uint8_t* data, std::size_t length, std::size_t& n)
    {
        std::function<void(const std::vector<uint8_t>&)> hook;
        std::vector<uint8_t> accepted;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            n = 0;
            if (!open_ || write_fails_)
            {
                error_ = open_ ? "mock: write failed" : "serial port is not open";
                return IoStatus::Error;
            }
            const std::size_t take = write_chunk_ > 0 ? std::min(length, write_chunk_) : length;
            accepted.assign(data, data + take);
            tx_.insert(tx_.end(), accepted.begin(), accepted.end());
            n = take;
            hook = write_hook_;
        }
        if (hook)
        {
            hook(accepted);
        }
        return n > 0 ? IoStatus::Ok : IoStatus::WouldBlock;
    }

    IoStatus MockSerialPort::waitReadable(std::chrono::microseconds)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!open_)
        {
            error_ = "serial port is not open";
            return IoStatus::Error;
        }
        return rx_.empty() ? IoStatus::WouldBlock : IoStatus::Ok;
    }

    IoStatus MockSerialPort::waitWritable(std::chrono::microseconds)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_ ? IoStatus::Ok : IoStatus::Error;
    }

    void MockSerialPort::flushInput()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
        ++flush_input_calls_;
    }

    void MockSerialPort::flushAll()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
        ++flush_all_calls_;
    }

    void MockSerialPort::drain()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++drain_calls_;
    }

    std::string MockSerialPort::lastError() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_;
    }

    std::vector<Notice> MockSerialPort::takeNotices() { return {}; }

    void MockSerialPort::pushReceived(const std::vector<uint8_t>& bytes)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.insert(rx_.end(), bytes.begin(), bytes.end());
    }

    std::vector<uint8_t> MockSerialPort::written() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_;
    }

    void MockSerialPort::clearWritten()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.clear();
    }

    SerialConfig MockSerialPort::openedWith() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }

    void MockSerialPort::setOpenFails(bool fails) { std::lock_guard<std::mutex> lock(mutex_); open_fails_ = fails; }
    void MockSerialPort::setReadFails(bool fails) { std::lock_guard<std::mutex> lock(mutex_); read_fails_ = fails; }
    void MockSerialPort::setWriteFails(bool fails) { std::lock_guard<std::mutex> lock(mutex_); write_fails_ = fails; }
    void MockSerialPort::setWriteChunk(std::size_t bytes) { std::lock_guard<std::mutex> lock(mutex_); write_chunk_ = bytes; }
    void MockSerialPort::setReadChunk(std::size_t bytes) { std::lock_guard<std::mutex> lock(mutex_); read_chunk_ = bytes; }

    void MockSerialPort::setWriteHook(std::function<void(const std::vector<uint8_t>&)> hook)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        write_hook_ = std::move(hook);
    }

    int MockSerialPort::flushInputCalls() const { std::lock_guard<std::mutex> lock(mutex_); return flush_input_calls_; }
    int MockSerialPort::flushAllCalls() const { std::lock_guard<std::mutex> lock(mutex_); return flush_all_calls_; }
    int MockSerialPort::drainCalls() const { std::lock_guard<std::mutex> lock(mutex_); return drain_calls_; }
} // namespace robot_comm
