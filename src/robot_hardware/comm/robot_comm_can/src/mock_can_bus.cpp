#include "robot_comm_can/mock_can_bus.h"

namespace robot_comm
{
    bool MockCanBus::open(const CanConfig& config)
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

    void MockCanBus::close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = false;
    }

    bool MockCanBus::isOpen() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_;
    }

    bool MockCanBus::send(const CanFrame& frame)
    {
        std::function<void(const CanFrame&)> hook;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!open_ || send_fails_)
            {
                error_ = open_ ? "mock: send failed" : "CAN bus is not open";
                ++stats_.tx_errors;
                return false;
            }
            tx_.push_back(frame);
            ++stats_.tx_frames;
            hook = send_hook_;
        }
        if (hook)
        {
            hook(frame);
        }
        return true;
    }

    CanReceive MockCanBus::receive(CanFrame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!open_ || receive_fails_)
        {
            error_ = open_ ? "mock: receive failed" : "CAN bus is not open";
            ++stats_.rx_errors;
            return CanReceive::Error;
        }
        if (rx_.empty())
        {
            return CanReceive::Empty;
        }
        frame = rx_.front();
        rx_.pop_front();
        ++stats_.rx_frames;
        return CanReceive::Frame;
    }

    bool MockCanBus::waitReadable(std::chrono::milliseconds)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_ && !rx_.empty();
    }

    std::string MockCanBus::lastError() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_;
    }

    CanStats MockCanBus::stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    void MockCanBus::pushReceived(const CanFrame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::vector<CanFrame> MockCanBus::sentFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_;
    }

    void MockCanBus::clearSent()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.clear();
    }

    void MockCanBus::setOpenFails(bool fails)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_fails_ = fails;
    }

    void MockCanBus::setSendFails(bool fails)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        send_fails_ = fails;
    }

    void MockCanBus::setReceiveFails(bool fails)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receive_fails_ = fails;
    }

    void MockCanBus::setSendHook(std::function<void(const CanFrame&)> hook)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        send_hook_ = std::move(hook);
    }

    CanConfig MockCanBus::openedWith() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }
} // namespace robot_comm
