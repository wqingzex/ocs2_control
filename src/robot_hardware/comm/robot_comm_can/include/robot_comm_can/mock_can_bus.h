#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "robot_comm/can.h"

namespace robot_comm
{
    /// An in-memory bus for tests: queue frames for receive(), read back what was sent, force failures.
    class MockCanBus final : public CanBus
    {
    public:
        bool open(const CanConfig& config) override;
        void close() override;
        bool isOpen() const override;

        bool send(const CanFrame& frame) override;
        CanReceive receive(CanFrame& frame) override;
        bool waitReadable(std::chrono::milliseconds timeout) override;

        std::string lastError() const override;
        CanStats stats() const override;

        // ---- test controls ----
        void pushReceived(const CanFrame& frame);
        std::vector<CanFrame> sentFrames() const;
        void clearSent();
        void setOpenFails(bool fails);
        void setSendFails(bool fails);
        void setReceiveFails(bool fails);
        CanConfig openedWith() const;

        /// Called after every successful send(), on the sending thread and without the bus lock, so a test can answer
        /// with pushReceived() at once (a closed loop with a device model).
        void setSendHook(std::function<void(const CanFrame&)> hook);

    private:
        mutable std::mutex mutex_;
        std::deque<CanFrame> rx_;
        std::vector<CanFrame> tx_;
        CanConfig config_;
        std::string error_;
        CanStats stats_;
        bool open_{false};
        bool open_fails_{false};
        bool send_fails_{false};
        bool receive_fails_{false};
        std::function<void(const CanFrame&)> send_hook_;
    };
} // namespace robot_comm
