#pragma once

#include <atomic>
#include <string>

#include "robot_comm/can.h"

namespace robot_comm
{
    /**
     * Linux SocketCAN (CAN_RAW). Non-blocking: send() and receive() return at once, so they are safe in a
     * real-time cycle. With CanConfig::fd() the socket carries CAN FD frames; frames with CanFrame::fd are written
     * as canfd_frame, the others as can_frame.
     *
     * One thread may call send() while another calls receive() (a socket allows that); open() / close() must not run
     * at the same time as either.
     */
    class SocketCanBus final : public CanBus
    {
    public:
        SocketCanBus() = default;
        ~SocketCanBus() override;
        SocketCanBus(const SocketCanBus&) = delete;
        SocketCanBus& operator=(const SocketCanBus&) = delete;

        bool open(const CanConfig& config) override;
        void close() override;
        bool isOpen() const override { return fd_ >= 0; }

        bool send(const CanFrame& frame) override;
        CanReceive receive(CanFrame& frame) override;
        bool waitReadable(std::chrono::milliseconds timeout) override;

        std::string lastError() const override { return error_; }
        CanStats stats() const override;

    private:
        int fd_{-1};
        std::string error_;
        std::atomic<uint64_t> tx_frames_{0};
        std::atomic<uint64_t> tx_errors_{0};
        std::atomic<uint64_t> rx_frames_{0};
        std::atomic<uint64_t> rx_errors_{0};
    };
} // namespace robot_comm
