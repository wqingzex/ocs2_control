#include "robot_comm_can/socket_can_bus.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_comm
{
    namespace
    {
        std::string errnoText(const std::string& what)
        {
            return what + ": " + std::strerror(errno);
        }

        canid_t encodeId(const CanFrame& frame)
        {
            canid_t id = frame.extended ? (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG : (frame.id & CAN_SFF_MASK);
            if (frame.rtr)
            {
                id |= CAN_RTR_FLAG;
            }
            return id;
        }
    } // namespace

    SocketCanBus::~SocketCanBus()
    {
        close();
    }

    bool SocketCanBus::open(const CanConfig& config)
    {
        close();
        if (!isValidInterfaceName(config.interface))
        {
            error_ = "invalid CAN interface name '" + config.interface + "'";
            return false;
        }

        fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0)
        {
            error_ = errnoText("CAN socket create failed");
            return false;
        }

        struct ifreq ifr = {};
        std::strncpy(ifr.ifr_name, config.interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            error_ = errnoText("CAN interface index failed");
            close();
            return false;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            error_ = errnoText("CAN bind failed");
            close();
            return false;
        }

        if (config.fd())
        {
            int enable_fd = 1;
            if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
            {
                error_ = errnoText("CAN FD enable failed");
                close();
                return false;
            }
        }

        // Non-blocking, so a read with nothing to read never stalls a real-time cycle.
        const int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            error_ = errnoText("CAN set non-blocking failed");
            close();
            return false;
        }
        return true;
    }

    void SocketCanBus::close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool SocketCanBus::send(const CanFrame& frame)
    {
        if (fd_ < 0)
        {
            error_ = "CAN bus is not open";
            ++tx_errors_;
            return false;
        }
        ssize_t written = -1;
        bool complete = false;
        if (frame.fd)
        {
            if (frame.len > kCanFdMaxLen)
            {
                error_ = "CAN FD frame longer than 64 bytes";
                ++tx_errors_;
                return false;
            }
            struct canfd_frame raw = {};
            raw.can_id = encodeId(frame);
            raw.len = frame.len;
            raw.flags = frame.brs ? CANFD_BRS : 0;
            std::memcpy(raw.data, frame.data.data(), frame.len);
            written = ::write(fd_, &raw, sizeof(raw));
            complete = written == static_cast<ssize_t>(sizeof(raw));
        }
        else
        {
            if (frame.len > CAN_MAX_DLEN)
            {
                error_ = "classic CAN frame longer than 8 bytes";
                ++tx_errors_;
                return false;
            }
            struct can_frame raw = {};
            raw.can_id = encodeId(frame);
            raw.can_dlc = frame.len;
            std::memcpy(raw.data, frame.data.data(), frame.len);
            written = ::write(fd_, &raw, sizeof(raw));
            complete = written == static_cast<ssize_t>(sizeof(raw));
        }
        if (!complete)
        {
            error_ = errnoText("CAN send failed");
            ++tx_errors_;
            return false;
        }
        ++tx_frames_;
        return true;
    }

    CanReceive SocketCanBus::receive(CanFrame& frame)
    {
        if (fd_ < 0)
        {
            error_ = "CAN bus is not open";
            return CanReceive::Error;
        }
        struct canfd_frame raw = {};
        const ssize_t n = ::read(fd_, &raw, sizeof(raw));
        if (n <= 0)
        {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                error_ = errnoText("CAN read error");
                ++rx_errors_;
                return CanReceive::Error;
            }
            return CanReceive::Empty;
        }

        frame = CanFrame{};
        frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
        frame.rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
        frame.error = (raw.can_id & CAN_ERR_FLAG) != 0;
        frame.id = frame.error ? (raw.can_id & CAN_ERR_MASK)
                               : (frame.extended ? (raw.can_id & CAN_EFF_MASK) : (raw.can_id & CAN_SFF_MASK));
        frame.fd = n == static_cast<ssize_t>(CANFD_MTU);
        frame.brs = frame.fd && (raw.flags & CANFD_BRS) != 0;
        frame.len = static_cast<uint8_t>(std::min<std::size_t>(raw.len, frame.fd ? kCanFdMaxLen : CAN_MAX_DLEN));
        std::memcpy(frame.data.data(), raw.data, frame.len);
        ++rx_frames_;
        return CanReceive::Frame;
    }

    bool SocketCanBus::waitReadable(std::chrono::milliseconds timeout)
    {
        if (fd_ < 0)
        {
            return false;
        }
        struct pollfd pfd = {};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        return ::poll(&pfd, 1, static_cast<int>(timeout.count())) > 0;
    }

    CanStats SocketCanBus::stats() const
    {
        return CanStats{tx_frames_.load(), tx_errors_.load(), rx_frames_.load(), rx_errors_.load()};
    }
} // namespace robot_comm
