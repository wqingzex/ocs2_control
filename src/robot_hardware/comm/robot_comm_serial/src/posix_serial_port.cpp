#include "robot_comm_serial/posix_serial_port.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "robot_comm_serial/usb_latency.h"

namespace robot_comm
{
    namespace
    {
        // 0 = not supported
        speed_t baudConstant(int baudrate)
        {
            switch (baudrate)
            {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
#ifdef B921600
            case 921600: return B921600;
#endif
#ifdef B1000000
            case 1000000: return B1000000;
#endif
#ifdef B1500000
            case 1500000: return B1500000;
#endif
#ifdef B2000000
            case 2000000: return B2000000;
#endif
#ifdef B2500000
            case 2500000: return B2500000;
#endif
#ifdef B3000000
            case 3000000: return B3000000;
#endif
            default: return 0;
            }
        }

        std::string errnoText(const std::string& what)
        {
            return what + ": " + std::strerror(errno);
        }
    } // namespace

    PosixSerialPort::~PosixSerialPort()
    {
        close();
    }

    bool PosixSerialPort::open(const SerialConfig& config)
    {
        close();
        notices_.clear();

        speed_t baud = baudConstant(config.baudrate);
        if (baud == 0)
        {
            if (config.strict_baudrate)
            {
                error_ = "unsupported baudrate " + std::to_string(config.baudrate);
                return false;
            }
            notices_.push_back({Notice::Level::Warn, "Unsupported baudrate " + std::to_string(config.baudrate) +
                                                         " for '" + config.device + "', using 115200"});
            baud = B115200;
        }

        fd_ = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            error_ = errnoText("Failed to open serial port '" + config.device + "'");
            return false;
        }

        termios options{};
        if (tcgetattr(fd_, &options) != 0)
        {
            error_ = errnoText("tcgetattr failed");
            close();
            return false;
        }
        cfmakeraw(&options);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &options) != 0)
        {
            error_ = errnoText("tcsetattr failed");
            close();
            return false;
        }

        if (config.low_latency)
        {
            for (auto& notice : configureUsbLatency(config.device))
            {
                notices_.push_back(std::move(notice));
            }
        }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void PosixSerialPort::close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    IoStatus PosixSerialPort::read(uint8_t* buffer, std::size_t capacity, std::size_t& n)
    {
        n = 0;
        if (fd_ < 0)
        {
            error_ = "serial port is not open";
            return IoStatus::Error;
        }
        const ssize_t got = ::read(fd_, buffer, capacity);
        if (got < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                return IoStatus::WouldBlock;
            }
            error_ = errnoText("[HW-012] read failed");
            return IoStatus::Error;
        }
        if (got == 0)
        {
            return IoStatus::WouldBlock;
        }
        n = static_cast<std::size_t>(got);
        return IoStatus::Ok;
    }

    IoStatus PosixSerialPort::write(const uint8_t* data, std::size_t length, std::size_t& n)
    {
        n = 0;
        if (fd_ < 0)
        {
            error_ = "serial port is not open";
            return IoStatus::Error;
        }
        const ssize_t put = ::write(fd_, data, length);
        if (put > 0)
        {
            n = static_cast<std::size_t>(put);
            return IoStatus::Ok;
        }
        if (put < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        {
            return IoStatus::WouldBlock;
        }
        if (put == 0)
        {
            return IoStatus::WouldBlock;
        }
        error_ = std::strerror(errno);
        return IoStatus::Error;
    }

    IoStatus PosixSerialPort::wait(bool for_read, std::chrono::microseconds timeout)
    {
        if (fd_ < 0)
        {
            error_ = "serial port is not open";
            return IoStatus::Error;
        }
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_, &set);
        timeval tv{};
        tv.tv_sec = static_cast<time_t>(timeout.count() / 1'000'000);
        tv.tv_usec = static_cast<suseconds_t>(timeout.count() % 1'000'000);
        const int result = for_read ? ::select(fd_ + 1, &set, nullptr, nullptr, &tv)
                                    : ::select(fd_ + 1, nullptr, &set, nullptr, &tv);
        if (result < 0)
        {
            if (errno == EINTR)
            {
                return IoStatus::WouldBlock;
            }
            error_ = errnoText("select failed");
            return IoStatus::Error;
        }
        return (result > 0 && FD_ISSET(fd_, &set)) ? IoStatus::Ok : IoStatus::WouldBlock;
    }

    IoStatus PosixSerialPort::waitReadable(std::chrono::microseconds timeout)
    {
        return wait(true, timeout);
    }

    IoStatus PosixSerialPort::waitWritable(std::chrono::microseconds timeout)
    {
        return wait(false, timeout);
    }

    void PosixSerialPort::flushInput()
    {
        if (fd_ >= 0)
        {
            tcflush(fd_, TCIFLUSH);
        }
    }

    void PosixSerialPort::flushAll()
    {
        if (fd_ >= 0)
        {
            tcflush(fd_, TCIOFLUSH);
        }
    }

    void PosixSerialPort::drain()
    {
        if (fd_ >= 0)
        {
            tcdrain(fd_);
        }
    }

    std::vector<Notice> PosixSerialPort::takeNotices()
    {
        std::vector<Notice> out;
        out.swap(notices_);
        return out;
    }
} // namespace robot_comm
