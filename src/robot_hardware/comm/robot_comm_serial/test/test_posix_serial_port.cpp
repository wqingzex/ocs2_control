// PosixSerialPort against a pseudo-terminal: the real termios code, no adapter needed.

#include <gtest/gtest.h>

#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "robot_comm_serial/posix_serial_port.h"

using robot_comm::IoStatus;
using robot_comm::PosixSerialPort;
using robot_comm::SerialConfig;
using namespace std::chrono_literals;

class PosixSerialPortTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(master_, 0);
        ASSERT_EQ(::grantpt(master_), 0);
        ASSERT_EQ(::unlockpt(master_), 0);
        slave_path_ = ::ptsname(master_);
        // raw on the master side too, so bytes pass through untouched
        termios t{};
        ::tcgetattr(master_, &t);
        ::cfmakeraw(&t);
        ::tcsetattr(master_, TCSANOW, &t);
    }

    void TearDown() override { ::close(master_); }

    SerialConfig config(int baud = 115200)
    {
        SerialConfig c;
        c.device = slave_path_;
        c.baudrate = baud;
        return c;
    }

    void masterWrite(const std::vector<uint8_t>& data) { ASSERT_EQ(::write(master_, data.data(), data.size()), static_cast<ssize_t>(data.size())); }

    std::vector<uint8_t> masterRead()
    {
        std::vector<uint8_t> out;
        uint8_t buffer[64];
        const int flags = ::fcntl(master_, F_GETFL);
        ::fcntl(master_, F_SETFL, flags | O_NONBLOCK);
        for (;;)
        {
            const ssize_t n = ::read(master_, buffer, sizeof(buffer));
            if (n <= 0) break;
            out.insert(out.end(), buffer, buffer + n);
        }
        return out;
    }

    int master_ = -1;
    std::string slave_path_;
};

TEST_F(PosixSerialPortTest, OpensAndConfiguresRaw8N1)
{
    PosixSerialPort port;

    ASSERT_TRUE(port.open(config(115200))) << port.lastError();

    EXPECT_TRUE(port.isOpen());
    const int fd = ::open(slave_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    termios t{};
    ASSERT_EQ(::tcgetattr(fd, &t), 0);
    ::close(fd);
    EXPECT_EQ(t.c_cflag & CSIZE, static_cast<tcflag_t>(CS8));
    EXPECT_EQ(t.c_cflag & PARENB, 0u);
    EXPECT_EQ(t.c_cflag & CSTOPB, 0u);
    EXPECT_EQ(t.c_cflag & CRTSCTS, 0u);
    EXPECT_EQ(t.c_cc[VMIN], 0);
    EXPECT_EQ(t.c_cc[VTIME], 0);
    EXPECT_EQ(cfgetispeed(&t), static_cast<speed_t>(B115200));
}

TEST_F(PosixSerialPortTest, ReadsWhatTheOtherEndSendsAndWouldBlockWhenThereIsNothing)
{
    PosixSerialPort port;
    ASSERT_TRUE(port.open(config()));
    uint8_t buffer[16];
    std::size_t n = 99;

    EXPECT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::WouldBlock);
    EXPECT_EQ(n, 0u);

    masterWrite({0xAA, 0x01, 0x02});
    ASSERT_EQ(port.waitReadable(100ms), IoStatus::Ok);
    ASSERT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::Ok);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(buffer[0], 0xAA);
    EXPECT_EQ(buffer[2], 0x02);
}

TEST_F(PosixSerialPortTest, WaitReadableTimesOutWhenNothingArrives)
{
    PosixSerialPort port;
    ASSERT_TRUE(port.open(config()));

    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(port.waitReadable(20ms), IoStatus::WouldBlock);

    EXPECT_GE(std::chrono::steady_clock::now() - start, 15ms);
}

TEST_F(PosixSerialPortTest, WritesReachTheOtherEnd)
{
    PosixSerialPort port;
    ASSERT_TRUE(port.open(config()));
    const uint8_t data[] = {0x55, 0xAA, 0x00, 0xFF};
    std::string error;

    ASSERT_TRUE(robot_comm::writeAll(port, data, sizeof(data), 50ms, error)) << error;
    port.drain();

    EXPECT_EQ(masterRead(), (std::vector<uint8_t>{0x55, 0xAA, 0x00, 0xFF}));
}

TEST_F(PosixSerialPortTest, FlushInputDropsBytesThatAreWaiting)
{
    PosixSerialPort port;
    ASSERT_TRUE(port.open(config()));
    masterWrite({1, 2, 3});
    ASSERT_EQ(port.waitReadable(100ms), IoStatus::Ok);

    port.flushInput();

    uint8_t buffer[8];
    std::size_t n = 0;
    EXPECT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::WouldBlock);
}

TEST_F(PosixSerialPortTest, OpeningFlushesWhatWasThereBefore)
{
    masterWrite({9, 9, 9});
    PosixSerialPort port;

    ASSERT_TRUE(port.open(config()));

    uint8_t buffer[8];
    std::size_t n = 0;
    EXPECT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::WouldBlock) << "stale bytes must not leak into the first frame";
}

TEST_F(PosixSerialPortTest, AnUnsupportedBaudrateIsAnErrorOrFallsBackWhenAsked)
{
    PosixSerialPort strict;
    EXPECT_FALSE(strict.open(config(12345)));
    EXPECT_NE(strict.lastError().find("12345"), std::string::npos);

    SerialConfig lenient = config(12345);
    lenient.strict_baudrate = false;
    PosixSerialPort port;
    ASSERT_TRUE(port.open(lenient)) << port.lastError();
    const auto notices = port.takeNotices();
    ASSERT_EQ(notices.size(), 1u);
    EXPECT_EQ(notices[0].level, robot_comm::Notice::Level::Warn);
    EXPECT_TRUE(port.takeNotices().empty()) << "notices are handed out once";
}

TEST_F(PosixSerialPortTest, OpeningADeviceThatDoesNotExistFailsWithTheDeviceName)
{
    PosixSerialPort port;
    SerialConfig c;
    c.device = "/dev/does_not_exist_ttyUSB9";

    EXPECT_FALSE(port.open(c));
    EXPECT_NE(port.lastError().find("does_not_exist_ttyUSB9"), std::string::npos);
    EXPECT_FALSE(port.isOpen());
}

TEST_F(PosixSerialPortTest, IoOnAClosedPortIsAnError)
{
    PosixSerialPort port;
    uint8_t buffer[1];
    std::size_t n = 0;

    EXPECT_EQ(port.read(buffer, 1, n), IoStatus::Error);
    EXPECT_EQ(port.write(buffer, 1, n), IoStatus::Error);
    EXPECT_EQ(port.waitReadable(1ms), IoStatus::Error);
}

TEST_F(PosixSerialPortTest, ThePtyIsNotAUsbAdapterSoNoLatencyNoticeComesBack)
{
    PosixSerialPort port;
    ASSERT_TRUE(port.open(config()));

    EXPECT_TRUE(port.takeNotices().empty());
}
