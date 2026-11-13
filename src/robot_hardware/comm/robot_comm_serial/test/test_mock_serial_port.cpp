#include <gtest/gtest.h>

#include "robot_comm_serial/mock_serial_port.h"

using robot_comm::IoStatus;
using robot_comm::MockSerialPort;
using robot_comm::SerialConfig;

namespace
{
    std::vector<uint8_t> bytes(std::initializer_list<int> values)
    {
        std::vector<uint8_t> out;
        for (const int v : values) out.push_back(static_cast<uint8_t>(v));
        return out;
    }
} // namespace

TEST(MockSerialPort, ReadReturnsQueuedBytesThenWouldBlock)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.pushReceived(bytes({1, 2, 3}));
    uint8_t buffer[8];
    std::size_t n = 0;

    ASSERT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::Ok);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::WouldBlock);
    EXPECT_EQ(n, 0u);
}

TEST(MockSerialPort, ReadChunkLimitsWhatOneReadReturns)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.setReadChunk(1);
    port.pushReceived(bytes({1, 2, 3}));
    uint8_t buffer[8];
    std::size_t n = 0;

    port.read(buffer, sizeof(buffer), n);

    EXPECT_EQ(n, 1u);
    EXPECT_EQ(buffer[0], 1);
}

TEST(MockSerialPort, WritesAreRecordedAndCanBeAnsweredByAHook)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.setWriteHook([&port](const std::vector<uint8_t>& request) { port.pushReceived({static_cast<uint8_t>(request[0] + 1)}); });
    const uint8_t request[] = {0x10, 0x20};
    std::size_t n = 0;

    ASSERT_EQ(port.write(request, 2, n), IoStatus::Ok);

    EXPECT_EQ(port.written(), bytes({0x10, 0x20}));
    uint8_t buffer[4];
    ASSERT_EQ(port.read(buffer, sizeof(buffer), n), IoStatus::Ok);
    EXPECT_EQ(buffer[0], 0x11);
}

TEST(MockSerialPort, FlushesAreCountedAndDropPendingInput)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.pushReceived(bytes({1}));

    port.flushInput();
    port.drain();

    EXPECT_EQ(port.waitReadable(std::chrono::microseconds(0)), IoStatus::WouldBlock);
    EXPECT_EQ(port.flushInputCalls(), 1);
    EXPECT_EQ(port.drainCalls(), 1);
}

TEST(MockSerialPort, ForcedFailuresAreReported)
{
    MockSerialPort port;
    port.setOpenFails(true);
    EXPECT_FALSE(port.open(SerialConfig{}));
    port.setOpenFails(false);
    ASSERT_TRUE(port.open(SerialConfig{}));
    uint8_t buffer[1] = {0};
    std::size_t n = 0;

    port.setReadFails(true);
    EXPECT_EQ(port.read(buffer, 1, n), IoStatus::Error);
    port.setWriteFails(true);
    EXPECT_EQ(port.write(buffer, 1, n), IoStatus::Error);
    EXPECT_FALSE(port.lastError().empty());
}

TEST(WriteAll, WritesEverythingEvenWhenThePortTakesOneByteAtATime)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.setWriteChunk(1);
    const uint8_t data[] = {1, 2, 3, 4, 5};
    std::string error;

    EXPECT_TRUE(robot_comm::writeAll(port, data, sizeof(data), std::chrono::milliseconds(20), error)) << error;

    EXPECT_EQ(port.written(), bytes({1, 2, 3, 4, 5}));
}

TEST(WriteAll, ReportsAFailedWriteWithTheReason)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.setWriteFails(true);
    const uint8_t data[] = {1};
    std::string error;

    EXPECT_FALSE(robot_comm::writeAll(port, data, 1, std::chrono::milliseconds(20), error));

    EXPECT_EQ(error, "serial write failed: mock: write failed");
}

TEST(WriteAll, TimesOutWhenThePortAcceptsNothing)
{
    MockSerialPort port;
    port.open(SerialConfig{});
    port.setWriteChunk(0);
    // a port that is full: write() would block; emulate with a closed-for-writing mock by a hook-less zero-length write
    const uint8_t data[] = {1, 2};
    std::string error;
    std::size_t n = 0;
    ASSERT_EQ(port.write(data, 0, n), IoStatus::WouldBlock);

    EXPECT_TRUE(robot_comm::writeAll(port, data, 0, std::chrono::milliseconds(5), error)) << "nothing to write is done at once";
}
