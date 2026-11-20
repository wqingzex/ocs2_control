// The flange buses of one arm (ArmBuses) on fake SDK channel functions: no SDK, no arm.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <thread>
#include <vector>

#include "marvin_hardware/comm/arm_bus.h"
#include "robot_comm/bus_hub.h"

using marvin_hardware::ArmBuses;
using marvin_hardware::CAN_CHANNEL;
using marvin_hardware::COM1_CHANNEL;
using marvin_hardware::COM2_CHANNEL;
using robot_comm::CompletionStatus;
using robot_comm::ReplySpec;

namespace
{
    struct Sent
    {
        std::vector<uint8_t> frame;
        long channel;
    };
    struct Chunk
    {
        std::vector<uint8_t> bytes;
        long channel;
    };

    // The SDK's per-arm buffers, as the fake send / get functions see them.
    std::vector<Sent> g_sent;
    std::deque<Chunk> g_rx;
    bool g_send_ok = true;

    bool fakeSend(uint8_t* data, long size, long channel)
    {
        g_sent.push_back({std::vector<uint8_t>(data, data + size), channel});
        return g_send_ok;
    }

    long fakeGet(unsigned char* buffer, long* channel)
    {
        if (g_rx.empty()) return 0;
        Chunk chunk = g_rx.front();
        g_rx.pop_front();
        std::copy(chunk.bytes.begin(), chunk.bytes.end(), buffer);
        *channel = chunk.channel;
        return static_cast<long>(chunk.bytes.size());
    }

    ReplySpec fixedLength(std::size_t n)
    {
        ReplySpec spec;
        spec.frame_length = [n](const std::vector<uint8_t>&) { return n; };
        spec.timeout = std::chrono::milliseconds(20);
        return spec;
    }

    /** One arm whose flange buses are provided and connected; the test thread is the control thread. */
    class ArmBusTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            g_sent.clear();
            g_rx.clear();
            g_send_ok = true;
            std::string error;
            ASSERT_TRUE(buses.provide(error)) << error;
            buses.setConnected(true);
            port1 = robot_comm::BusHub::instance().connect("arm_rs485://10.0.0.9/left/port1", error);
            port2 = robot_comm::BusHub::instance().connect("arm_rs485://10.0.0.9/left/port2", error);
            canfd = robot_comm::BusHub::instance().connect("arm_canfd://10.0.0.9/left", error);
            ASSERT_TRUE(port1 && port2 && canfd) << error;
        }

        /** One control cycle: read() then write(). */
        void cycle()
        {
            buses.pull();
            buses.push();
        }

        ArmBuses buses{"10.0.0.9", "left", &fakeSend, &fakeGet};
        std::shared_ptr<robot_comm::BusClient> port1, port2, canfd;
    };
}  // namespace

TEST_F(ArmBusTest, RequestGoesOutOnItsChannelAndTheReplyComesBackOnTheNextRead)
{
    std::string error;
    ASSERT_TRUE(port1->submit({0x01, 0x03}, fixedLength(3), 7, error)) << error;
    buses.push();
    ASSERT_EQ(g_sent.size(), 1u);
    EXPECT_EQ(g_sent[0].channel, COM1_CHANNEL);
    EXPECT_EQ(g_sent[0].frame, (std::vector<uint8_t>{0x01, 0x03}));
    EXPECT_FALSE(port1->poll());  // still in flight

    g_rx.push_back({{0xAA, 0xBB}, COM1_CHANNEL});
    g_rx.push_back({{0xCC, 0xDD}, COM1_CHANNEL});  // more than the frame: cut at frame_length
    buses.pull();
    auto done = port1->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
    EXPECT_EQ(done->tag, 7u);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{0xAA, 0xBB, 0xCC}));
}

TEST_F(ArmBusTest, PortsOfOneArmShareOneRequestInFlightAndTakeTurns)
{
    std::string error;
    ASSERT_TRUE(port1->submit({0x11}, fixedLength(1), 1, error));
    ASSERT_TRUE(port2->submit({0x22}, fixedLength(1), 2, error));
    ASSERT_TRUE(canfd->submit({0x33}, fixedLength(1), 3, error));

    buses.push();
    buses.push();  // the first one is still in flight: nothing more on the wire
    ASSERT_EQ(g_sent.size(), 1u);

    std::vector<long> order{g_sent[0].channel};
    for (int i = 0; i < 2; ++i)
    {
        g_rx.push_back({{0x00}, order.back()});
        cycle();
        ASSERT_EQ(g_sent.size(), order.size() + 1);
        order.push_back(g_sent.back().channel);
    }
    EXPECT_EQ(order, (std::vector<long>{COM1_CHANNEL, COM2_CHANNEL, CAN_CHANNEL}));
}

TEST_F(ArmBusTest, BytesOfAnotherChannelAreNotTakenAsTheReply)
{
    std::string error;
    ASSERT_TRUE(port2->submit({0x01}, fixedLength(2), 0, error));
    buses.push();
    g_rx.push_back({{0x99, 0x99}, COM1_CHANNEL});
    g_rx.push_back({{0x05, 0x06}, COM2_CHANNEL});
    buses.pull();
    auto done = port2->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{0x05, 0x06}));
}

TEST_F(ArmBusTest, NoReplyTimesOut)
{
    std::string error;
    ASSERT_TRUE(port1->submit({0x01}, fixedLength(4), 0, error));
    buses.push();
    g_rx.push_back({{0x01}, COM1_CHANNEL});
    buses.pull();
    EXPECT_FALSE(port1->poll());
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    buses.pull();
    auto done = port1->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Timeout);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{0x01}));  // what did arrive

    // The line is free again.
    ASSERT_TRUE(port1->submit({0x02}, fixedLength(1), 0, error));
    buses.push();
    EXPECT_EQ(g_sent.size(), 2u);
}

TEST_F(ArmBusTest, CommandWithoutReplyCompletesWhenSent)
{
    std::string error;
    ReplySpec no_reply;
    no_reply.expect_reply = false;
    ASSERT_TRUE(port1->submit({0x06}, no_reply, 0, error));
    buses.push();
    auto done = port1->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
}

TEST_F(ArmBusTest, RefusedSendFailsTheRequest)
{
    g_send_ok = false;
    std::string error;
    ASSERT_TRUE(port1->submit({0x06}, fixedLength(1), 0, error));
    buses.push();
    auto done = port1->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Error);
}

TEST_F(ArmBusTest, IdleBusDoesNotReadTheSdkBuffer)
{
    g_rx.push_back({{0x42}, COM1_CHANNEL});
    buses.pull();
    EXPECT_EQ(g_rx.size(), 1u);  // left to whoever else reads it
}

TEST_F(ArmBusTest, DisconnectFailsWhatIsPendingAndRefusesNewRequests)
{
    std::string error;
    ASSERT_TRUE(port1->submit({0x01}, fixedLength(1), 0, error));
    buses.push();
    buses.setConnected(false);
    auto done = port1->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Error);
    EXPECT_FALSE(port1->submit({0x01}, fixedLength(1), 0, error));

    buses.setConnected(true);
    EXPECT_TRUE(port1->submit({0x01}, fixedLength(1), 0, error)) << error;
}

TEST_F(ArmBusTest, TransactFromTheControlThreadIsRefused)
{
    buses.push();  // the control thread is now the pump
    auto done = port1->transact({0x01}, fixedLength(1), std::chrono::milliseconds(10));
    EXPECT_EQ(done.status, CompletionStatus::Error);
}

TEST(ArmBusProvide, TheSameArmCannotBeProvidedTwice)
{
    ArmBuses first{"10.0.0.8", "right", &fakeSend, &fakeGet};
    ArmBuses second{"10.0.0.8", "right", &fakeSend, &fakeGet};
    std::string error;
    ASSERT_TRUE(first.provide(error)) << error;
    EXPECT_FALSE(second.provide(error));
}
