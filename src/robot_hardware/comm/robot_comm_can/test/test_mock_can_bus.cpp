#include <gtest/gtest.h>

#include "robot_comm_can/mock_can_bus.h"

using robot_comm::CanConfig;
using robot_comm::CanFrame;
using robot_comm::CanReceive;
using robot_comm::MockCanBus;

namespace
{
    CanFrame frameWithId(uint32_t id)
    {
        CanFrame frame;
        frame.id = id;
        frame.len = 1;
        return frame;
    }
} // namespace

TEST(MockCanBus, SendNeedsAnOpenBus)
{
    MockCanBus bus;

    EXPECT_FALSE(bus.send(frameWithId(0x10)));
    EXPECT_EQ(bus.stats().tx_errors, 1u);
    EXPECT_TRUE(bus.open(CanConfig{}));
    EXPECT_TRUE(bus.send(frameWithId(0x10)));
    EXPECT_EQ(bus.stats().tx_frames, 1u);
}

TEST(MockCanBus, ReceivesQueuedFramesInOrderThenReportsEmpty)
{
    MockCanBus bus;
    bus.open(CanConfig{});
    bus.pushReceived(frameWithId(1));
    bus.pushReceived(frameWithId(2));
    CanFrame frame;

    EXPECT_TRUE(bus.waitReadable(std::chrono::milliseconds(0)));
    ASSERT_EQ(bus.receive(frame), CanReceive::Frame);
    EXPECT_EQ(frame.id, 1u);
    ASSERT_EQ(bus.receive(frame), CanReceive::Frame);
    EXPECT_EQ(frame.id, 2u);
    EXPECT_EQ(bus.receive(frame), CanReceive::Empty);
    EXPECT_FALSE(bus.waitReadable(std::chrono::milliseconds(0)));
}

TEST(MockCanBus, RecordsWhatWasSentAndWhichConfigItWasOpenedWith)
{
    MockCanBus bus;
    CanConfig config;
    config.interface = "can7";
    bus.open(config);

    bus.send(frameWithId(0x200));

    ASSERT_EQ(bus.sentFrames().size(), 1u);
    EXPECT_EQ(bus.sentFrames().front().id, 0x200u);
    EXPECT_EQ(bus.openedWith().interface, "can7");
    bus.clearSent();
    EXPECT_TRUE(bus.sentFrames().empty());
}

TEST(MockCanBus, ForcedFailuresAreReportedAndCounted)
{
    MockCanBus bus;
    bus.setOpenFails(true);
    EXPECT_FALSE(bus.open(CanConfig{}));
    bus.setOpenFails(false);
    ASSERT_TRUE(bus.open(CanConfig{}));

    bus.setSendFails(true);
    EXPECT_FALSE(bus.send(frameWithId(1)));
    bus.setReceiveFails(true);
    CanFrame frame;
    EXPECT_EQ(bus.receive(frame), CanReceive::Error);

    EXPECT_EQ(bus.stats().tx_errors, 1u);
    EXPECT_EQ(bus.stats().rx_errors, 1u);
    EXPECT_FALSE(bus.lastError().empty());
}

TEST(MockCanBus, TheSendHookCanAnswerAtOnce)
{
    MockCanBus bus;
    bus.open(CanConfig{});
    bus.setSendHook([&bus](const CanFrame& frame) { bus.pushReceived(frameWithId(frame.id + 1)); });

    ASSERT_TRUE(bus.send(frameWithId(0x10)));

    CanFrame answer;
    ASSERT_EQ(bus.receive(answer), CanReceive::Frame);
    EXPECT_EQ(answer.id, 0x11u);
}
