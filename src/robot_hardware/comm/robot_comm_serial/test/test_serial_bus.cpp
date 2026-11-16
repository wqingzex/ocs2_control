#include <gtest/gtest.h>

#include <thread>

#include "robot_comm_serial/mock_serial_port.h"
#include "robot_comm_serial/serial_bus.h"

using namespace robot_comm;
using namespace std::chrono_literals;

namespace
{
    struct Fixture
    {
        MockSerialPort* port = nullptr;  ///< owned by bus
        std::unique_ptr<SerialBus> bus;
        ClientId client = 0;

        Fixture()
        {
            auto owned = std::make_unique<MockSerialPort>();
            port = owned.get();
            SerialConfig config;
            config.device = "/dev/ttyMOCK";
            bus = std::make_unique<SerialBus>("usb_rs485:///dev/ttyMOCK", std::move(owned), config);
        }
        void start()
        {
            std::string error;
            ASSERT_TRUE(bus->start(error)) << error;
            client = bus->attach();
        }
        /** The "device": answers every request with `reply`. */
        void answerWith(std::vector<uint8_t> reply)
        {
            port->setWriteHook([this, reply](const std::vector<uint8_t>&) { port->pushReceived(reply); });
        }
    };

    ReplySpec fixedLength(std::size_t n, std::chrono::milliseconds timeout = 200ms)
    {
        ReplySpec spec;
        spec.frame_length = [n](const std::vector<uint8_t>&) { return n; };
        spec.timeout = timeout;
        return spec;
    }

    std::optional<Completion> waitFor(SerialBus& bus, ClientId client, std::chrono::milliseconds limit = 1s)
    {
        const auto until = std::chrono::steady_clock::now() + limit;
        while (std::chrono::steady_clock::now() < until)
        {
            if (auto done = bus.poll(client)) return done;
            std::this_thread::sleep_for(1ms);
        }
        return std::nullopt;
    }
}  // namespace

TEST(SerialBus, OpenFailureIsReported)
{
    Fixture f;
    f.port->setOpenFails(true);
    std::string error;
    EXPECT_FALSE(f.bus->start(error));
    EXPECT_NE(error.find("cannot open /dev/ttyMOCK"), std::string::npos) << error;
    EXPECT_EQ(f.bus->state(), BusState::NotReady);
}

TEST(SerialBus, TransactReturnsTheReplyOfTheFrameLength)
{
    Fixture f;
    f.start();
    f.answerWith({0x09, 0x04, 0x02, 0x00, 0x2A, 0xFF, 0xFF, 0xEE});  // trailing bytes beyond the frame are dropped
    const auto done = f.bus->transact(f.client, {0x09, 0x04, 0x00, 0x00}, fixedLength(7), 500ms);
    ASSERT_EQ(done.status, CompletionStatus::Ok) << done.error;
    EXPECT_EQ(done.reply, (std::vector<uint8_t>{0x09, 0x04, 0x02, 0x00, 0x2A, 0xFF, 0xFF}));
    EXPECT_EQ(f.port->written(), (std::vector<uint8_t>{0x09, 0x04, 0x00, 0x00}));
}

TEST(SerialBus, AsyncSubmitAndPoll)
{
    Fixture f;
    f.start();
    f.answerWith({1, 2, 3});
    std::string error;
    ASSERT_TRUE(f.bus->submit(f.client, {0xAA}, fixedLength(3), 42, error)) << error;
    const auto done = waitFor(*f.bus, f.client);
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
    EXPECT_EQ(done->tag, 42u);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{1, 2, 3}));
}

TEST(SerialBus, ReplyOfUnknownLengthEndsAtSilence)
{
    Fixture f;
    f.start();
    f.answerWith({5, 6, 7, 8});
    ReplySpec spec;  // no frame_length
    spec.timeout = 500ms;
    spec.inter_byte_gap = 3ms;
    const auto done = f.bus->transact(f.client, {1}, spec, 500ms);
    ASSERT_EQ(done.status, CompletionStatus::Ok) << done.error;
    EXPECT_EQ(done.reply, (std::vector<uint8_t>{5, 6, 7, 8}));
}

TEST(SerialBus, PartialReplyTimesOut)
{
    Fixture f;
    f.start();
    f.answerWith({1, 2});
    const auto done = f.bus->transact(f.client, {1}, fixedLength(5, 30ms), 500ms);
    EXPECT_EQ(done.status, CompletionStatus::Timeout);
    EXPECT_EQ(done.reply, (std::vector<uint8_t>{1, 2}));
    EXPECT_EQ(f.bus->stats().timeouts, 1u);
}

TEST(SerialBus, StaleInputIsDiscardedBeforeEachRequest)
{
    Fixture f;
    f.start();
    f.port->pushReceived({0xDE, 0xAD});  // a late reply to something earlier
    f.answerWith({0x11});
    const auto done = f.bus->transact(f.client, {1}, fixedLength(1), 500ms);
    ASSERT_EQ(done.status, CompletionStatus::Ok);
    EXPECT_EQ(done.reply, (std::vector<uint8_t>{0x11}));
    EXPECT_GE(f.port->flushInputCalls(), 1);
}

TEST(SerialBus, WriteErrorFailsTheBus)
{
    Fixture f;
    f.start();
    f.port->setWriteFails(true);
    const auto done = f.bus->transact(f.client, {1}, fixedLength(1), 500ms);
    EXPECT_EQ(done.status, CompletionStatus::Error);
    EXPECT_EQ(f.bus->state(), BusState::Failed);
    std::string error;
    EXPECT_FALSE(f.bus->submit(f.client, {1}, fixedLength(1), 1, error));
}

TEST(SerialBus, NoAckWriteCompletesWhenSent)
{
    Fixture f;
    f.start();
    ReplySpec spec;
    spec.expect_reply = false;
    const auto done = f.bus->transact(f.client, {7, 7}, spec, 500ms);
    EXPECT_EQ(done.status, CompletionStatus::Ok);
    EXPECT_EQ(f.port->written(), (std::vector<uint8_t>{7, 7}));
}

TEST(SerialBus, ReopensThePortAfterAFailureAndWorksAgain)
{
    Fixture f;
    f.bus->setReopenInterval(20ms);
    f.start();
    f.answerWith({7});
    f.port->setWriteFails(true);
    f.port->setOpenFails(true);  // unplugged: the port cannot be reopened yet
    EXPECT_EQ(f.bus->transact(f.client, {1}, fixedLength(1), 500ms).status, CompletionStatus::Error);
    EXPECT_EQ(f.bus->state(), BusState::Failed);
    std::this_thread::sleep_for(60ms);
    EXPECT_EQ(f.bus->state(), BusState::Failed);  // still unplugged

    f.port->setWriteFails(false);
    f.port->setOpenFails(false);  // plugged in again
    const auto until = std::chrono::steady_clock::now() + 1s;
    while (f.bus->state() == BusState::Failed && std::chrono::steady_clock::now() < until)
    {
        std::this_thread::sleep_for(5ms);
    }
    ASSERT_EQ(f.bus->state(), BusState::Idle);
    const auto done = f.bus->transact(f.client, {1}, fixedLength(1), 500ms);
    EXPECT_EQ(done.status, CompletionStatus::Ok);
    EXPECT_EQ(done.reply, std::vector<uint8_t>{7});
}
