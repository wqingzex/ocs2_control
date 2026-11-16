#include <gtest/gtest.h>

#include <thread>

#include "robot_comm/mock_bus.h"

using namespace robot_comm;
using namespace std::chrono_literals;

namespace
{
    std::vector<uint8_t> echoPlusOne(const std::vector<uint8_t>& request)
    {
        std::vector<uint8_t> reply = request;
        for (auto& b : reply) b = static_cast<uint8_t>(b + 1);
        return reply;
    }
}  // namespace

TEST(QueuedBus, CompletesOnlyForTheSubmittingClient)
{
    MockBus bus;
    bus.setResponder([](const auto& r) { return std::optional(echoPlusOne(r)); });
    const ClientId a = bus.attach();
    const ClientId b = bus.attach();
    std::string error;
    ASSERT_TRUE(bus.submit(a, {1, 2}, {}, 7, error)) << error;
    EXPECT_FALSE(bus.poll(b).has_value());
    const auto done = bus.poll(a);
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
    EXPECT_EQ(done->tag, 7u);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{2, 3}));
    EXPECT_FALSE(bus.poll(a).has_value());
}

TEST(QueuedBus, OneOutstandingRequestPerClientAndFifoAcrossClients)
{
    MockBus bus;
    bus.setAutoPump(false);
    bus.setResponder([](const auto& r) { return std::optional(r); });
    const ClientId a = bus.attach();
    const ClientId b = bus.attach();
    std::string error;
    ASSERT_TRUE(bus.submit(a, {0xA1}, {}, 1, error));
    EXPECT_FALSE(bus.submit(a, {0xA2}, {}, 2, error));
    EXPECT_NE(error.find("previous request still pending"), std::string::npos);
    ASSERT_TRUE(bus.submit(b, {0xB1}, {}, 3, error));
    EXPECT_EQ(bus.state(), BusState::Busy);

    ASSERT_TRUE(bus.pumpOne());
    ASSERT_TRUE(bus.pumpOne());
    EXPECT_FALSE(bus.pumpOne());
    EXPECT_EQ(bus.sent(), (std::vector<std::vector<uint8_t>>{{0xA1}, {0xB1}}));
    EXPECT_EQ(bus.state(), BusState::Idle);
    ASSERT_TRUE(bus.submit(a, {0xA2}, {}, 2, error)) << error;  // free again once its first request finished
}

TEST(QueuedBus, NoReplyIsATimeoutAndNoAckWritesNeedNone)
{
    MockBus bus;  // no responder
    const ClientId a = bus.attach();
    std::string error;
    ASSERT_TRUE(bus.submit(a, {1}, {}, 1, error));
    auto done = bus.poll(a);
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Timeout);

    ReplySpec no_ack;
    no_ack.expect_reply = false;
    ASSERT_TRUE(bus.submit(a, {2}, no_ack, 2, error));
    done = bus.poll(a);
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
    EXPECT_EQ(bus.stats().timeouts, 1u);
    EXPECT_EQ(bus.stats().ok, 1u);
    EXPECT_EQ(bus.stats().sent, 2u);
}

TEST(QueuedBus, NotReadyRefusesAndGoingDownFailsWhatIsPending)
{
    MockBus bus("mock://arm", /*ready=*/false);
    bus.setAutoPump(false);
    const ClientId a = bus.attach();
    const ClientId b = bus.attach();
    std::string error;
    EXPECT_EQ(bus.state(), BusState::NotReady);
    EXPECT_FALSE(bus.submit(a, {1}, {}, 1, error));
    EXPECT_NE(error.find("not ready"), std::string::npos);

    bus.setReady(true);
    ASSERT_TRUE(bus.submit(a, {1}, {}, 1, error));
    ASSERT_TRUE(bus.submit(b, {2}, {}, 2, error));
    bus.setReady(false, "arm disconnected");
    for (const ClientId c : {a, b})
    {
        const auto done = bus.poll(c);
        ASSERT_TRUE(done);
        EXPECT_EQ(done->status, CompletionStatus::Error);
        EXPECT_NE(done->error.find("arm disconnected"), std::string::npos);
    }
    bus.setReady(true);
    EXPECT_TRUE(bus.submit(a, {3}, {}, 3, error)) << error;  // nothing left outstanding
}

TEST(QueuedBus, FailedIsReported)
{
    MockBus bus;
    const ClientId a = bus.attach();
    bus.setFailed("port lost");
    EXPECT_EQ(bus.state(), BusState::Failed);
    std::string error;
    EXPECT_FALSE(bus.submit(a, {1}, {}, 1, error));
    EXPECT_NE(error.find("port lost"), std::string::npos);
}

TEST(QueuedBus, DetachDropsQueuedRequests)
{
    MockBus bus;
    bus.setAutoPump(false);
    const ClientId a = bus.attach();
    std::string error;
    ASSERT_TRUE(bus.submit(a, {1}, {}, 1, error));
    bus.detach(a);
    EXPECT_FALSE(bus.pumpOne());
    EXPECT_FALSE(bus.poll(a).has_value());
}

TEST(QueuedBus, TransactWaitsForTheReply)
{
    MockBus bus;
    bus.setResponder([](const auto& r) { return std::optional(echoPlusOne(r)); });
    const ClientId a = bus.attach();
    const auto done = bus.transact(a, {9}, {}, 100ms);
    EXPECT_EQ(done.status, CompletionStatus::Ok);
    EXPECT_EQ(done.reply, (std::vector<uint8_t>{10}));
    EXPECT_FALSE(bus.poll(a).has_value());  // a synchronous result is not also queued for poll()
}

TEST(QueuedBus, TransactFromThePumpThreadIsRefused)
{
    MockBus bus;
    bus.setPumpThread(std::this_thread::get_id());
    const ClientId a = bus.attach();
    const auto done = bus.transact(a, {1}, {}, 10ms);
    EXPECT_EQ(done.status, CompletionStatus::Error);
    EXPECT_NE(done.error.find("thread that pumps this bus"), std::string::npos);
    EXPECT_TRUE(bus.sent().empty());
}

TEST(QueuedBus, TransactFromAnotherThreadWhileThePumpRuns)
{
    MockBus bus;
    bus.setAutoPump(false);
    bus.setResponder([](const auto& r) { return std::optional(r); });
    bus.setPumpThread(std::this_thread::get_id());
    const ClientId a = bus.attach();
    Completion result;
    std::thread device([&] { result = bus.transact(a, {5}, {}, 500ms); });
    // The "control thread" keeps pumping; the device thread waits for its reply.
    const auto until = std::chrono::steady_clock::now() + 2s;
    while (bus.sent().empty() && std::chrono::steady_clock::now() < until)
    {
        bus.pumpOne();
        std::this_thread::sleep_for(1ms);
    }
    device.join();
    EXPECT_EQ(result.status, CompletionStatus::Ok);
    EXPECT_EQ(result.reply, (std::vector<uint8_t>{5}));
}
