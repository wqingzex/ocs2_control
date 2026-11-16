#include <gtest/gtest.h>

#include "robot_comm/bus_hub.h"
#include "robot_comm/mock_bus.h"

using namespace robot_comm;

namespace
{
    /** A usb_rs485 / usb_serial factory that hands out MockBus objects and counts them. */
    struct FakeUsbFactory
    {
        int opened = 0;
        bool fail = false;
        void install()
        {
            for (const char* kind : {"usb_rs485", "usb_serial"})
            {
                BusHub::instance().registerFactory(
                    kind, [this](const BusName& name, std::string& error) -> std::shared_ptr<Bus> {
                        if (fail)
                        {
                            error = "cannot open " + name.device;
                            return nullptr;
                        }
                        ++opened;
                        auto bus = std::make_shared<MockBus>(name.text());
                        bus->setResponder([](const auto& r) { return std::optional(r); });
                        return bus;
                    });
            }
        }
    };
}  // namespace

TEST(BusHub, SameLineSameBusSeparateClients)
{
    FakeUsbFactory factory;
    factory.install();
    std::string error;
    auto a = BusHub::instance().connect("usb_rs485:///dev/ttyHUB0", error);
    auto b = BusHub::instance().connect("usb_rs485:///dev/ttyHUB0?baud=115200", error);
    ASSERT_TRUE(a && b) << error;
    EXPECT_EQ(factory.opened, 1);
    ASSERT_TRUE(a->submit({1}, {}, 11, error)) << error;
    ASSERT_TRUE(b->submit({2}, {}, 22, error)) << error;
    const auto da = a->poll();
    const auto db = b->poll();
    ASSERT_TRUE(da && db);
    EXPECT_EQ(da->tag, 11u);
    EXPECT_EQ(db->tag, 22u);
    EXPECT_EQ(a->stats().sent, 2u);  // one line: both requests went over it
}

TEST(BusHub, UsbPortWithOtherOptionsIsRefused)
{
    FakeUsbFactory factory;
    factory.install();
    std::string error;
    auto a = BusHub::instance().connect("usb_rs485:///dev/ttyHUB1", error);
    ASSERT_TRUE(a) << error;
    EXPECT_FALSE(BusHub::instance().connect("usb_serial:///dev/ttyHUB1?baud=2500000", error));
    EXPECT_NE(error.find("already open with baud=115200"), std::string::npos) << error;
}

TEST(BusHub, UsbBusClosesWithItsLastClient)
{
    FakeUsbFactory factory;
    factory.install();
    std::string error;
    {
        auto a = BusHub::instance().connect("usb_rs485:///dev/ttyHUB2", error);
        ASSERT_TRUE(a);
    }
    auto again = BusHub::instance().connect("usb_rs485:///dev/ttyHUB2?baud=9600", error);
    ASSERT_TRUE(again) << error;  // reopened, so other options are fine now
    EXPECT_EQ(factory.opened, 2);
}

TEST(BusHub, OpenFailureIsReported)
{
    FakeUsbFactory factory;
    factory.fail = true;
    factory.install();
    std::string error;
    EXPECT_FALSE(BusHub::instance().connect("usb_rs485:///dev/ttyHUB3", error));
    EXPECT_NE(error.find("cannot open /dev/ttyHUB3"), std::string::npos) << error;
}

TEST(BusHub, ArmBusIsNotReadyUntilProvidedAndAfterWithdrawn)
{
    std::string error;
    auto hand = BusHub::instance().connect("arm_rs485://10.9.9.9/left/port1", error);
    ASSERT_TRUE(hand) << error;
    EXPECT_EQ(hand->state(), BusState::NotReady);
    EXPECT_FALSE(hand->submit({1}, {}, 1, error));
    EXPECT_NE(error.find("arm not connected"), std::string::npos) << error;

    auto bus = std::make_shared<MockBus>("arm_rs485://10.9.9.9/left/port1");
    bus->setResponder([](const auto& r) { return std::optional(r); });
    auto provision = BusHub::instance().provide("arm_rs485://10.9.9.9/left/port1", bus, error);
    ASSERT_TRUE(provision) << error;
    EXPECT_FALSE(BusHub::instance().provide("arm_rs485://10.9.9.9/left/port1", bus, error));
    EXPECT_NE(error.find("already provided"), std::string::npos);

    EXPECT_EQ(hand->state(), BusState::Idle);
    ASSERT_TRUE(hand->submit({7}, {}, 3, error)) << error;
    const auto done = hand->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->reply, (std::vector<uint8_t>{7}));

    provision.reset();
    EXPECT_EQ(hand->state(), BusState::NotReady);
}

TEST(BusHub, OtherSideIsAnotherBus)
{
    std::string error;
    auto right = BusHub::instance().connect("arm_rs485://10.9.9.8/right/port1", error);
    auto provision = BusHub::instance().provide("arm_rs485://10.9.9.8/left/port1", std::make_shared<MockBus>(), error);
    ASSERT_TRUE(right && provision) << error;
    EXPECT_EQ(right->state(), BusState::NotReady);
}

TEST(BusHub, RejectsBadNamesAndCan)
{
    std::string error;
    EXPECT_FALSE(BusHub::instance().connect("arm_rs485://10.0.0.1/A/port1", error));
    EXPECT_NE(error.find("side must be left or right"), std::string::npos);
    EXPECT_FALSE(BusHub::instance().connect("usb_can://can0", error));
    EXPECT_NE(error.find("CAN hand driver"), std::string::npos);
}

TEST(BusHub, PendingRequestOfAWithdrawnBusCompletesWithAnError)
{
    std::string error;
    auto first = std::make_shared<MockBus>("arm_rs485://10.9.0.1/left/port1");
    first->setAutoPump(false);  // the request stays in flight
    auto provision = BusHub::instance().provide("arm_rs485://10.9.0.1/left/port1", first, error);
    ASSERT_TRUE(provision) << error;
    auto client = BusHub::instance().connect("arm_rs485://10.9.0.1/left/port1", error);
    ASSERT_TRUE(client) << error;
    ASSERT_TRUE(client->submit({1}, ReplySpec{}, 42, error)) << error;

    provision.reset();  // the arm driver went away (re-initialized)
    auto second = std::make_shared<MockBus>("arm_rs485://10.9.0.1/left/port1");
    second->setResponder([](const auto& r) { return std::optional(r); });
    provision = BusHub::instance().provide("arm_rs485://10.9.0.1/left/port1", second, error);
    ASSERT_TRUE(provision) << error;

    auto done = client->poll();
    ASSERT_TRUE(done);  // not lost: the device would otherwise wait for it forever
    EXPECT_EQ(done->status, CompletionStatus::Error);
    EXPECT_EQ(done->tag, 42u);
    EXPECT_FALSE(client->poll());

    ASSERT_TRUE(client->submit({2}, ReplySpec{}, 43, error)) << error;  // the new bus works
    done = client->poll();
    ASSERT_TRUE(done);
    EXPECT_EQ(done->status, CompletionStatus::Ok);
    EXPECT_EQ(done->tag, 43u);
}
