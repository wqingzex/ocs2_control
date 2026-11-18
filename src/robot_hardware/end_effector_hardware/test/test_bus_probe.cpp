// The bus measurement (bus_probe.h) on a MockBus with simulated devices: the numbers it reports, a device that never
// answers, and that every device can be configured from the default joints.
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <rclcpp/logger.hpp>

#include "end_effector_hardware/bus/bus_probe.h"
#include "end_effector_hardware/bus/jodell_rg75.h"
#include "end_effector_hardware/bus/linkerhand_l6.h"
#include "robot_comm/mock_bus.h"
#include "robot_comm_modbus/rtu_frame.h"

using namespace end_effector_hardware;
namespace rtu = robot_comm::modbus_rtu;

namespace
{
    std::vector<uint8_t> readReply(uint8_t slave, uint8_t fc, const std::vector<uint16_t>& regs)
    {
        std::vector<uint8_t> out{slave, fc, static_cast<uint8_t>(regs.size() * 2)};
        for (const uint16_t r : regs)
        {
            out.push_back(static_cast<uint8_t>(r >> 8));
            out.push_back(static_cast<uint8_t>(r & 0xFF));
        }
        return rtu::detail::withCrc(out);
    }

    /** A bus in the hub (an arm's flange name, so it can be provided) with a client for the probe. */
    struct Rig
    {
        std::shared_ptr<robot_comm::MockBus> bus;
        std::unique_ptr<robot_comm::BusHub::Provision> provision;
        std::shared_ptr<robot_comm::BusClient> client;

        explicit Rig(const std::string& name)
        {
            bus = std::make_shared<robot_comm::MockBus>(name);
            std::string error;
            provision = robot_comm::BusHub::instance().provide(name, bus, error);
            client = robot_comm::BusHub::instance().connect(name, error);
            EXPECT_TRUE(provision && client) << error;
        }
    };

    ProbeOptions quick(double cycle_hz = 0.0)
    {
        ProbeOptions options;
        options.cycle_hz = cycle_hz;
        options.warmup_s = 0.1;
        options.duration_s = 0.4;
        return options;
    }
}  // namespace

TEST(BusProbe, ReportsTheRoundTripOfTheRequestsTheDeviceSends)
{
    Rig rig("arm_rs485://10.3.0.1/left/port1");
    rig.bus->setResponder([](const std::vector<uint8_t>& request) -> std::optional<std::vector<uint8_t>> {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));  // the "device" takes 3 ms
        if (request[1] == rtu::kReadInputRegisters) return readReply(0x09, 0x04, {0x0009, 0x6400, 0x80FF});
        return std::nullopt;
    });
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(defaultJoints("jodell_rg75", "left"), defaultParams("jodell_rg75", "left"), error)) << error;

    const ProbeReport report = runProbe(gripper, rig.client, quick());

    EXPECT_TRUE(report.ready);
    EXPECT_FALSE(report.failed);
    EXPECT_GT(report.ok, 20u);
    EXPECT_EQ(report.timeouts, 0u);
    EXPECT_EQ(report.round_trip_ms.size(), report.ok);
    EXPECT_GE(report.p50_ms, 3.0);
    EXPECT_LT(report.p50_ms, 30.0);
    EXPECT_GE(report.max_ms, report.p99_ms);
    EXPECT_GE(report.p99_ms, report.p50_ms);
    EXPECT_GT(report.hz, 20.0);
    EXPECT_EQ(report.gaps, 0u);
}

TEST(BusProbe, AControllerRateLimitsTheRequestRate)
{
    Rig rig("arm_rs485://10.3.0.2/left/port1");
    rig.bus->setResponder([](const std::vector<uint8_t>& request) -> std::optional<std::vector<uint8_t>> {
        if (request[1] == rtu::kReadInputRegisters) return readReply(0x09, 0x04, {0x0009, 0x6400, 0x80FF});
        return std::nullopt;
    });
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(defaultJoints("jodell_rg75", "left"), defaultParams("jodell_rg75", "left"), error)) << error;

    const ProbeReport report = runProbe(gripper, rig.client, quick(100.0));

    EXPECT_TRUE(report.ready);
    EXPECT_GT(report.hz, 60.0);
    EXPECT_LT(report.hz, 130.0);  // paced at 100 Hz, not as fast as the mock answers
}

TEST(BusProbe, ADeviceThatNeverAnswersFailsAndTheTimeoutsAreCounted)
{
    Rig rig("arm_rs485://10.3.0.3/left/port1");
    rig.bus->setResponder([](const std::vector<uint8_t>&) { return std::nullopt; });
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(defaultJoints("linkerhand_l6", "left"), defaultParams("linkerhand_l6", "left"), error)) << error;

    ProbeOptions options = quick();
    options.warmup_s = 0.0;
    const ProbeReport report = runProbe(hand, rig.client, options);

    EXPECT_FALSE(report.ready);
    EXPECT_TRUE(report.failed);
    EXPECT_FALSE(report.failure.empty());
    EXPECT_GT(report.timeouts, 0u);
    EXPECT_EQ(report.ok, 0u);
    EXPECT_TRUE(report.round_trip_ms.empty());
}

TEST(BusProbe, StopEndsTheRunEarly)
{
    Rig rig("arm_rs485://10.3.0.4/left/port1");
    rig.bus->setResponder([](const std::vector<uint8_t>& request) -> std::optional<std::vector<uint8_t>> {
        if (request[1] == rtu::kReadInputRegisters) return readReply(0x09, 0x04, {0x0009, 0x6400, 0x80FF});
        return std::nullopt;
    });
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(defaultJoints("jodell_rg75", "left"), defaultParams("jodell_rg75", "left"), error)) << error;
    std::atomic<bool> stop{true};

    ProbeOptions options = quick();
    options.duration_s = 30.0;
    const auto begin = std::chrono::steady_clock::now();
    runProbe(gripper, rig.client, options, &stop);

    EXPECT_LT(std::chrono::steady_clock::now() - begin, std::chrono::seconds(2));
}

TEST(BusProbe, PercentileOfASortedList)
{
    const std::vector<double> sorted{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    EXPECT_DOUBLE_EQ(percentile(sorted, 0.5), 5.0);
    EXPECT_DOUBLE_EQ(percentile(sorted, 0.99), 10.0);
    EXPECT_DOUBLE_EQ(percentile(sorted, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(percentile({}, 0.5), 0.0);
}

TEST(BusProbe, EveryDeviceConfiguresFromTheDefaultJoints)
{
    for (const std::string device : {"jodell_rg75", "linkerhand_l6", "xhand1"})
    {
        for (const std::string side : {"left", "right"})
        {
            std::string error;
            auto made = makeBusDevice(device, rclcpp::get_logger("test"), error);
            ASSERT_TRUE(made) << error;
            EXPECT_TRUE(made->configure(defaultJoints(device, side), defaultParams(device, side), error))
                << device << " " << side << ": " << error;
        }
    }
    EXPECT_EQ(defaultBaud("linkerhand_l6"), 115200);
    EXPECT_EQ(defaultBaud("xhand1"), 3000000);
    EXPECT_EQ(defaultBaud("jodell_rg75"), 0);
}
