// End effector components on the arm's flange buses, as on the robot: the real bus devices
// of end_effector_hardware (Jodell RG75 on the left arm, LinkerHand L6 on the right) behind ArmBuses, whose fake SDK
// channel functions answer like the devices. The buses move only in the arm's read() / write() (pull / push), in the
// order ros2_control runs the components: arms read, end effector read, arms write, end effector write.
#include <gtest/gtest.h>

#include <array>
#include <deque>
#include <functional>
#include <optional>
#include <rclcpp/logger.hpp>
#include <vector>

#include "end_effector_hardware/bus/jodell_rg75.h"
#include "end_effector_hardware/bus/linkerhand_l6.h"
#include "marvin_hardware/comm/arm_bus.h"
#include "robot_comm/bus_hub.h"
#include "robot_comm_modbus/rtu_frame.h"

using end_effector_hardware::BusDevice;
using end_effector_hardware::JodellRg75;
using end_effector_hardware::LinkerHandL6;
using marvin_hardware::ArmBuses;
namespace rtu = robot_comm::modbus_rtu;

namespace
{
    using Responder = std::function<std::optional<std::vector<uint8_t>>(const std::vector<uint8_t>&)>;

    /** One arm's SDK send / receive buffer, with the device on its COM1 answering. */
    struct FakeArm
    {
        Responder device;
        std::deque<std::pair<std::vector<uint8_t>, long>> rx;
        std::vector<std::vector<uint8_t>> sent;
    };
    std::array<FakeArm, 2> g_arms;  // A = left, B = right

    template <int Arm>
    bool fakeSend(uint8_t* data, long size, long channel)
    {
        std::vector<uint8_t> frame(data, data + size);
        g_arms[Arm].sent.push_back(frame);
        if (channel != marvin_hardware::COM1_CHANNEL || !g_arms[Arm].device) return true;
        if (auto reply = g_arms[Arm].device(frame))
        {
            // The SDK hands the reply over in pieces.
            const auto half = reply->begin() + static_cast<long>(reply->size() / 2);
            g_arms[Arm].rx.push_back({{reply->begin(), half}, channel});
            g_arms[Arm].rx.push_back({{half, reply->end()}, channel});
        }
        return true;
    }

    template <int Arm>
    long fakeGet(unsigned char* buffer, long* channel)
    {
        auto& rx = g_arms[Arm].rx;
        if (rx.empty()) return 0;
        auto [bytes, ch] = rx.front();
        rx.pop_front();
        std::copy(bytes.begin(), bytes.end(), buffer);
        *channel = ch;
        return static_cast<long>(bytes.size());
    }

    uint16_t at(const std::vector<uint8_t>& f, std::size_t i) { return static_cast<uint16_t>((f[i] << 8) | f[i + 1]); }

    std::vector<uint8_t> readReply(uint8_t slave, const std::vector<uint16_t>& regs)
    {
        std::vector<uint8_t> out{slave, 0x04, static_cast<uint8_t>(regs.size() * 2)};
        for (const uint16_t r : regs)
        {
            out.push_back(static_cast<uint8_t>(r >> 8));
            out.push_back(static_cast<uint8_t>(r & 0xFF));
        }
        return rtu::detail::withCrc(out);
    }

    std::optional<std::vector<uint8_t>> jodell(const std::vector<uint8_t>& request)
    {
        if (request[1] == rtu::kReadInputRegisters && at(request, 2) == 0x07D0)
        {
            return readReply(0x09, {0x0009, static_cast<uint16_t>(155 << 8), 0x80FF});
        }
        return std::nullopt;  // activation / move: not acknowledged
    }

    struct L6Sim
    {
        std::array<uint16_t, 18> regs{200, 190, 180, 170, 160, 150};
        int position_writes = 0;
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& request)
        {
            if (request[0] != 0x27) return std::nullopt;  // the right hand
            if (request[1] == rtu::kReadInputRegisters) return readReply(0x27, {regs.begin(), regs.begin() + 6});
            if (request[1] == rtu::kWriteMultipleRegisters)
            {
                ++position_writes;
                for (uint16_t i = 0; i < at(request, 4); ++i) regs[i] = at(request, 7 + 2 * i);
                return rtu::detail::withCrc({0x27, 0x10, request[2], request[3], request[4], request[5]});
            }
            return std::nullopt;
        }
    };

    /** The arms system (ArmBuses of both arms) and two end effector components in one controller_manager loop. */
    struct Robot
    {
        ArmBuses left{"10.2.0.1", "left", &fakeSend<0>, &fakeGet<0>};
        ArmBuses right{"10.2.0.1", "right", &fakeSend<1>, &fakeGet<1>};
        std::vector<BusDevice*> devices;

        void cycle(int n = 1)
        {
            for (int i = 0; i < n; ++i)
            {
                left.pull();  // arms read()
                right.pull();
                for (auto* d : devices) d->collect();  // end effector read()
                left.push();  // arms write()
                right.push();
                for (auto* d : devices) d->step();  // end effector write()
            }
        }
    };

    std::shared_ptr<robot_comm::BusClient> connect(const std::string& name)
    {
        std::string error;
        auto client = robot_comm::BusHub::instance().connect(name, error);
        EXPECT_TRUE(client) << error;
        return client;
    }
}  // namespace

TEST(ArmBusDevices, GripperAndHandInitializeAndFollowCommandsOverTheArm)
{
    g_arms = {};
    g_arms[0].device = jodell;
    L6Sim hand_sim;
    g_arms[1].device = std::ref(hand_sim);

    // The end effector components come up first (on_init / on_activate), before the arm has connected.
    JodellRg75 gripper(rclcpp::get_logger("test"));
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure({"left_gripper_joint"}, {}, error)) << error;
    ASSERT_TRUE(hand.configure({"right_thumb_joint1", "right_thumb_joint2", "right_index_joint", "right_middle_joint",
                                "right_ring_joint", "right_pinky_joint"},
                               {{"side", "right"}}, error))
        << error;
    gripper.attach(connect("arm_rs485://10.2.0.1/left/port1"), robot_comm::IoMode::Async);
    hand.attach(connect("arm_rs485://10.2.0.1/right/port1"), robot_comm::IoMode::Async);
    gripper.start();
    hand.start();

    Robot robot;
    robot.devices = {&gripper, &hand};
    robot.cycle(3);  // the arm's buses are not there yet: nothing happens, nothing fails
    EXPECT_FALSE(gripper.ready() || hand.ready() || gripper.failed() || hand.failed());

    // The arm driver provides its buses and connects.
    ASSERT_TRUE(robot.left.provide(error)) << error;
    ASSERT_TRUE(robot.right.provide(error)) << error;
    robot.left.setConnected(true);
    robot.right.setConnected(true);
    robot.cycle(15);
    ASSERT_TRUE(gripper.ready()) << gripper.failure();
    ASSERT_TRUE(hand.ready()) << hand.failure();
    EXPECT_TRUE(gripper.online() && hand.online());
    EXPECT_GT(gripper.joints()[0].position, 0.0);
    EXPECT_EQ(g_arms[0].sent.front(), rtu::writeRegisters(0x09, 0x03E8, {0x0000}));  // RG75 activation first

    // A hand command reaches the hand through the right arm.
    const int writes_before = hand_sim.position_writes;
    hand.joints()[0].command = hand.joints()[0].position + 0.2;
    robot.cycle(6);
    EXPECT_GT(hand_sim.position_writes, writes_before);

    // The arm goes away: the devices keep their state, nothing is sent until it is back.
    robot.left.setConnected(false);
    const std::size_t sent = g_arms[0].sent.size();
    robot.cycle(3);
    EXPECT_EQ(g_arms[0].sent.size(), sent);
}
