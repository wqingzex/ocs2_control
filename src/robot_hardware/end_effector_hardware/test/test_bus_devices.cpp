// The bus devices (JodellRg75, LinkerHandL6, XHand1) on a MockBus with a simulated device behind it: initialization order,
// no commands before the first state, which frames go out, unit conversion, failures.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <thread>

#include <rclcpp/logger.hpp>

#include "end_effector_hardware/bus/jodell_rg75.h"
#include "end_effector_hardware/bus/linkerhand_l6.h"
#include "end_effector_hardware/bus/params.h"
#include "end_effector_hardware/bus/xhand1.h"
#include "end_effector_hardware/gripper_protocol/utils/ModbusConfig.h"
#include "robot_comm/mock_bus.h"
#include "robot_comm_modbus/rtu_frame.h"

using namespace end_effector_hardware;
namespace rtu = robot_comm::modbus_rtu;

namespace
{
    uint16_t at(const std::vector<uint8_t>& f, std::size_t i) { return static_cast<uint16_t>((f[i] << 8) | f[i + 1]); }

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

    /** A bus in the hub (an arm's flange port, so it can be provided) with a client for the device. */
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
        static void cycle(BusDevice& device, int n = 1)
        {
            for (int i = 0; i < n; ++i)
            {
                device.collect();
                device.step();
            }
            device.collect();
        }
    };

    // ---- Jodell RG75 simulator ---------------------------------------------------------------------------------
    struct JodellSim
    {
        bool answer = true;
        int position_raw = 255 - 100;  ///< register value: 0 = open, 255 = closed
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& request)
        {
            if (!answer) return std::nullopt;
            if (request[1] == rtu::kReadInputRegisters && at(request, 2) == 0x07D0)
            {
                return readReply(0x09, 0x04, {0x0009, static_cast<uint16_t>(position_raw << 8), 0x80FF});
            }
            return std::nullopt;
        }
    };

    std::vector<std::string> gripperJoint() { return {"left_gripper_joint"}; }
}  // namespace

TEST(JodellRg75, ActivatesReadsTheFirstStateThenHoldsIt)
{
    Rig rig("arm_rs485://10.1.0.1/left/port1");
    JodellSim sim;
    rig.bus->setResponder(std::ref(sim));
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(gripperJoint(), {}, error)) << error;
    gripper.attach(rig.client, robot_comm::IoMode::Async);
    gripper.start();

    Rig::cycle(gripper, 2);  // activation write, then the first status read
    ASSERT_TRUE(gripper.ready());
    const auto sent = rig.bus->sent();
    ASSERT_GE(sent.size(), 2u);
    EXPECT_EQ(sent[0], rtu::writeRegisters(0x09, 0x03E8, {0x0000}));  // activation, not acknowledged
    EXPECT_EQ(sent[1][1], rtu::kReadInputRegisters);
    auto& joint = gripper.joints()[0];
    const double expected = (1.0 - (255 - 100) / 255.0) * gripper_protocol::ModbusConfig::Jodell::MAX_OPENING_DISTANCE;
    EXPECT_NEAR(joint.position, expected, 1e-9);
    EXPECT_DOUBLE_EQ(joint.command, joint.position);  // holds where it is
    EXPECT_NEAR(joint.effort, 0x80 / 255.0, 1e-9);
    EXPECT_NEAR(joint.velocity, 1.0, 1e-9);

    const std::size_t before = rig.bus->sent().size();
    Rig::cycle(gripper, 3);  // no new command: only status reads
    for (std::size_t i = before; i < rig.bus->sent().size(); ++i) EXPECT_EQ(rig.bus->sent()[i][1], 0x04);
}

TEST(JodellRg75, ActivatesAgainWhenItComesBackAfterGoingOffline)
{
    Rig rig("arm_rs485://10.1.0.30/left/port1");
    JodellSim sim;
    rig.bus->setResponder(std::ref(sim));
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(gripperJoint(), {{"offline_after_ms", "30"}}, error)) << error;
    gripper.attach(rig.client, robot_comm::IoMode::Async);
    gripper.start();
    Rig::cycle(gripper, 2);
    ASSERT_TRUE(gripper.ready());

    sim.answer = false;  // power cut
    Rig::cycle(gripper, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    Rig::cycle(gripper, 2);
    EXPECT_FALSE(gripper.online());
    EXPECT_FALSE(gripper.failed());  // it was there: keep trying

    sim.answer = true;  // powered again: it needs the activation write before it moves
    sim.position_raw = 255 - 30;
    const std::size_t before = rig.bus->sent().size();
    Rig::cycle(gripper, 4);
    const auto sent = rig.bus->sent();
    const auto activation = rtu::writeRegisters(0x09, 0x03E8, {0x0000});
    EXPECT_NE(std::find(sent.begin() + static_cast<long>(before), sent.end(), activation), sent.end());
    EXPECT_TRUE(gripper.online());
    auto& joint = gripper.joints()[0];
    EXPECT_DOUBLE_EQ(joint.command, joint.position);  // holds where it is now, not the command from before
}

TEST(JodellRg75, SendsAMoveWhenTheCommandChanges)
{
    Rig rig("arm_rs485://10.1.0.2/left/port1");
    JodellSim sim;
    rig.bus->setResponder(std::ref(sim));
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(gripperJoint(), {{"torque", "0.5"}, {"velocity", "0.25"}}, error)) << error;
    gripper.attach(rig.client, robot_comm::IoMode::Async);
    gripper.start();
    Rig::cycle(gripper, 2);
    ASSERT_TRUE(gripper.ready());

    gripper.joints()[0].command = 0.0;  // close
    Rig::cycle(gripper, 1);
    const auto move = rig.bus->sent().back();
    ASSERT_EQ(move[1], rtu::kWriteMultipleRegisters);
    EXPECT_EQ(at(move, 2), 0x03E8);
    EXPECT_EQ(at(move, 7), 0x0009);    // trigger
    EXPECT_EQ(at(move, 9), 255 << 8);  // position 255 = closed
    EXPECT_EQ(move[11], 127);          // torque 0.5 -> 127
    EXPECT_EQ(move[12], 64);           // speed 0.25 -> 64
    Rig::cycle(gripper, 1);
    EXPECT_EQ(rig.bus->sent().back()[1], 0x04);  // same command: not sent again
}

TEST(JodellRg75, FailsWhenTheFirstStatusNeverComes)
{
    Rig rig("arm_rs485://10.1.0.3/left/port1");
    JodellSim sim;
    sim.answer = false;
    rig.bus->setResponder(std::ref(sim));
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(gripper.configure(gripperJoint(), {}, error));
    gripper.attach(rig.client, robot_comm::IoMode::Async);
    gripper.start();
    Rig::cycle(gripper, 3);  // activation + 2 status reads without a reply
    EXPECT_FALSE(gripper.failed());
    Rig::cycle(gripper, 1);  // the third
    EXPECT_TRUE(gripper.failed());
    EXPECT_NE(gripper.failure().find("not responding"), std::string::npos);
    const std::size_t sent = rig.bus->sent().size();
    Rig::cycle(gripper, 2);
    EXPECT_EQ(rig.bus->sent().size(), sent);  // a failed device stays quiet
}

TEST(JodellRg75, RejectsBadConfiguration)
{
    JodellRg75 gripper(rclcpp::get_logger("test"));
    std::string error;
    EXPECT_FALSE(gripper.configure({"left_finger"}, {}, error));
    EXPECT_NE(error.find("exactly one joint"), std::string::npos);
    EXPECT_FALSE(gripper.configure(gripperJoint(), {{"torque", "2"}}, error));
    EXPECT_NE(error.find("torque must be a number in [0, 1]"), std::string::npos);
    EXPECT_FALSE(gripper.configure(gripperJoint(), {{"slave_id", "0x100"}}, error));
}

TEST(BusDevice, NothingGoesOutWhileTheArmIsNotConnected)
{
    std::string error;
    auto client = robot_comm::BusHub::instance().connect("arm_rs485://10.1.0.4/left/port1", error);
    ASSERT_TRUE(client) << error;
    JodellRg75 gripper(rclcpp::get_logger("test"));
    ASSERT_TRUE(gripper.configure(gripperJoint(), {}, error));
    gripper.attach(client, robot_comm::IoMode::Async);
    gripper.start();
    Rig::cycle(gripper, 3);
    EXPECT_FALSE(gripper.ready());
    EXPECT_FALSE(gripper.failed());
}

// ---- LinkerHand L6 ------------------------------------------------------------------------------------------

namespace
{
    struct L6Sim
    {
        uint8_t slave = 0x28;
        bool answer_reads = true;
        std::array<uint16_t, 18> regs{200, 190, 180, 170, 160, 150};
        std::vector<std::vector<uint8_t>> writes;
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& request)
        {
            if (request[0] != slave) return std::nullopt;
            if (request[1] == rtu::kReadInputRegisters)
            {
                if (!answer_reads) return std::nullopt;
                return readReply(slave, 0x04, std::vector<uint16_t>(regs.begin(), regs.begin() + 6));
            }
            if (request[1] == rtu::kWriteMultipleRegisters)
            {
                writes.push_back(request);
                const uint16_t count = at(request, 4);
                for (uint16_t i = 0; i < count; ++i) regs[i] = at(request, 7 + 2 * i);
                return rtu::detail::withCrc({slave, 0x10, request[2], request[3], request[4], request[5]});
            }
            return std::nullopt;
        }
    };

    // Deliberately not in register order: the mapping is by name.
    std::vector<std::string> handJoints()
    {
        return {"left_pinky_joint", "left_thumb_joint1", "left_index_joint", "left_thumb_joint2", "left_ring_joint",
                "left_middle_joint"};
    }
}  // namespace

TEST(LinkerHandL6, InitializesByReadingThenWritingBackWithDynamics)
{
    Rig rig("arm_rs485://10.1.1.1/left/port1");
    L6Sim sim;
    rig.bus->setResponder(std::ref(sim));
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(handJoints(), {{"side", "left"}, {"torque", "0.4"}}, error)) << error;
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();
    Rig::cycle(hand, 1);
    EXPECT_FALSE(hand.ready());  // read done, initial write still to go
    Rig::cycle(hand, 1);
    ASSERT_TRUE(hand.ready());

    ASSERT_EQ(sim.writes.size(), 1u);
    EXPECT_EQ(at(sim.writes[0], 4), 18);  // positions + torque + speed
    EXPECT_EQ(sim.regs[0], 200);          // written back unchanged: no motion
    EXPECT_EQ(sim.regs[5], 150);
    EXPECT_EQ(sim.regs[6], 102);          // torque 0.4 -> 102
    EXPECT_EQ(sim.regs[12], 255);         // speed 1.0
    EXPECT_NEAR(hand.joints()[0].position, hand.toRadians(0, 150), 1e-12);  // pinky -> register 5
    EXPECT_NEAR(hand.joints()[1].position, hand.toRadians(1, 200), 1e-12);  // thumb_joint1 -> register 0
}

TEST(LinkerHandL6, WritesPositionsOnChangeAndDynamicsWhenSet)
{
    Rig rig("arm_rs485://10.1.1.2/left/port1");
    L6Sim sim;
    rig.bus->setResponder(std::ref(sim));
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(handJoints(), {{"side", "left"}}, error));
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();
    Rig::cycle(hand, 2);
    ASSERT_TRUE(hand.ready());

    hand.joints()[1].command += 0.0005;  // below the 0.001 rad threshold
    Rig::cycle(hand, 1);
    EXPECT_EQ(sim.writes.size(), 1u);
    hand.joints()[1].command = 0.0;  // thumb_joint1 to its lower limit -> raw 255
    Rig::cycle(hand, 1);
    ASSERT_EQ(sim.writes.size(), 2u);
    EXPECT_EQ(at(sim.writes[1], 4), 6);  // positions only
    EXPECT_EQ(sim.regs[0], 255);

    hand.setDynamics(0.0, 0.5);
    Rig::cycle(hand, 1);
    ASSERT_EQ(sim.writes.size(), 3u);
    EXPECT_EQ(at(sim.writes[2], 4), 18);
    EXPECT_EQ(sim.regs[6], 0);
    EXPECT_EQ(sim.regs[12], 128);
}

TEST(LinkerHandL6, RightHandUsesItsSlaveAndFailsWithoutReplies)
{
    Rig rig("arm_rs485://10.1.1.3/right/port1");
    L6Sim sim;
    sim.slave = 0x27;
    sim.answer_reads = false;
    rig.bus->setResponder(std::ref(sim));
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    std::vector<std::string> joints;
    for (const auto& j : handJoints()) joints.push_back("right" + j.substr(4));
    ASSERT_TRUE(hand.configure(joints, {{"side", "right"}}, error)) << error;
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();
    Rig::cycle(hand, 6);
    EXPECT_TRUE(hand.failed());
    EXPECT_NE(hand.failure().find("slave 39"), std::string::npos) << hand.failure();
    EXPECT_EQ(rig.bus->sent().front()[0], 0x27);
}

TEST(LinkerHandL6, SyncModeWorksOnABusPumpedElsewhere)
{
    Rig rig("arm_rs485://10.1.1.4/left/port1");
    L6Sim sim;
    rig.bus->setResponder(std::ref(sim));
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(handJoints(), {{"side", "left"}}, error));
    hand.attach(rig.client, robot_comm::IoMode::Sync);
    hand.start();
    hand.step();
    hand.step();
    EXPECT_TRUE(hand.ready());
}

TEST(LinkerHandL6, RejectsBadConfiguration)
{
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    EXPECT_FALSE(hand.configure(handJoints(), {}, error));
    EXPECT_NE(error.find("side = left or right"), std::string::npos);
    auto five = handJoints();
    five.pop_back();
    EXPECT_FALSE(hand.configure(five, {{"side", "left"}}, error));
    EXPECT_NE(error.find("found 5"), std::string::npos);
    auto dup = handJoints();
    dup[0] = "left_thumb_joint1_copy";
    EXPECT_FALSE(hand.configure(dup, {{"side", "left"}}, error));
    EXPECT_NE(error.find("two joints map to thumb_joint1"), std::string::npos);
}

TEST(LinkerHandL6, RawConversionMatchesTheEarlierDriver)
{
    LinkerHandL6 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(handJoints(), {{"side", "left"}}, error));
    // joint 2 is the index finger, limits [0, 1.26]
    EXPECT_EQ(hand.toRaw(2, 0.0), 255);
    EXPECT_EQ(hand.toRaw(2, 1.26), 0);
    EXPECT_EQ(hand.toRaw(2, 9.0), 0);  // clamped
    EXPECT_NEAR(hand.toRadians(2, 255), 0.0, 1e-12);
    EXPECT_NEAR(hand.toRadians(2, 0), 1.26, 1e-12);
}

// ---- XHand1 -------------------------------------------------------------------------------------------------

namespace
{
    namespace xh = end_effector_hardware::xhand1;

    /** An XHand1 (hand id 0) behind the bus: follows position frames, answers with its state and tactile data. */
    struct XHand1Sim
    {
        bool answer = true;
        std::array<float, xh::kJoints> position{0.5F, 0.1F, 0.2F, 0.0F, 0.3F, 0.3F, 0.4F, 0.4F, 0.5F, 0.5F, 0.6F, 0.6F};
        std::vector<uint16_t> modes;    ///< the control mode of each frame
        std::vector<uint16_t> torques;  ///< the torque limit of joint 0 of each frame
        std::vector<float> targets;     ///< joint 0's target of each frame
        // Per joint record: id, kp, ki, kd (u16 each), target float @8, torque limit @12, mode @14, 4 reserved.

        static float f32(const std::vector<uint8_t>& f, std::size_t at)
        {
            float v;
            std::memcpy(&v, f.data() + at, 4);
            return v;
        }
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& r)
        {
            if (r.size() != xh::kHeader + xh::kJoints * xh::kCommandBytesPerJoint + xh::kCrc || r[3] != 0x80) return {};
            const uint16_t crc = static_cast<uint16_t>(r[r.size() - 2] | (r[r.size() - 1] << 8));
            if (crc != xh::crc16(r.data(), r.size() - 2)) return {};
            modes.push_back(static_cast<uint16_t>(r[7 + 14] | (r[7 + 15] << 8)));
            torques.push_back(static_cast<uint16_t>(r[7 + 12] | (r[7 + 13] << 8)));
            targets.push_back(f32(r, 7 + 8));
            if (modes.back() == xh::kModePosition)
            {
                for (std::size_t j = 0; j < xh::kJoints; ++j) position[j] = f32(r, 7 + j * 24 + 8);
            }
            if (!answer) return {};
            const std::size_t data = xh::kJoints * xh::kStateBytesPerJoint + xh::kTactileBytes;
            std::vector<uint8_t> out{0x55, 0xAA, 0x80, 0xFE, 0x02, static_cast<uint8_t>(data & 0xFF),
                                     static_cast<uint8_t>(data >> 8)};
            // Records not in id order: the parser goes by the id in each record.
            for (std::size_t k = 0; k < xh::kJoints; ++k)
            {
                const std::size_t id = xh::kJoints - 1 - k;
                std::vector<uint8_t> rec(24, 0);
                rec[0] = static_cast<uint8_t>(id);
                std::memcpy(rec.data() + 2, &position[id], 4);
                rec[6] = static_cast<uint8_t>(10 + id);
                out.insert(out.end(), rec.begin(), rec.end());
            }
            out.resize(out.size() + xh::kTactileBytes, 0x11);
            const uint16_t c = xh::crc16(out.data(), out.size());
            out.push_back(static_cast<uint8_t>(c & 0xFF));
            out.push_back(static_cast<uint8_t>(c >> 8));
            return out;
        }
    };

    std::vector<std::string> xhandJoints()
    {
        return {"left_hand_thumb_joint1",  "left_hand_thumb_joint2",  "left_hand_thumb_joint3", "left_hand_index_joint1",
                "left_hand_index_joint2",  "left_hand_index_joint3",  "left_hand_middle_joint1",
                "left_hand_middle_joint2", "left_hand_ring_joint1",   "left_hand_ring_joint2",
                "left_hand_pinky_joint1",  "left_hand_pinky_joint2"};
    }
}  // namespace

TEST(XHand1Frame, CrcIsCcittWithZeroInit)
{
    const std::string check = "123456789";
    EXPECT_EQ(xh::crc16(reinterpret_cast<const uint8_t*>(check.data()), check.size()), 0x31C3);
}

TEST(XHand1Frame, ReplyLengthFromTheHeaderAndGarbageEndsAtOnce)
{
    EXPECT_EQ(xh::replyLength({0x55, 0xAA, 0x80}), 0u);
    EXPECT_EQ(xh::replyLength({0x55, 0xAA, 0x80, 0xFE, 0x02, 0x20, 0x01}), 7u + 0x120 + 2);
    EXPECT_EQ(xh::replyLength({0x12, 0x34}), 2u);
    EXPECT_EQ(xh::replyLength({0x55, 0xAA, 0x80, 0xFE, 0x02, 0xFF, 0xFF}), 7u);  // longer than any reply
}

TEST(XHand1Frame, EffortIsASignedTorqueReading)
{
    // The hand reports the motor torque as a signed 16-bit value: on the robot an index finger bending gave raw
    // 0xFFF8 .. 0xFFE0 (a few units against the motion) and 185 .. 235 on the way back, never a real 65528.
    const std::size_t data = xh::kJoints * xh::kStateBytesPerJoint + xh::kTactileBytes;
    std::vector<uint8_t> frame{0x55, 0xAA, 0x80, 0xFE, 0x02, static_cast<uint8_t>(data & 0xFF),
                               static_cast<uint8_t>(data >> 8)};
    for (std::size_t id = 0; id < xh::kJoints; ++id)
    {
        std::vector<uint8_t> record(24, 0);
        record[0] = static_cast<uint8_t>(id);
        if (id == 4)  // 0xFFF8 = -8
        {
            record[6] = 0xF8;
            record[7] = 0xFF;
        }
        if (id == 5)  // 185
        {
            record[6] = 0xB9;
            record[7] = 0x00;
        }
        frame.insert(frame.end(), record.begin(), record.end());
    }
    frame.resize(frame.size() + xh::kTactileBytes, 0);
    const uint16_t crc = xh::crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc >> 8));

    const auto state = xh::parseReply(frame, 0xFE, 0);

    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->effort[4], -8.0);
    EXPECT_DOUBLE_EQ(state->effort[5], 185.0);
    EXPECT_DOUBLE_EQ(state->effort[0], 0.0);
}

TEST(XHand1, ProbesWithZeroTorqueThenHoldsAndFollows)
{
    Rig rig("arm_rs485://10.1.0.20/left/port1");
    XHand1Sim sim;
    rig.bus->setResponder(std::ref(sim));
    XHand1 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(xhandJoints(), {}, error)) << error;
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();

    Rig::cycle(hand, 1);
    ASSERT_TRUE(hand.ready());
    EXPECT_EQ(sim.modes.front(), xh::kModeDisabled);
    EXPECT_EQ(sim.torques.front(), 0);
    EXPECT_NEAR(hand.joints()[0].position, 0.5, 1e-6);
    EXPECT_NEAR(hand.joints()[11].position, 0.6, 1e-6);
    EXPECT_DOUBLE_EQ(hand.joints()[0].effort, 10.0);
    EXPECT_DOUBLE_EQ(hand.joints()[0].command, hand.joints()[0].position);  // holds where it is

    Rig::cycle(hand, 2);  // running: position mode, torque 0.8 * 300, target = the held position
    EXPECT_EQ(sim.modes.back(), xh::kModePosition);
    EXPECT_EQ(sim.torques.back(), 240);
    EXPECT_NEAR(sim.targets.back(), 0.5, 1e-6);

    hand.joints()[0].command = 1.2;
    Rig::cycle(hand, 2);
    EXPECT_NEAR(sim.targets.back(), 1.2, 1e-6);
    EXPECT_NEAR(hand.joints()[0].position, 1.2, 1e-6);

    hand.joints()[0].command = 9.0;  // beyond the joint: clamped
    Rig::cycle(hand, 2);
    EXPECT_NEAR(sim.targets.back(), 1.832, 1e-5);
}

TEST(XHand1, VelocityLimitsTheStepFromTheMeasuredPosition)
{
    Rig rig("arm_rs485://10.1.0.21/left/port1");
    XHand1Sim sim;
    rig.bus->setResponder(std::ref(sim));
    XHand1 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(xhandJoints(), {{"velocity", "0.001"}}, error)) << error;  // 0.03 rad/s
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();
    Rig::cycle(hand, 2);
    ASSERT_TRUE(hand.ready());
    hand.joints()[0].command = 1.5;
    Rig::cycle(hand, 1);
    EXPECT_LT(sim.targets.back(), 0.51);  // a cycle of a few ms moves it by far less than 1 rad
    EXPECT_GE(sim.targets.back(), 0.5);
}

TEST(XHand1, FailsWhenTheHandNeverAnswersAndReprobesAfterGoingOffline)
{
    {
        Rig rig("arm_rs485://10.1.0.22/left/port1");
        XHand1Sim sim;
        sim.answer = false;
        rig.bus->setResponder(std::ref(sim));
        XHand1 hand(rclcpp::get_logger("test"));
        std::string error;
        ASSERT_TRUE(hand.configure(xhandJoints(), {}, error));
        hand.attach(rig.client, robot_comm::IoMode::Async);
        hand.start();
        Rig::cycle(hand, 25);
        EXPECT_TRUE(hand.failed());
        EXPECT_EQ(sim.modes.size(), 20u);
        for (const auto mode : sim.modes) EXPECT_EQ(mode, xh::kModeDisabled);  // never torque before it answered
    }
    Rig rig("arm_rs485://10.1.0.23/left/port1");
    XHand1Sim sim;
    rig.bus->setResponder(std::ref(sim));
    XHand1 hand(rclcpp::get_logger("test"));
    std::string error;
    ASSERT_TRUE(hand.configure(xhandJoints(), {}, error));
    hand.attach(rig.client, robot_comm::IoMode::Async);
    hand.start();
    Rig::cycle(hand, 2);
    ASSERT_TRUE(hand.ready());
    hand.joints()[0].command = 1.0;
    sim.answer = false;
    Rig::cycle(hand, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(210));
    Rig::cycle(hand, 3);
    EXPECT_FALSE(hand.online());
    EXPECT_FALSE(hand.failed());  // it was there: keep trying
    EXPECT_EQ(sim.modes.back(), xh::kModeDisabled);
    sim.position[0] = 0.7F;  // moved by hand meanwhile
    sim.answer = true;
    Rig::cycle(hand, 3);
    EXPECT_TRUE(hand.online());
    EXPECT_EQ(sim.modes.back(), xh::kModePosition);
    EXPECT_NEAR(sim.targets.back(), 0.7, 1e-6);  // holds where it is now, not the stale command
}

TEST(XHand1, RejectsBadConfiguration)
{
    XHand1 hand(rclcpp::get_logger("test"));
    std::string error;
    auto joints = xhandJoints();
    joints.pop_back();
    EXPECT_FALSE(hand.configure(joints, {}, error));
    EXPECT_FALSE(hand.configure(xhandJoints(), {{"hand_id", "200"}}, error));
    EXPECT_FALSE(hand.configure(xhandJoints(), {{"torque_limit", "400"}}, error));
    EXPECT_FALSE(hand.configure(xhandJoints(), {{"kp", "40000"}}, error));
    EXPECT_TRUE(hand.configure(xhandJoints(), {{"hand_id", "1"}, {"kp", "-5"}}, error)) << error;
}

// ---- parameters ---------------------------------------------------------------------------------------------

TEST(Params, TheWholeTextMustBeTheNumber)
{
    namespace params = end_effector_hardware::params;
    EXPECT_EQ(params::toDouble("0.5"), 0.5);
    EXPECT_FALSE(params::toDouble("0.5x"));  // std::stod would have taken 0.5
    EXPECT_FALSE(params::toDouble(""));
    EXPECT_FALSE(params::toDouble("nan"));
    EXPECT_EQ(params::toLong("0xFE"), 254);
    EXPECT_EQ(params::toLong("-5"), -5);
    EXPECT_FALSE(params::toLong("12abc"));
    EXPECT_FALSE(params::toLong("0x"));
    EXPECT_FALSE(params::toLong("1.5"));

    double ratio = 0.3;
    std::string error;
    EXPECT_TRUE(params::ratio({}, "torque", ratio, error));  // absent: kept
    EXPECT_DOUBLE_EQ(ratio, 0.3);
    EXPECT_FALSE(params::ratio({{"torque", "1.5"}}, "torque", ratio, error));
    EXPECT_NE(error.find("torque"), std::string::npos);  // names the parameter
    long id = 0;
    EXPECT_FALSE(params::integer({{"hand_id", "300"}}, "hand_id", 0, 127, id, error));
    EXPECT_TRUE(params::integer({{"host_id", "0xFE"}}, "host_id", 0, 255, id, error));
    EXPECT_EQ(id, 254);
}
