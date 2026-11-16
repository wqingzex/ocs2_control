// The bus XHand1 (xhand1::realtimeFrame / parseReply) speaks byte for byte what the earlier XHand1Rs485Hardware sends,
// checked by running the earlier driver on a MockSerialPort that answers like a hand. Goes away with that driver.
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "end_effector_hardware/bus/xhand1.h"
#include "end_effector_hardware/hands/xhand1/xhand1_rs485_hardware.h"
#include "robot_comm_serial/mock_serial_port.h"

using namespace std::chrono_literals;
namespace xh = end_effector_hardware::xhand1;

namespace
{
    constexpr std::size_t kFrame = xh::kHeader + xh::kJoints * xh::kCommandBytesPerJoint + xh::kCrc;
    const char* kNames[] = {"thumb_joint1",  "thumb_joint2",  "thumb_joint3", "index_joint1",
                            "index_joint2",  "index_joint3",  "middle_joint1", "middle_joint2",
                            "ring_joint1",   "ring_joint2",   "pinky_joint1", "pinky_joint2"};

    std::vector<uint8_t> reply(const std::array<float, xh::kJoints>& position)
    {
        const std::size_t data = xh::kJoints * xh::kStateBytesPerJoint + xh::kTactileBytes;
        std::vector<uint8_t> out{0x55, 0xAA, 0x80, 0xFE, 0x02, static_cast<uint8_t>(data & 0xFF),
                                 static_cast<uint8_t>(data >> 8)};
        for (std::size_t id = 0; id < xh::kJoints; ++id)
        {
            std::vector<uint8_t> rec(24, 0);
            rec[0] = static_cast<uint8_t>(id);
            std::memcpy(rec.data() + 2, &position[id], 4);
            out.insert(out.end(), rec.begin(), rec.end());
        }
        out.resize(out.size() + xh::kTactileBytes, 0);
        const uint16_t c = xh::crc16(out.data(), out.size());
        out.push_back(static_cast<uint8_t>(c & 0xFF));
        out.push_back(static_cast<uint8_t>(c >> 8));
        return out;
    }
}  // namespace

TEST(XHand1Parity, ProbeAndPositionFramesMatchTheEarlierDriver)
{
    rclcpp::init(0, nullptr);
    const std::array<float, xh::kJoints> hand{0.5F, 0.1F, 0.2F, 0.0F, 0.3F, 0.3F, 0.4F, 0.4F, 0.5F, 0.5F, 0.6F, 0.6F};
    {
        auto port = std::make_unique<robot_comm::MockSerialPort>();
        auto* mock = port.get();
        mock->setWriteHook([mock, &hand](const std::vector<uint8_t>&) { mock->pushReceived(reply(hand)); });
        end_effector_hardware::XHand1Rs485Hardware old;
        old.set_serial_port_for_testing(std::move(port));

        hardware_interface::HardwareInfo info;
        info.name = "left_end_effector";
        info.type = "system";
        info.hardware_parameters = {{"serial_port", "/dev/null"}};
        for (const char* name : kNames)
        {
            hardware_interface::ComponentInfo joint;
            joint.name = std::string("left_hand_") + name;
            joint.type = "joint";
            for (const char* i : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
                                  hardware_interface::HW_IF_EFFORT})
            {
                hardware_interface::InterfaceInfo ii;
                ii.name = i;
                joint.command_interfaces.push_back(ii);
                joint.state_interfaces.push_back(ii);
            }
            info.joints.push_back(joint);
        }
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>();
        params.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        ASSERT_EQ(old.init(params), hardware_interface::CallbackReturn::SUCCESS);
        auto commands = old.on_export_command_interfaces();
        ASSERT_EQ(old.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

        // The probe: zero torque, mode 0, at the URDF initial positions (0, inside every limit).
        const auto written = mock->written();
        ASSERT_GE(written.size(), kFrame);
        const std::vector<uint8_t> probe(written.begin(), written.begin() + kFrame);
        EXPECT_EQ(probe, xh::realtimeFrame(0xFE, 0, {}, {}, {}, xh::kModeDisabled));

        // A position command after the hand answered: mode 3, torque limit 0.8 * 300, no speed limit.
        mock->clearWritten();
        for (auto& c : commands)
        {
            if (c->get_interface_name() == hardware_interface::HW_IF_POSITION) (void)c->set_value(1.0);
        }
        ASSERT_EQ(old.write(rclcpp::Time(0), rclcpp::Duration(10ms)), hardware_interface::return_type::OK);
        std::this_thread::sleep_for(50ms);
        ASSERT_EQ(old.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
        const auto sent = mock->written();
        ASSERT_GE(sent.size(), kFrame);
        std::array<double, xh::kJoints> target{};
        target.fill(1.0);
        target[3] = 0.174;  // index_joint1 clamped to its limit
        std::array<uint16_t, xh::kJoints> torque{};
        torque.fill(240);
        EXPECT_EQ(std::vector<uint8_t>(sent.end() - kFrame, sent.end()),
                  xh::realtimeFrame(0xFE, 0, {}, target, torque, xh::kModePosition));
    }

    // And the reply the earlier driver accepted parses the same.
    const auto state = xh::parseReply(reply(hand), 0xFE, 0);
    ASSERT_TRUE(state);
    for (std::size_t id = 0; id < xh::kJoints; ++id) EXPECT_FLOAT_EQ(state->position[id], hand[id]);
    rclcpp::shutdown();
}
