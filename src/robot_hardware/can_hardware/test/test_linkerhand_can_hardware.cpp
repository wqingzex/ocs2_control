// LinkerHandCanHardware on a MockCanBus: parameter checks, feedback before commands, the command frame the I/O
// thread sends, tool limits and bus errors, without a CAN interface.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "can_hardware/hands/linkerhand/linkerhand_can_hardware.h"
#include "robot_comm_can/mock_can_bus.h"

namespace lh = can_hardware::linkerhand;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using robot_comm::CanFrame;
using robot_comm::MockCanBus;
using namespace std::chrono_literals;

namespace
{
    constexpr uint32_t kLeftId = 0x28;

    CanFrame feedback(uint32_t id, uint8_t value)
    {
        CanFrame frame;
        frame.id = id;
        frame.len = 7;
        frame.data[0] = lh::kPositionCommand;
        for (int i = 1; i <= 6; ++i)
        {
            frame.data[i] = value;
        }
        return frame;
    }

    bool waitFor(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 1000ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (condition())
            {
                return true;
            }
            std::this_thread::sleep_for(2ms);
        }
        return condition();
    }

    std::vector<CanFrame> framesWithCommand(const MockCanBus& bus, uint8_t command, std::size_t len)
    {
        std::vector<CanFrame> out;
        for (const auto& frame : bus.sentFrames())
        {
            if (frame.len == len && frame.data[0] == command)
            {
                out.push_back(frame);
            }
        }
        return out;
    }
} // namespace

class LinkerHandCanHardwareTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        auto mock = std::make_unique<MockCanBus>();
        mock_ = mock.get();
        hw_ = std::make_unique<can_hardware::LinkerHandCanHardware>();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        hw_->setBusForTesting(std::move(mock));
    }

    void TearDown() override { hw_->on_deactivate(rclcpp_lifecycle::State()); }

    CallbackReturn init(const std::unordered_map<std::string, std::string>& overrides = {}, int joints = 6,
                        bool effort = false)
    {
        hardware_interface::HardwareInfo info;
        info.name = "linkerhand_test";
        info.type = "system";
        info.hardware_parameters = {{"hand_type", "linkerhand_l6"}, {"hand_side", "left"}, {"can_interface", "can7"}};
        for (const auto& item : overrides)
        {
            info.hardware_parameters[item.first] = item.second;
        }
        for (int i = 0; i < joints; ++i)
        {
            hardware_interface::ComponentInfo joint;
            joint.name = "left_hand_joint" + std::to_string(i + 1);
            joint.type = "joint";
            hardware_interface::InterfaceInfo command;
            command.name = hardware_interface::HW_IF_POSITION;
            joint.command_interfaces.push_back(command);
            hardware_interface::InterfaceInfo position;
            position.name = hardware_interface::HW_IF_POSITION;
            hardware_interface::InterfaceInfo velocity;
            velocity.name = hardware_interface::HW_IF_VELOCITY;
            joint.state_interfaces = {position, velocity};
            if (effort)
            {
                hardware_interface::InterfaceInfo eff;
                eff.name = hardware_interface::HW_IF_EFFORT;
                joint.state_interfaces.push_back(eff);
            }
            info.joints.push_back(joint);
        }
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>();
        params.executor = executor_;
        const CallbackReturn result = hw_->init(params);
        if (result == CallbackReturn::SUCCESS)
        {
            commands_ = hw_->on_export_command_interfaces();
            states_ = hw_->on_export_state_interfaces();
        }
        return result;
    }

    CallbackReturn activate() { return hw_->on_activate(rclcpp_lifecycle::State()); }
    return_type read() { return hw_->read(rclcpp::Time(0), rclcpp::Duration(20ms)); }
    return_type write() { return hw_->write(rclcpp::Time(0), rclcpp::Duration(20ms)); }
    double position(std::size_t joint) { return states_.at(joint * 2)->get_optional().value_or(NAN); }

    MockCanBus* mock_ = nullptr;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<can_hardware::LinkerHandCanHardware> hw_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
};

TEST_F(LinkerHandCanHardwareTest, InitAcceptsL6WithSixJoints)
{
    EXPECT_EQ(init(), CallbackReturn::SUCCESS);
    EXPECT_EQ(commands_.size(), 6u);
    EXPECT_EQ(states_.size(), 12u);
}

TEST_F(LinkerHandCanHardwareTest, InitRejectsBadParameters)
{
    EXPECT_EQ(init({{"hand_type", "xhand1"}}), CallbackReturn::ERROR);
    SetUp();
    EXPECT_EQ(init({}, 7), CallbackReturn::ERROR) << "L6 has 6 joints";
    SetUp();
    EXPECT_EQ(init({{"hand_side", "middle"}}), CallbackReturn::ERROR);
    SetUp();
    EXPECT_EQ(init({{"can_interface", "-bad"}}), CallbackReturn::ERROR);
    SetUp();
    EXPECT_EQ(init({{"can_data_bitrate", "5000000"}}), CallbackReturn::ERROR) << "classic CAN only";
}

TEST_F(LinkerHandCanHardwareTest, EffortStateIsExportedWhenDeclared)
{
    ASSERT_EQ(init({}, 6, true), CallbackReturn::SUCCESS);
    EXPECT_EQ(states_.size(), 18u);
}

TEST_F(LinkerHandCanHardwareTest, OpensTheConfiguredInterfaceWithoutTouchingItInTests)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    EXPECT_EQ(mock_->openedWith().interface, "can7");
    EXPECT_EQ(mock_->openedWith().bitrate, 1'000'000u);
    EXPECT_FALSE(mock_->openedWith().fd());
    EXPECT_FALSE(mock_->openedWith().manage_interface) << "an injected bus is never reconfigured";
}

TEST_F(LinkerHandCanHardwareTest, PollsPositionsAndSendsNoCommandBeforeFeedback)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    EXPECT_TRUE(waitFor([&] { return !framesWithCommand(*mock_, lh::kPositionCommand, 1).empty(); }));
    for (const auto& frame : mock_->sentFrames())
    {
        EXPECT_EQ(frame.id, kLeftId);
    }
    EXPECT_EQ(write(), return_type::OK);
    std::this_thread::sleep_for(50ms);
    EXPECT_TRUE(framesWithCommand(*mock_, lh::kPositionCommand, 7).empty()) << "no command before the first pose";
}

TEST_F(LinkerHandCanHardwareTest, FeedbackBecomesStateAndCommandsFollow)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    // Raw 0 = upper limit (closed); the initial state is 0 rad, so the state only changes through this frame.
    mock_->pushReceived(feedback(0x27, 0)); // the other hand: ignored
    mock_->pushReceived(feedback(kLeftId, 0));
    ASSERT_TRUE(waitFor([&]
    {
        read();
        return std::abs(position(1) - 1.39) < 1e-9;
    }));
    EXPECT_NEAR(position(0), 0.99, 1e-9);

    // The first command starts from the measured pose; open joint 2 fully.
    EXPECT_NEAR(commands_.at(0)->get_optional().value_or(NAN), 0.99, 1e-9) << "commands start at the measured pose";
    ASSERT_TRUE(commands_.at(1)->set_value(0.0));
    ASSERT_EQ(write(), return_type::OK);
    ASSERT_TRUE(waitFor([&] { return !framesWithCommand(*mock_, lh::kPositionCommand, 7).empty(); }));
    const auto command = framesWithCommand(*mock_, lh::kPositionCommand, 7).back();
    EXPECT_EQ(command.id, kLeftId);
    EXPECT_EQ(command.data[1], 0) << "joint 1 stays closed";
    EXPECT_EQ(command.data[2], 255) << "joint 2 open";
}

TEST_F(LinkerHandCanHardwareTest, ToolLimitsAreSentTwiceAfterActivation)
{
    ASSERT_EQ(init({{"left_tool_torque", "0.5"}}), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    mock_->pushReceived(feedback(kLeftId, 128));
    ASSERT_TRUE(waitFor([&]
    {
        return framesWithCommand(*mock_, lh::kMaxTorqueCommand, 7).size() >= 2 &&
            framesWithCommand(*mock_, lh::kMaxVelocityCommand, 7).size() >= 2;
    }));
    EXPECT_EQ(framesWithCommand(*mock_, lh::kMaxTorqueCommand, 7).front().data[1], 128);
    EXPECT_EQ(framesWithCommand(*mock_, lh::kMaxVelocityCommand, 7).front().data[1], 255);
}

TEST_F(LinkerHandCanHardwareTest, WithoutFeedbackTheStateMirrorsTheCommand)
{
    ASSERT_EQ(init({{"read_feedback", "false"}}), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    ASSERT_TRUE(commands_.at(0)->set_value(0.5));
    ASSERT_EQ(write(), return_type::OK);
    ASSERT_EQ(read(), return_type::OK);
    EXPECT_NEAR(position(0), 0.5, 1e-9);
    EXPECT_TRUE(framesWithCommand(*mock_, lh::kPositionCommand, 1).empty()) << "no position requests";
}

TEST_F(LinkerHandCanHardwareTest, OpenFailureFailsActivation)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    mock_->setOpenFails(true);
    EXPECT_EQ(activate(), CallbackReturn::ERROR);
}

TEST_F(LinkerHandCanHardwareTest, ReceiveErrorStopsIoAndReadReportsIt)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    mock_->setReceiveFails(true);
    mock_->pushReceived(feedback(kLeftId, 1)); // makes the bus readable
    EXPECT_TRUE(waitFor([&] { return read() == return_type::ERROR; }));
}

TEST_F(LinkerHandCanHardwareTest, SendFailuresDoNotStopTheDriver)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    mock_->setSendFails(true);
    std::this_thread::sleep_for(60ms);
    EXPECT_EQ(read(), return_type::OK);
    mock_->setSendFails(false);
    EXPECT_TRUE(waitFor([&] { return !framesWithCommand(*mock_, lh::kPositionCommand, 1).empty(); }))
        << "requests resume after the back-off";
}
