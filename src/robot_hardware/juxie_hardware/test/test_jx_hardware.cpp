// JxHardware on a MockCanBus: the activation sequence, the command ramp, feedback handling and fault handling,
// without a CAN interface. The protocol details are in test_juxie_protocol.cpp.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "juxie_hardware/jx_hardware.h"
#include "robot_comm_can/mock_can_bus.h"

namespace p = juxie_hardware::protocol;
using hardware_interface::CallbackReturn;
using robot_comm::CanFrame;
using robot_comm::MockCanBus;
using namespace std::chrono_literals;

namespace
{
    CanFrame feedback(uint8_t id, int16_t raw_pos, uint8_t status = 0x80, uint16_t error = 0, uint8_t len = 12)
    {
        CanFrame frame;
        frame.id = p::kFeedbackBase + id;
        frame.fd = true;
        frame.len = len;
        if (len == 12)
        {
            frame.data[0] = static_cast<uint8_t>(raw_pos >> 8);
            frame.data[1] = static_cast<uint8_t>(raw_pos & 0xFF);
            frame.data[6] = static_cast<uint8_t>(error & 0xFF);
            frame.data[7] = static_cast<uint8_t>(error >> 8);
            frame.data[10] = 0x03;
            frame.data[11] = status;
        }
        else
        {
            frame.data[0] = static_cast<uint8_t>(raw_pos >> 8);
            frame.data[1] = static_cast<uint8_t>(raw_pos & 0xFF);
            frame.data[6] = status;
        }
        return frame;
    }

    int16_t rawOf(const CanFrame& control, int motor_index)
    {
        return static_cast<int16_t>((control.data[motor_index * 7 + 1] << 8) | control.data[motor_index * 7 + 2]);
    }

    std::vector<CanFrame> controlFrames(const MockCanBus& bus)
    {
        std::vector<CanFrame> out;
        for (const auto& frame : bus.sentFrames())
        {
            if (frame.id == p::kMultiControlId)
            {
                out.push_back(frame);
            }
        }
        return out;
    }
} // namespace

class JxHardwareTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        auto mock = std::make_unique<MockCanBus>();
        mock_ = mock.get();
        hw_ = std::make_unique<juxie_hardware::JxHardware>();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        hw_->setBusForTesting(std::move(mock));
    }

    void TearDown() override
    {
        if (hw_) {
            hw_->on_deactivate(rclcpp_lifecycle::State());
        }
    }

    CallbackReturn init(const std::unordered_map<std::string, std::string>& overrides = {}, int joints = 2)
    {
        hardware_interface::HardwareInfo info;
        info.name = "juxie_test";
        info.type = "system";
        info.hardware_parameters = {{"motor_ids", "1,2"},       {"control_period_ms", "1"},
                                    {"max_position_step_nct", "25"}, {"max_position_accel_nct", "5"}};
        for (const auto& item : overrides)
        {
            info.hardware_parameters[item.first] = item.second;
        }
        for (int i = 0; i < joints; ++i)
        {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(i + 1);
            joint.type = "joint";
            info.joints.push_back(joint);
        }
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>();
        params.executor = executor_; // without an executor ros2_control creates no node for the component
        const CallbackReturn result = hw_->init(params);
        if (result == CallbackReturn::SUCCESS)
        {
            commands_ = hw_->on_export_command_interfaces();
            states_ = hw_->on_export_state_interfaces();
        }
        return result;
    }

    void queueFeedback(int16_t pos1, int16_t pos2)
    {
        mock_->pushReceived(feedback(1, pos1));
        mock_->pushReceived(feedback(2, pos2));
    }

    void queueUndervoltage()
    {
        CanFrame emergency;
        emergency.id = p::kEmergencyBase + 2;
        emergency.fd = true;
        emergency.len = 8;
        emergency.data[0] = 0x02; // 欠压
        mock_->pushReceived(emergency);
    }

    CallbackReturn activate() { return hw_->on_activate(rclcpp_lifecycle::State()); }

    double statePosition(size_t joint) { return states_.at(joint * 3)->get_optional().value_or(NAN); }

    MockCanBus* mock_ = nullptr;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<juxie_hardware::JxHardware> hw_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
};

TEST_F(JxHardwareTest, InitAcceptsTheNormalParameters)
{
    EXPECT_EQ(init(), CallbackReturn::SUCCESS);
    EXPECT_EQ(commands_.size(), 2u);
    EXPECT_EQ(states_.size(), 6u);
}

TEST_F(JxHardwareTest, InitDoesNotConfigureTheInterfaceWhenABusIsInjected)
{
    // A real SudoIpConfigurator would try `sudo ip link set ...` here, and fail in a test environment.
    EXPECT_EQ(init({{"can_manage_interface", "true"}}), CallbackReturn::SUCCESS);
}

TEST_F(JxHardwareTest, InitRejectsAnInterfaceNameThatIsNotPlain)
{
    EXPECT_EQ(init({{"can_interface", "can0;reboot"}}), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, InitRequiresCanFd)
{
    EXPECT_EQ(init({{"can_data_bitrate", "0"}}), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, InitRejectsMoreThanEightMotors)
{
    EXPECT_EQ(init({{"motor_ids", "1,2,3,4,5,6,7,8,9"}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({{"max_position_step_nct", "0"}}), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, ActivationOpensTheBusSendsSyncAndStartsFromTheFeedbackPosition)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(1000, -1000);

    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(50ms);

    EXPECT_TRUE(mock_->isOpen());
    EXPECT_EQ(mock_->openedWith().interface, "can0");
    EXPECT_EQ(mock_->openedWith().data_bitrate, 5'000'000u);
    const auto sent = mock_->sentFrames();
    ASSERT_FALSE(sent.empty());
    EXPECT_EQ(sent.front().id, p::kSyncFrameId) << "the sync frame comes first";

    const auto controls = controlFrames(*mock_);
    ASSERT_GE(controls.size(), 3u);
    EXPECT_EQ(controls.front().data[0], p::kCtrlWordCspClear) << "the first control frame clears errors";
    EXPECT_EQ(controls[1].data[0], p::kCtrlWordCsp);
    for (const auto& frame : controls)
    {
        EXPECT_NEAR(rawOf(frame, 0), 1000, 1) << "no jump at start-up";
        EXPECT_NEAR(rawOf(frame, 1), -1000, 1);
        EXPECT_EQ(frame.data[56], 1);
        EXPECT_EQ(frame.data[57], 2);
    }
}

TEST_F(JxHardwareTest, ActivationFailsWhenTheBusCannotBeOpened)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    mock_->setOpenFails(true);

    EXPECT_EQ(activate(), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, ActivationFailsWhenAMotorDoesNotAnswer)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    mock_->pushReceived(feedback(1, 0)); // motor 2 stays silent

    EXPECT_EQ(activate(), CallbackReturn::ERROR);
    EXPECT_FALSE(mock_->isOpen()) << "a failed activation closes the bus";
}

TEST_F(JxHardwareTest, ActivationFailsWhenTheMotorsAnswerWithDifferentLengths)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    mock_->pushReceived(feedback(1, 0, 0x20, 0, 7));
    mock_->pushReceived(feedback(2, 0, 0x80, 0, 12));

    EXPECT_EQ(activate(), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, ACommandIsRampedTowardsTheTargetWithinTheLimits)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    ASSERT_TRUE(commands_[0]->set_value(0.5)); // rad
    ASSERT_EQ(hw_->write(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
    const int16_t target = p::angleToRaw(0.5 * 180.0 / M_PI);

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    int16_t reached = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto controls = controlFrames(*mock_);
        if (!controls.empty())
        {
            reached = rawOf(controls.back(), 0);
        }
        if (reached == target)
        {
            break;
        }
        std::this_thread::sleep_for(5ms);
    }
    EXPECT_EQ(reached, target) << "the ramp has to arrive";

    int previous = 0;
    int previous_delta = 0;
    for (const auto& frame : controlFrames(*mock_))
    {
        const int value = rawOf(frame, 0);
        const int delta = value - previous;
        EXPECT_LE(std::abs(delta), 25);
        if (value != target)
        {
            // The last step may break the acceleration limit: not overshooting the target wins (documented rule).
            EXPECT_LE(std::abs(delta - previous_delta), 5);
        }
        EXPECT_LE(value, target) << "never past the target";
        previous = value;
        previous_delta = delta;
    }
    EXPECT_EQ(rawOf(controlFrames(*mock_).back(), 1), 0) << "the other joint was not commanded";
}

TEST_F(JxHardwareTest, ReadPublishesTheFeedbackToTheStateInterfaces)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    queueFeedback(p::angleToRaw(45.0), p::angleToRaw(-90.0));
    ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);

    EXPECT_NEAR(statePosition(0), M_PI / 4, 1e-3);
    EXPECT_NEAR(statePosition(1), -M_PI / 2, 1e-3);
}

TEST_F(JxHardwareTest, ReadKeepsTheLastValueWhenNothingArrived)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(p::angleToRaw(10.0), 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    const double before = statePosition(0);

    EXPECT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);

    EXPECT_DOUBLE_EQ(statePosition(0), before);
}

TEST_F(JxHardwareTest, ReadFailsWhenAMotorReportsAFault)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    mock_->pushReceived(feedback(2, 0, 0x80, 0x0004)); // over-temperature

    EXPECT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
}

TEST_F(JxHardwareTest, ErrorTransitionClosesTheBusAfterUndervoltage)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    queueUndervoltage();
    ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
    EXPECT_EQ(hw_->on_error(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_FALSE(mock_->isOpen());
}

TEST_F(JxHardwareTest, ShutdownClosesTheBusWithoutDeactivate)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    EXPECT_EQ(hw_->on_shutdown(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_FALSE(mock_->isOpen());
}

TEST_F(JxHardwareTest, DestructionJoinsTheThreadAfterUndervoltageWithoutALifecycleCallback)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    queueUndervoltage();
    ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
    hw_.reset();
}

TEST_F(JxHardwareTest, ReadIgnoresFramesOfOtherDevices)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    mock_->pushReceived(feedback(9, 0, 0xA0, 0x0004)); // a motor that is not ours, with a fault
    CanFrame error_frame;
    error_frame.id = p::kEmergencyBase + 1;
    error_frame.error = true; // a controller error frame, not an emergency of motor 1
    mock_->pushReceived(error_frame);

    EXPECT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

TEST_F(JxHardwareTest, DeactivateAfterTheControlThreadStoppedOnAnErrorDoesNotTerminate)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    mock_->pushReceived(feedback(1, 0, 0xA0)); // a fault stops the thread (running_ = false)
    ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
    std::this_thread::sleep_for(20ms);

    // The thread has to be joined even though running_ is already false: destroying a joinable std::thread
    // calls std::terminate, which is what the driver did before.
    EXPECT_EQ(hw_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_FALSE(mock_->isOpen());
}

TEST_F(JxHardwareTest, TheOlderSevenByteProtocolUsesItsOwnControlBytes)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    mock_->pushReceived(feedback(1, 0, 0x20, 0, 7));
    mock_->pushReceived(feedback(2, 0, 0x20, 0, 7));
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(30ms);

    const auto controls = controlFrames(*mock_);
    ASSERT_GE(controls.size(), 2u);
    EXPECT_EQ(controls.front().data[0], 0xD8);
    EXPECT_EQ(controls[1].data[0], 0xD0);
}

TEST_F(JxHardwareTest, InitRejectsANegativeStatsPeriod)
{
    EXPECT_EQ(init({{"stats_log_period_s", "-1"}}), CallbackReturn::ERROR);
}

TEST_F(JxHardwareTest, PeriodicStatsDoNotDisturbTheControlLoop)
{
    ASSERT_EQ(init({{"stats_log_period_s", "0.05"}}), CallbackReturn::SUCCESS);
    queueFeedback(0, 0);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);

    std::this_thread::sleep_for(300ms);

    EXPECT_GE(controlFrames(*mock_).size(), 200u);
}
