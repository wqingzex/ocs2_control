// JxHardware and a MotorSim in a closed loop over a MockCanBus: the driver's real activation, control thread,
// rate limiter and feedback handling working against a model of the motors. No CAN interface needed.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "juxie_hardware/jx_hardware.h"
#include "juxie_hardware/juxie_motor_sim.h"
#include "robot_comm_can/mock_can_bus.h"

using hardware_interface::CallbackReturn;
using juxie_hardware::MotorSim;
using robot_comm::MockCanBus;
using namespace std::chrono_literals;
namespace p = juxie_hardware::protocol;

class ClosedLoopTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override { build(12); }

    void TearDown() override { hw_->on_deactivate(rclcpp_lifecycle::State()); }

    void build(uint8_t feedback_len)
    {
        MotorSim::Options options;
        options.ids = {1, 2};
        options.feedback_len = feedback_len;
        sim_ = std::make_unique<MotorSim>(options);

        auto mock = std::make_unique<MockCanBus>();
        mock_ = mock.get();
        mock_->setSendHook(
            [this](const robot_comm::CanFrame& frame)
            {
                std::lock_guard<std::mutex> lock(sim_mutex_);
                for (const auto& answer : sim_->onFrame(frame))
                {
                    mock_->pushReceived(answer);
                }
            });

        hw_ = std::make_unique<juxie_hardware::JxHardware>();
        hw_->setBusForTesting(std::move(mock));
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        hardware_interface::HardwareInfo info;
        info.name = "juxie_closed_loop";
        info.type = "system";
        info.hardware_parameters = {{"motor_ids", "1,2"},           {"control_period_ms", "1"},
                                    {"max_position_step_nct", "90"}, {"max_position_accel_nct", "10"}};
        for (int i = 0; i < 2; ++i)
        {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(i + 1);
            joint.type = "joint";
            info.joints.push_back(joint);
        }
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>();
        params.executor = executor_;
        ASSERT_EQ(hw_->init(params), CallbackReturn::SUCCESS);
        commands_ = hw_->on_export_command_interfaces();
        states_ = hw_->on_export_state_interfaces();
    }

    double position(size_t joint) { return states_.at(joint * 3)->get_optional().value_or(NAN); }

    hardware_interface::return_type cycle()
    {
        return hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0));
    }

    /// Run read()/write() at about 500 Hz until `done()` or the timeout; returns how long it took, or -1.
    template <typename Done>
    double runUntil(Done done, std::chrono::milliseconds timeout, bool& read_failed)
    {
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout)
        {
            if (cycle() != hardware_interface::return_type::OK)
            {
                read_failed = true;
                return -1.0;
            }
            hw_->write(rclcpp::Time(), rclcpp::Duration(0, 0));
            if (done())
            {
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            }
            std::this_thread::sleep_for(2ms);
        }
        return -1.0;
    }

    std::unique_ptr<MotorSim> sim_;
    std::mutex sim_mutex_;
    MockCanBus* mock_ = nullptr;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<juxie_hardware::JxHardware> hw_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
};

TEST_F(ClosedLoopTest, ActivationAgainstTheMotorsSucceedsAndTheMotorsAreCommanded)
{
    ASSERT_EQ(hw_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(50ms);

    EXPECT_EQ(sim_->syncFrames(), 1u);
    EXPECT_GE(sim_->controlFrames(), 10u) << "the control thread sends about one frame per millisecond";
}

TEST_F(ClosedLoopTest, TheJointsFollowACommandAndSettleOnIt)
{
    ASSERT_EQ(hw_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_TRUE(commands_[0]->set_value(0.5));
    ASSERT_TRUE(commands_[1]->set_value(-0.3));

    bool read_failed = false;
    const double seconds = runUntil(
        [&] { return std::abs(position(0) - 0.5) < 2e-3 && std::abs(position(1) + 0.3) < 2e-3; }, 3000ms, read_failed);

    EXPECT_FALSE(read_failed);
    EXPECT_GT(seconds, 0.0) << "never arrived";
    EXPECT_LT(seconds, 1.5) << "0.5 rad at 90 counts per millisecond must not take that long";
    EXPECT_EQ(sim_->rawPosition(1), p::angleToRaw(0.5 * 180.0 / M_PI));
}

TEST_F(ClosedLoopTest, ARepeatedSweepRunsWithoutAReadError)
{
    ASSERT_EQ(hw_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    bool read_failed = false;
    double worst_lag = 0.0;

    for (int sweep = 0; sweep < 6 && !read_failed; ++sweep)
    {
        const double target = (sweep % 2 == 0) ? 0.4 : -0.4;
        ASSERT_TRUE(commands_[0]->set_value(target));
        const double seconds =
            runUntil([&] { return std::abs(position(0) - target) < 2e-3; }, 2000ms, read_failed);
        ASSERT_FALSE(read_failed) << "sweep " << sweep;
        ASSERT_GT(seconds, 0.0) << "sweep " << sweep << " never arrived";
        worst_lag = std::max(worst_lag, seconds);
    }

    EXPECT_LT(worst_lag, 1.5);
    EXPECT_GE(sim_->controlFrames(), 500u);
}

TEST_F(ClosedLoopTest, AMotorFaultIsReportedAsAReadError)
{
    ASSERT_EQ(hw_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
    // The first control frame carries the "clear error" bit and wipes a fault injected before it.
    std::this_thread::sleep_for(30ms);

    {
        std::lock_guard<std::mutex> lock(sim_mutex_);
        sim_->injectFault(2, 0x0004);
    }

    bool read_failed = false;
    runUntil([] { return false; }, 500ms, read_failed);
    EXPECT_TRUE(read_failed);
}

TEST_F(ClosedLoopTest, TheOlderSevenByteProtocolWorksEndToEnd)
{
    hw_->on_deactivate(rclcpp_lifecycle::State());
    build(7);
    ASSERT_EQ(hw_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_TRUE(commands_[0]->set_value(0.25));

    bool read_failed = false;
    const double seconds = runUntil([&] { return std::abs(position(0) - 0.25) < 2e-3; }, 3000ms, read_failed);

    EXPECT_FALSE(read_failed);
    EXPECT_GT(seconds, 0.0);
}
