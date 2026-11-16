// EndEffectorHardware with a LinkerHand L6 on a MockBus provided as an arm's flange bus: parameter checks, waiting
// for the bus in on_activate, initialization in the control loop, runtime torque, failures.
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <functional>
#include <thread>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "end_effector_hardware/bus/end_effector_hardware.h"
#include "robot_comm/bus_hub.h"
#include "robot_comm/mock_bus.h"
#include "robot_comm_modbus/rtu_frame.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using namespace std::chrono_literals;
namespace rtu = robot_comm::modbus_rtu;

namespace
{
    uint16_t at(const std::vector<uint8_t>& f, std::size_t i) { return static_cast<uint16_t>((f[i] << 8) | f[i + 1]); }

    /** A left L6 behind the bus: answers reads with its registers, stores writes. */
    struct L6Sim
    {
        bool answer = true;
        std::array<uint16_t, 18> regs{100, 100, 100, 100, 100, 100};
        std::vector<uint16_t> write_counts;
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& r)
        {
            if (!answer || r[0] != 0x28) return std::nullopt;
            if (r[1] == rtu::kReadInputRegisters)
            {
                std::vector<uint8_t> out{0x28, 0x04, 12};
                for (int i = 0; i < 6; ++i)
                {
                    out.push_back(static_cast<uint8_t>(regs[i] >> 8));
                    out.push_back(static_cast<uint8_t>(regs[i] & 0xFF));
                }
                return rtu::detail::withCrc(out);
            }
            const uint16_t count = at(r, 4);
            write_counts.push_back(count);
            for (uint16_t i = 0; i < count; ++i) regs[i] = at(r, 7 + 2 * i);
            return rtu::detail::withCrc({0x28, 0x10, r[2], r[3], r[4], r[5]});
        }
    };

    int next_arm = 1;
}  // namespace

class EndEffectorHardwareTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        bus_name_ = "arm_rs485://10.2.0." + std::to_string(next_arm++) + "/left/port1";
        hw_ = std::make_unique<end_effector_hardware::EndEffectorHardware>();
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    void provide()
    {
        bus_ = std::make_shared<robot_comm::MockBus>(bus_name_);
        bus_->setResponder(std::ref(sim_));
        std::string error;
        provision_ = robot_comm::BusHub::instance().provide(bus_name_, bus_, error);
        EXPECT_TRUE(provision_) << error;
    }

    CallbackReturn init(std::unordered_map<std::string, std::string> overrides = {}, int joints = 6)
    {
        hardware_interface::HardwareInfo info;
        info.name = "left_end_effector";
        info.type = "system";
        info.hardware_parameters = {{"device", "linkerhand_l6"}, {"side", "left"}, {"bus", bus_name_}};
        for (auto& [k, v] : overrides)
        {
            if (v.empty()) info.hardware_parameters.erase(k);
            else info.hardware_parameters[k] = v;
        }
        const char* names[] = {"left_thumb_joint1", "left_thumb_joint2", "left_index_joint",
                               "left_middle_joint", "left_ring_joint",   "left_pinky_joint"};
        for (int i = 0; i < joints; ++i)
        {
            hardware_interface::ComponentInfo joint;
            joint.name = names[i];
            joint.type = "joint";
            hardware_interface::InterfaceInfo position;
            position.name = hardware_interface::HW_IF_POSITION;
            hardware_interface::InterfaceInfo velocity;
            velocity.name = hardware_interface::HW_IF_VELOCITY;
            joint.command_interfaces = {position};
            joint.state_interfaces = {position, velocity};
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

    CallbackReturn configure() { return hw_->on_configure(rclcpp_lifecycle::State()); }
    CallbackReturn activate() { return hw_->on_activate(rclcpp_lifecycle::State()); }
    return_type cycle(int n = 1)
    {
        return_type r = return_type::OK;
        for (int i = 0; i < n && r == return_type::OK; ++i)
        {
            r = hw_->read(rclcpp::Time(0), rclcpp::Duration(10ms));
            if (r == return_type::OK) hw_->write(rclcpp::Time(0), rclcpp::Duration(10ms));
        }
        return r;
    }

    std::string bus_name_;
    L6Sim sim_;
    std::shared_ptr<robot_comm::MockBus> bus_;
    std::unique_ptr<robot_comm::BusHub::Provision> provision_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<end_effector_hardware::EndEffectorHardware> hw_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
};

TEST_F(EndEffectorHardwareTest, RejectsBadParameters)
{
    const std::vector<std::pair<std::unordered_map<std::string, std::string>, int>> cases = {
        {{{"device", ""}}, 6},
        {{{"bus", ""}}, 6},
        {{{"bus", "arm_rs485://10.0.0.1/A/port1"}}, 6},
        {{{"io_mode", "blocking"}}, 6},
        {{{"device", "inspire_e2"}}, 6},
        {{}, 5},
        {{{"torque", "1.5"}}, 6},
    };
    for (const auto& [overrides, joints] : cases)
    {
        SetUp();
        EXPECT_EQ(init(overrides, joints), CallbackReturn::ERROR);
    }
}

TEST_F(EndEffectorHardwareTest, ExportsWhatTheUrdfDeclares)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    EXPECT_EQ(commands_.size(), 6u);
    EXPECT_EQ(states_.size(), 12u);  // position + velocity, no effort declared
}

TEST_F(EndEffectorHardwareTest, InitializesOnTheArmBusInTheControlLoop)
{
    provide();
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    EXPECT_TRUE(bus_->sent().empty());  // nothing sent in on_activate
    ASSERT_EQ(cycle(3), return_type::OK);
    ASSERT_FALSE(sim_.write_counts.empty());
    EXPECT_EQ(sim_.write_counts.front(), 18);  // initial write with torque / speed
    EXPECT_GT(states_.at(0)->get_optional().value_or(-1.0), 0.0);

    ASSERT_TRUE(commands_.at(0)->set_value(0.0));
    ASSERT_EQ(cycle(2), return_type::OK);
    EXPECT_EQ(sim_.regs[0], 255);  // thumb_joint1 at its lower limit
}

TEST_F(EndEffectorHardwareTest, RuntimeTorqueIsAppliedOnTheControlThread)
{
    provide();
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    ASSERT_EQ(cycle(3), return_type::OK);
    const auto writes = sim_.write_counts.size();
    ASSERT_TRUE(hw_->get_node()->set_parameter(rclcpp::Parameter("torque", 0.2)).successful);
    EXPECT_FALSE(hw_->get_node()->set_parameter(rclcpp::Parameter("torque", 3.0)).successful);
    ASSERT_EQ(cycle(2), return_type::OK);
    ASSERT_GT(sim_.write_counts.size(), writes);
    EXPECT_EQ(sim_.write_counts.back(), 18);
    EXPECT_EQ(sim_.regs[6], 51);  // 0.2 * 255
}

// ros2_control may activate this component before the arm (which connects in its own on_activate, same thread).
TEST_F(EndEffectorHardwareTest, ActivatesBeforeTheArmAndInitializesOnceItConnects)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(activate(), CallbackReturn::SUCCESS);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 50ms);  // does not wait for the arm
    ASSERT_EQ(cycle(3), return_type::OK);                       // nothing to talk to yet, not an error
    EXPECT_TRUE(sim_.write_counts.empty());

    provide();  // the arm has connected
    ASSERT_EQ(cycle(10), return_type::OK);
    EXPECT_FALSE(sim_.write_counts.empty());  // initialized: read, then written back
}

TEST_F(EndEffectorHardwareTest, ReadReportsAFailedInitialization)
{
    sim_.answer = false;
    provide();
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    EXPECT_EQ(cycle(10), return_type::ERROR);
}

TEST_F(EndEffectorHardwareTest, UsbPortThatCannotBeOpenedFailsConfigure)
{
    ASSERT_EQ(init({{"bus", "usb_rs485:///dev/ttyDOES_NOT_EXIST"}}), CallbackReturn::SUCCESS);
    EXPECT_EQ(configure(), CallbackReturn::ERROR);
}
