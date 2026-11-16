// Kwr75Sensor on a MockBus provided as an arm's flange bus (port2): frame parsing, units, one request per read(),
// nothing before the arm connects, going to 0 when frames stop, parameter checks.
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "end_effector_hardware/bus/kwr75_sensor.h"
#include "robot_comm/bus_hub.h"
#include "robot_comm/mock_bus.h"

using end_effector_hardware::Kwr75Sensor;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using namespace std::chrono_literals;
namespace kwr75 = end_effector_hardware::kwr75;

namespace
{
    std::vector<uint8_t> frame(const std::array<float, 6>& values, uint8_t command = 0x48)
    {
        std::vector<uint8_t> out{command, 0xAA};
        for (const float v : values)
        {
            uint8_t bytes[4];
            std::memcpy(bytes, &v, 4);
            out.insert(out.end(), bytes, bytes + 4);
        }
        out.push_back(0x0D);
        out.push_back(0x0A);
        return out;
    }

    /** A KWR75 behind the bus: answers each poll with its current values. */
    struct Kwr75Sim
    {
        bool answer = true;
        std::array<float, 6> values{1.0F, -2.0F, 3.5F, 0.1F, -0.2F, 0.3F};
        int polls = 0;
        std::optional<std::vector<uint8_t>> operator()(const std::vector<uint8_t>& request)
        {
            if (request != kwr75::pollRequest()) return std::nullopt;
            ++polls;
            if (!answer) return std::nullopt;
            return frame(values);
        }
    };

    int next_arm = 1;
}  // namespace

TEST(Kwr75Frame, ParsesLowByteFirstFloatsAndRejectsBadFrames)
{
    std::array<double, 6> out{};
    ASSERT_TRUE(kwr75::parseFrame(frame({1.5F, 0, 0, 0, 0, -0.25F}), out));
    EXPECT_DOUBLE_EQ(out[0], 1.5);
    EXPECT_DOUBLE_EQ(out[5], -0.25);
    EXPECT_TRUE(kwr75::parseFrame(frame({0, 0, 0, 0, 0, 0}, 0x49), out));  // stream mode frames the same

    auto bad = frame({0, 0, 0, 0, 0, 0});
    bad[1] = 0x00;
    EXPECT_FALSE(kwr75::parseFrame(bad, out));
    bad = frame({0, 0, 0, 0, 0, 0});
    bad.pop_back();
    EXPECT_FALSE(kwr75::parseFrame(bad, out));
    EXPECT_FALSE(kwr75::parseFrame(frame({5000.0F, 0, 0, 0, 0, 0}), out));  // beyond the sensor: corrupted
    EXPECT_EQ(kwr75::pollRequest(), (std::vector<uint8_t>{0x48, 0xAA, 0x0D, 0x0A}));
}

class Kwr75SensorTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        bus_name_ = "arm_rs485://10.3.0." + std::to_string(next_arm++) + "/left/port2";
        sensor_ = std::make_unique<Kwr75Sensor>();
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

    CallbackReturn init(std::unordered_map<std::string, std::string> overrides = {},
                        std::vector<std::string> interfaces = {"force.x", "force.y", "force.z", "torque.x", "torque.y",
                                                               "torque.z"})
    {
        hardware_interface::HardwareInfo info;
        info.name = "left_ft";
        info.type = "sensor";
        info.hardware_parameters = {{"bus", bus_name_}, {"offline_after_ms", "30"}};
        for (auto& [k, v] : overrides)
        {
            if (v.empty()) info.hardware_parameters.erase(k);
            else info.hardware_parameters[k] = v;
        }
        hardware_interface::ComponentInfo sensor;
        sensor.name = "left_ft_sensor";
        sensor.type = "sensor";
        for (const auto& name : interfaces)
        {
            hardware_interface::InterfaceInfo i;
            i.name = name;
            sensor.state_interfaces.push_back(i);
        }
        info.sensors.push_back(sensor);
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>();
        params.executor = executor_;
        const CallbackReturn result = sensor_->init(params);
        if (result == CallbackReturn::SUCCESS) states_ = sensor_->on_export_state_interfaces();
        return result;
    }

    CallbackReturn start()
    {
        const auto r = sensor_->on_configure(rclcpp_lifecycle::State());
        return r == CallbackReturn::SUCCESS ? sensor_->on_activate(rclcpp_lifecycle::State()) : r;
    }

    void cycle(int n = 1)
    {
        for (int i = 0; i < n; ++i) ASSERT_EQ(sensor_->read(rclcpp::Time(0), rclcpp::Duration(2ms)), return_type::OK);
    }

    double state(std::size_t i) const { return states_.at(i)->get_optional().value(); }

    std::string bus_name_;
    std::unique_ptr<Kwr75Sensor> sensor_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<robot_comm::MockBus> bus_;
    std::unique_ptr<robot_comm::BusHub::Provision> provision_;
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
    Kwr75Sim sim_;
};

TEST_F(Kwr75SensorTest, ReadsTheWrenchInNewtonsOneRequestPerRead)
{
    provide();
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(start(), CallbackReturn::SUCCESS);
    cycle(1);  // the first request goes out
    cycle(1);  // its reply is taken
    EXPECT_TRUE(sensor_->online());
    EXPECT_NEAR(state(0), 1.0 * 9.80665, 1e-5);
    EXPECT_NEAR(state(1), -2.0 * 9.80665, 1e-5);
    EXPECT_NEAR(state(5), 0.3 * 9.80665, 1e-5);
    cycle(5);
    EXPECT_EQ(sim_.polls, 7);  // never more than one in flight
}

TEST_F(Kwr75SensorTest, RawUnitsWhenNotConverted)
{
    provide();
    ASSERT_EQ(init({{"convert_to_si", "false"}}), CallbackReturn::SUCCESS);
    ASSERT_EQ(start(), CallbackReturn::SUCCESS);
    cycle(2);
    EXPECT_NEAR(state(2), 3.5, 1e-6);
}

TEST_F(Kwr75SensorTest, ActivatesBeforeTheArmAndStartsOnceItConnects)
{
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(start(), CallbackReturn::SUCCESS);  // the arm (and its bus) is not there yet
    cycle(3);
    EXPECT_FALSE(sensor_->online());
    EXPECT_EQ(state(0), 0.0);

    provide();
    cycle(3);
    EXPECT_TRUE(sensor_->online());
    EXPECT_NEAR(state(0), 9.80665, 1e-5);
}

TEST_F(Kwr75SensorTest, GoesToZeroWhenFramesStopAndRecovers)
{
    provide();
    ASSERT_EQ(init(), CallbackReturn::SUCCESS);
    ASSERT_EQ(start(), CallbackReturn::SUCCESS);
    cycle(2);
    ASSERT_TRUE(sensor_->online());

    sim_.answer = false;
    cycle(2);
    EXPECT_NE(state(0), 0.0);  // within offline_after_ms: the last sample
    std::this_thread::sleep_for(40ms);
    cycle(1);
    EXPECT_FALSE(sensor_->online());
    for (std::size_t i = 0; i < 6; ++i) EXPECT_EQ(state(i), 0.0);

    sim_.answer = true;
    cycle(2);
    EXPECT_TRUE(sensor_->online());
    EXPECT_NE(state(0), 0.0);
}

TEST_F(Kwr75SensorTest, RejectsBadConfiguration)
{
    EXPECT_EQ(init({{"bus", ""}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({{"bus", "usb_rs485://"}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({{"convert_to_si", "yes"}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({{"offline_after_ms", "0"}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({}, {"force.x", "force.y", "force.z"}), CallbackReturn::ERROR);
    EXPECT_EQ(init({}, {"force.y", "force.x", "force.z", "torque.x", "torque.y", "torque.z"}), CallbackReturn::ERROR);
}

TEST_F(Kwr75SensorTest, BadGravityFailsInitInsteadOfAFallback)
{
    EXPECT_EQ(init({{"gravity", "9.8m"}}), CallbackReturn::ERROR);
    EXPECT_EQ(init({{"gravity", "9.81"}}), CallbackReturn::SUCCESS);
}
