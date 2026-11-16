#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_comm/bus_hub.h"

namespace end_effector_hardware
{
    /** The KWR75 wire format (custom RS485, not Modbus). */
    namespace kwr75
    {
        constexpr std::size_t kAxes = 6;
        constexpr std::size_t kFrameLength = 28;  ///< [0x48][0xAA][Fx Fy Fz Mx My Mz: float32, low byte first][0x0D][0x0A]
        /** One sample on request (command 0x48). */
        std::vector<uint8_t> pollRequest();
        /** A reply frame -> the six values in the sensor's units (kgf, kgf*m). False if it is not a valid frame. */
        bool parseFrame(const std::vector<uint8_t>& frame, std::array<double, kAxes>& out);
    }  // namespace kwr75

    /**
     * KWR75 six-axis force / torque sensor on a robot_comm bus, so it can be wired to a USB-RS485 adapter or to an
     * arm's flange port alike:
     *
     *   <ros2_control name="left_ft" type="sensor">
     *     <hardware>
     *       <plugin>end_effector_hardware/Kwr75Sensor</plugin>
     *       <param name="bus">usb_rs485:///dev/ttyUSB4?baud=2500000</param>    or arm_rs485://<arm ip>/left/port2
     *     </hardware>
     *     <sensor name="left_ft_sensor"> force.x force.y force.z torque.x torque.y torque.z </sensor>
     *   </ros2_control>
     *
     * Each read() takes the reply to the previous request and sends the next one, so it never waits on the bus; the
     * values are one bus round trip old. Optional: wrench_topic (also publishes geometry_msgs/WrenchStamped),
     * frame_id, convert_to_si (default true: kgf -> N with `gravity`), offline_after_ms (default 100: without a good
     * frame for that long the outputs go to 0 until frames come back). On an arm's bus nothing is sent until the arm
     * has connected.
     */
    class Kwr75Sensor final : public hardware_interface::SensorInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        /** A good frame arrived within offline_after_ms. */
        bool online() const { return online_; }

    private:
        void take(const std::vector<uint8_t>& frame);
        void publish(const rclcpp::Time& time);

        std::string sensor_name_;
        std::string bus_name_;
        std::string wrench_topic_;
        std::string frame_id_ = "ft_sensor";
        double scale_ = 9.80665;
        std::chrono::milliseconds offline_after_{100};
        std::shared_ptr<robot_comm::BusClient> bus_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;

        std::array<double, kwr75::kAxes> wrench_{};
        bool outstanding_ = false;
        bool online_ = false;
        bool reported_offline_ = false;
        std::chrono::steady_clock::time_point last_good_{};
        uint64_t bad_frames_ = 0;
    };
}  // namespace end_effector_hardware
