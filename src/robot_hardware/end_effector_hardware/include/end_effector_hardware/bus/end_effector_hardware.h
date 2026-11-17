#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "end_effector_hardware/bus/bus_device.h"

namespace end_effector_hardware
{
    /**
     * One end effector (hand or gripper) on a robot_comm bus; which one and where is in the URDF:
     *
     *   <ros2_control name="left_end_effector" type="system">
     *     <hardware>
     *       <plugin>end_effector_hardware/EndEffectorHardware</plugin>
     *       <param name="device">linkerhand_l6</param>                     jodell_rg75 | linkerhand_l6
     *       <param name="side">left</param>
     *       <param name="bus">usb_rs485:///dev/ttyUSB0</param>             or arm_rs485://<arm ip>/left/port1
     *       <param name="io_mode">async</param>                            async (default) | sync
     *     </hardware>
     *     ... joints
     *
     * Optional: torque / velocity (0..1; also node parameters, settable at run time) and the device's own parameters.
     * The device initializes in the control loop (see BusDevice), on an arm's bus once the arm has connected;
     * read() reports ERROR if that fails.
     */
    class EndEffectorHardware final : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        void declareDynamicsParameters();

        std::unique_ptr<BusDevice> device_;
        std::string bus_name_;
        robot_comm::IoMode io_mode_ = robot_comm::IoMode::Async;
        std::atomic<double> torque_{1.0};
        std::atomic<double> velocity_{1.0};
        std::atomic<bool> dynamics_changed_{false};  ///< set by the parameter callback, applied in write()
        bool failure_reported_ = false;
        bool bus_failure_reported_ = false;  ///< the bus is Failed and that was logged; cleared when it is back
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    };
}  // namespace end_effector_hardware
