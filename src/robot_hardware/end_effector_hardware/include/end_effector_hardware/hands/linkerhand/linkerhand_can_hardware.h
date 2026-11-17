#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "end_effector_hardware/hands/linkerhand/linkerhand_can_protocol.h"
#include "robot_comm/can.h"

namespace end_effector_hardware
{
    /**
     * LinkerHand dexterous hand on a CAN bus (one hand per ros2_control system).
     *
     * Hardware parameters:
     *   hand_type      robot.end_effector.type key, selects the model (linkerhand_l6)
     *   hand_side      left | right; selects the CAN ID (left 0x28, right 0x27) unless can_id is given
     *   can_id         optional CAN ID override
     *   CAN link       shared CAN parameters of robot_comm::parseCanConfig: can_interface (default can0),
     *                  can_bitrate (1000000), can_data_bitrate (0, classic CAN), can_restart_ms (100),
     *                  can_manage_interface (true: the driver runs `sudo ip link set ...`)
     *   read_feedback  true: poll positions (50 Hz) and start from the measured pose
     *   read_tactile   true: poll the tactile matrices, published on /<model>_hand/<side>/tactile/<finger>
     *   tactile_period_ms, tactile_timeout_ms, send_initial_command
     *
     * Bus I/O runs in its own thread; read() / write() only exchange snapshots with it.
     * Node parameters <side>_tool_torque / <side>_tool_velocity (0..1) set the joint torque / velocity limits.
     */
    class LinkerHandCanHardware : public hardware_interface::SystemInterface
    {
    public:
        LinkerHandCanHardware();
        ~LinkerHandCanHardware() override;

        /// Replace the CAN bus (tests). Call before on_init; the driver then does not configure the interface.
        void setBusForTesting(std::unique_ptr<robot_comm::CanBus> bus);

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        using Finger = linkerhand::Finger;
        using SteadyTime = std::chrono::steady_clock::time_point;
        using RawJoints = linkerhand::RawJoints;
        using Joints = std::array<double, linkerhand::kMaxJointCount>;
        using TactileMatrix =
            std::array<std::array<uint8_t, linkerhand::kMaxTactileColumns>, linkerhand::kMaxTactileRows>;

        bool loadParameters();
        void declareToolParameters();
        rcl_interfaces::msg::SetParametersResult onToolParameters(const std::vector<rclcpp::Parameter>& parameters);
        bool validateJointInterfaces() const;

        void startIoThread();
        void stopIoThread();
        void ioLoop();
        bool drainFeedback();
        bool sendFrame(const robot_comm::CanFrame& frame, const char* operation);
        bool sendCommand(const RawJoints& raw_command);
        void processPendingToolSettings(SteadyTime now);
        void scheduleTactileRequest();
        bool processTactileFrame(const robot_comm::CanFrame& frame);
        void resetTactile(Finger finger);
        void publishTactile(Finger finger);

        uint8_t toRaw(double radians, std::size_t joint) const;
        double toRadians(uint8_t raw, std::size_t joint) const;
        static double initialValueForJoint(const hardware_interface::ComponentInfo& joint);

        struct TactileState
        {
            TactileMatrix matrix{};
            std::array<bool, linkerhand::kMaxTactileRows> received_rows{};
            std::size_t received_frame_count{0};
        };

        struct StateSnapshot
        {
            Joints positions{};
            SteadyTime timestamp{};
            uint64_t sequence{0};
            bool valid{false};
        };

        struct CommandSnapshot
        {
            RawJoints raw{};
            uint64_t sequence{0};
        };

        enum class ToolSettingPhase
        {
            Idle,
            WaitTorqueSecond,
            WaitVelocitySecond,
        };

        std::unique_ptr<robot_comm::CanBus> bus_;
        bool bus_injected_ = false;
        robot_comm::CanConfig can_config_;

        const linkerhand::ModelConfig* model_ = nullptr;
        std::string hand_side_ = "right";
        uint32_t can_id_ = linkerhand::kRightHandCanId;
        bool read_feedback_ = true;
        bool read_tactile_ = false;
        bool send_initial_command_ = false;
        bool export_effort_ = false;
        int tactile_timeout_ms_ = 100;
        int tactile_period_ms_ = 20;

        std::string tool_torque_parameter_name_;
        std::string tool_velocity_parameter_name_;
        std::atomic<double> tool_torque_scale_{1.0};
        std::atomic<double> tool_velocity_scale_{1.0};
        std::atomic_bool tool_settings_pending_{true};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // Shared with the I/O thread.
        std::atomic_bool stop_io_{false};
        std::atomic_bool io_failed_{false}; ///< the I/O thread stopped on a bus error; read() then reports ERROR
        std::thread io_thread_;
        std::mutex state_mutex_;
        StateSnapshot latest_state_;
        std::mutex command_mutex_;
        CommandSnapshot latest_command_;
        std::atomic_bool position_feedback_initialized_{false};

        // read() / write() only.
        uint64_t consumed_state_sequence_ = 0;
        SteadyTime last_consumed_state_time_{};

        // I/O thread only.
        uint64_t sent_command_sequence_ = 0;
        SteadyTime write_backoff_until_{};
        SteadyTime last_write_error_log_{};
        uint64_t suppressed_write_errors_ = 0;
        bool write_failed_ = false;
        bool io_position_initialized_ = false;
        bool position_request_pending_ = false;
        SteadyTime position_request_time_{};
        ToolSettingPhase tool_setting_phase_ = ToolSettingPhase::Idle;
        SteadyTime next_tool_setting_time_{};
        RawJoints pending_tool_torques_{};
        RawJoints pending_tool_velocities_{};
        bool tactile_request_pending_ = false;
        SteadyTime next_tactile_request_time_{};
        std::array<bool, linkerhand::kFingerCount> tactile_batch_complete_{};
        SteadyTime tactile_request_time_{};
        std::array<TactileState, linkerhand::kFingerCount> tactile_states_{};

        std::vector<std::string> joint_names_;
        Joints lower_limits_{};
        Joints upper_limits_{};
        Joints hw_positions_{};
        Joints hw_velocities_{};
        Joints hw_efforts_{};
        Joints hw_commands_{};
        Joints previous_positions_{};
        std::array<rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr, linkerhand::kFingerCount>
            tactile_publishers_{};
    };
} // namespace end_effector_hardware
