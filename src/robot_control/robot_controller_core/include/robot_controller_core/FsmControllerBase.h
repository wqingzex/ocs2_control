//
// Shared skeleton of the ros2_control controllers that run a HOME / HOLD / MOVEJ (/ OCS2) state machine.
//
// What is common here: joint and interface parameters, command/state interface configuration,
// loaning the interfaces on activate, the state-switch sequence in update() and the fsm_state topic.
// What differs is expressed through the virtual hooks below.
//
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <robot_log/blackbox.hpp>
#include <robot_log/blackbox_types.hpp>
#include <robot_log/health.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "robot_controller_core/CtrlInterfaces.h"
#include "robot_controller_core/FSMState.h"

namespace robot_controller_core
{
    class FsmControllerBase : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /// Runs the state machine: onCycleBegin -> run current state -> optional switch -> onCycleEnd.
        controller_interface::return_type update(const rclcpp::Time& time,
                                                 const rclcpp::Duration& period) override;

    protected:
        // ---- helpers for the derived controller's lifecycle callbacks ----

        /// on_init: controller name, update rate, command_prefix, joints, command/state interface types.
        void declareHardwareParameters();
        /// on_configure: subscribe to "fsm_command".
        void setupFsmCommandSubscription();
        /// on_configure: publish the current state code on the transient-local topic "fsm_state".
        void setupFsmStatePublisher();
        /// on_activate: clear ctrl_interfaces_ and loan the command/state interfaces into it.
        void assignInterfaces();
        /// on_activate: detect position / mixed control from the loaned interfaces and log it.
        void detectControlModeAndLog();
        /// on_activate: enter `initial`, put the machine in NORMAL mode and publish fsm_state.
        void startFsm(const std::shared_ptr<FSMState>& initial);

        void publishCurrentFsmState() const;

        // ---- hooks ----

        virtual std::shared_ptr<FSMState> getNextState(FSMStateName state_name) const = 0;
        /// Start of every update(), before the current state runs.
        virtual void onCycleBegin(const rclcpp::Time& /*time*/) {}
        /// End of every update(); `fsm_at_cycle_start` is the state name seen at the start of the cycle.
        virtual void onCycleEnd(const rclcpp::Time& /*time*/, const std::string& /*fsm_at_cycle_start*/) {}
        /// While an asynchronous exit is pending: keep the joints where they are.
        virtual void holdWhileExiting() {}
        /// After the old state has exited and before the new one is entered.
        virtual void onLeaveState(FSMStateName /*leaving*/) {}
        /// After the new state has been entered and fsm_state published.
        virtual void onEnterState(FSMStateName /*entered*/) {}

        std::string controller_name_;
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        CtrlInterfaces ctrl_interfaces_;

        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        FSMStateName next_state_name_{FSMStateName::INVALID};
        FSMMode mode_{FSMMode::NORMAL};
        /// True while an async-exit state has beginExit()'d but not tryFinishExit()'d yet.
        bool async_exit_started_{false};

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscription_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_state_publisher_;

        /// Once per health window (10 s, 1 s in debug mode), on the controller's node and never in the real-time thread,
        /// writes the control-cycle line: "[HEALTH] <controller> window=... cycles=... late=..." (WARN with [CTL-011]
        /// when some cycles came late). Call on activate and stop on deactivate.
        void startHealthReport();
        void stopHealthReport();

    private:
        void switchToNextState(FSMStateName leaving);

        // Control-cycle statistics: filled by update() (a few relaxed atomics), read by the health timer.
        robot_log::CycleStats cycle_stats_;

        // Blackbox (shared memory ring named after the controller): measured positions and commands of every
        // cycle, and the state changes. Made on the first activation; nullptr when no ring could be made.
        void recordCycleState(FSMStateName state);
        /// WARN once per /fsm_command message that asked for a state change the current state does not accept.
        void reportIgnoredCommand();
        void recordCycleCommand(FSMStateName state);
        std::shared_ptr<robot_log::BlackboxRecorder> blackbox_;
        std::uint64_t last_cycle_begin_ns_{0};
        /// Counts /fsm_command messages (written by the subscription thread); update() compares it with the last one
        /// it looked at, so a command that changed nothing is reported once per message (CTL-013).
        std::atomic<std::uint64_t> fsm_command_seq_{0};
        std::uint64_t fsm_command_seq_seen_{0};
        std::uint64_t position_read_failures_total_{0};  ///< measured-position reads that failed since activation (CTL-012)
        std::uint64_t rejected_commands_total_{0};       ///< position commands refused since activation (CTL-015)
        std::uint64_t late_period_ns_{0};
        double health_window_s_{10.0};
        rclcpp::TimerBase::SharedPtr health_timer_;

        std::unordered_map<std::string,
                           std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"effort", &ctrl_interfaces_.joint_force_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        std::unordered_map<std::string,
                           std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
            {"effort", &ctrl_interfaces_.joint_force_state_interface_}
        };
    };
}
