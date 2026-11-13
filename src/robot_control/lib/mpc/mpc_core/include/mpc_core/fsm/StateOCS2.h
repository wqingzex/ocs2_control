//
// OCS2 (MPC) state of the controller state machine, shared by the arm and whole-body controllers.
// Everything specific to one robot model sits behind MpcRuntimeBase.
//
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <robot_controller_core/CtrlInterfaces.h>
#include <robot_controller_core/FSMState.h>

#include "mpc_core/MpcRuntimeBase.h"

namespace ocs2::mpc_core
{
    class StateOCS2 : public robot_controller_core::FSMState
    {
    public:
        using CtrlInterfaces = robot_controller_core::CtrlInterfaces;
        using FSMStateName = robot_controller_core::FSMStateName;

        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                  const std::shared_ptr<MpcRuntimeBase>& ctrl_comp = nullptr);

        ~StateOCS2() override;

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        [[nodiscard]] bool needsAsyncExit() const override { return true; }
        void beginExit() override;
        [[nodiscard]] bool tryFinishExit() override;

    private:
        void mpcUpdateThread();
        void stopMpcThread();

        std::shared_ptr<MpcRuntimeBase> ctrl_comp_;
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        std::vector<std::string> joint_names_;
        double mpc_period_{};
        rclcpp::Time last_mpc_time_;
        int thread_sleep_duration_ms_{};

        std::thread mpc_thread_;
        std::atomic_bool mpc_running_{false};         // loop while true; beginExit sets false
        std::atomic_bool mpc_thread_finished_{true};  // set true when thread leaves loop
        std::atomic_bool mpc_update_requested_{false};

        bool collision_detected_{false};
    };
}
