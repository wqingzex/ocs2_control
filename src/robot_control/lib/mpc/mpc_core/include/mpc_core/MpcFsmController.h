//
// Controller skeleton shared by the OCS2 (MPC) controllers (arm and whole body): FsmController plus an MPC
// component and the OCS2 state.
//
// What it adds to FsmController: the MPC component created in createMpcComponent(), the per-cycle MPC runtime
// calls, the OCS2 state, the visualization thread, and the MOVEJ joint limits and self-collision guard taken
// from the planning model. What differs per robot model is reached through the virtual hooks; only
// createMpcComponent() is mandatory.
//
#pragma once

#include <memory>

#include <robot_controller_core/FsmController.h>
#include <mpc_core/MpcRuntimeBase.h>
#include <mpc_core/fsm/StateOCS2.h>

namespace ocs2::mpc_core
{
    class MpcFsmController : public robot_controller_core::FsmController
    {
    protected:
        using FSMState = robot_controller_core::FSMState;
        using FSMStateName = robot_controller_core::FSMStateName;

        /** Builds the robot-specific MPC component (declare its parameters through parameterDeclarer()). */
        virtual std::shared_ptr<MpcRuntimeBase> createMpcComponent() = 0;
        /** Number of trailing model joints the MOVEJ joint-limit clamp applies to. */
        virtual int jointLimitDim() const { return static_cast<int>(joint_names_.size()); }

        // ---- FsmController hooks ----
        void createTask() override;
        [[nodiscard]] const pinocchio::Model* planningModel() const override;
        void createTaskStates() override;
        void onInitFinished() override;
        robot_controller_core::FsmFlavor flavor() const override { return robot_controller_core::FsmFlavor::MPC; }
        // Reuse the PoseBasedReferenceManager buffer (relative /<ns>/tf), not a stock TransformListener on absolute
        // /tf which misses multi-robot Isaac TF.
        std::shared_ptr<tf2_ros::Buffer> waistTfBuffer() override;
        // Visualization thread: whole controller active lifetime (HOLD / MOVEJ / OCS2).
        void startRuntime() override;
        void stopRuntime() override;

        // ---- FsmControllerBase hooks ----
        void onCycleBegin(const rclcpp::Time& time) override;
        void onCycleEnd(const rclcpp::Time& time, const std::string& fsm_at_cycle_start) override;
        void holdWhileExiting() override;
        void onEnterState(FSMStateName entered) override;

        std::shared_ptr<MpcRuntimeBase> mpc_;
    };
} // namespace ocs2::mpc_core
