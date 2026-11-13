//
// WholeBodyController: ros2_control plugin of the whole-body MPC task (wheeled humanoid).
//
// The skeleton (lifecycle, HOME / OCS2 / HOLD / MOVEJ states, parameters) is MpcFsmController; this class adds
// what is specific to the whole-body task: the home reference of the MPC, the world TF and the humanoid mode.
//

#include <memory>
#include <vector>

#include <mpc_core/MpcFsmController.h>
#include <whole_body_mpc/WholeBodyMpcTask.h>

namespace whole_body_controller
{
    class WholeBodyController final : public ocs2::mpc_core::MpcFsmController
    {
    protected:
        std::shared_ptr<ocs2::mpc_core::MpcRuntimeBase> createMpcComponent() override
        {
            auto declare = parameterDeclarer();
            task_ = std::make_shared<ocs2::whole_body_mpc::WholeBodyMpcTask>(get_node(), ctrl_interfaces_, declare);
            return task_;
        }

        void onHomeStateCreated() override
        {
            if (task_ && states_.home)
            {
                const std::vector<double> home_joints = states_.home->getConfiguration(0);
                if (!home_joints.empty())
                {
                    task_->setHomeJointReferenceJoints(home_joints);
                }
            }
        }

        void setupMoveJInterfaces() override
        {
            states_.movej->setupJointTrajectoryService("joint_trajectory_with_para");
        }

        void onObservationUpdated(const rclcpp::Time& time) override { task_->publishWorldTF(time); }

        void onActivateBeforeVisualization() override
        {
            task_->beginMobileBaseTfProbe();
            task_->resetHumanoidModeToFsmDefault();
        }

        void onDeactivate() override
        {
            if (states_.movej)
            {
                states_.movej->exit();
            }
        }

        // Stop MPC before resetting ModeSchedule so the preSolverRun flush is not raced by the MPC thread.
        // OCS2 joins the MPC/viz threads on exit, which must not block a single RT update cycle (async exit).
        void onLeaveState(FSMStateName leaving) override
        {
            if (leaving == robot_controller_core::FSMStateName::OCS2)
            {
                task_->resetHumanoidModeToFsmDefault();
            }
        }

    private:
        std::shared_ptr<ocs2::whole_body_mpc::WholeBodyMpcTask> task_;
    };
} // namespace whole_body_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whole_body_controller::WholeBodyController, controller_interface::ControllerInterface)
