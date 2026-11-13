//
// ArmController: ros2_control plugin of the arm MPC task (split-body arms, dual-arm robots).
//
// The skeleton (lifecycle, HOME / OCS2 / HOLD / MOVEJ states, parameters) is MpcFsmController; this class adds
// what is specific to the arm task: the kinematics solver and service, Cartesian MOVEJ actions and the pose
// target handling of the MPC reference manager.
//

#include <functional>
#include <memory>
#include <string>

#include <arm_mpc/ArmMpcTask.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mpc_core/MpcFsmController.h>
#include <nav_msgs/msg/path.hpp>
#include <robot_control_msgs/srv/kinematics_service.hpp>
#include <robot_dynamics/Kinematics.h>

namespace arm_controller
{
    class ArmController final : public ocs2::mpc_core::MpcFsmController
    {
    protected:
        std::shared_ptr<ocs2::mpc_core::MpcRuntimeBase> createMpcComponent() override
        {
            auto declare = parameterDeclarer();
            task_ = std::make_shared<ocs2::arm_mpc::ArmMpcTask>(get_node(), ctrl_interfaces_, declare);
            return task_;
        }

        void onTaskCreated(
            const std::shared_ptr<robot_controller_core::GravityCompensation>& /*gravity*/) override
        {
            if (!task_->interface_)
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "OCS2 interface not available, no kinematics solver (default end effectors "
                            "left_tcp/right_tcp)");
                return;
            }

            // End effector names come from the OCS2 model info (task .info: eeFrame / eeFrame1)
            const auto& model_info = task_->interface_->getManipulatorModelInfo();
            const std::string left_ee_name = model_info.eeFrame;
            const std::string right_ee_name = model_info.eeFrame1;
            RCLCPP_INFO(get_node()->get_logger(),
                        "Using end effector names from OCS2 config: left='%s', right='%s'",
                        left_ee_name.c_str(), right_ee_name.c_str());

            kinematics_ = std::make_shared<robot_controller_core::ArmKinematics>(
                task_->interface_->getPinocchioInterface().getModel());
            kinematics_->initializeFromParameters(
                joint_names_, left_ee_name, right_ee_name, task_->interface_->dual_arm_);
        }

        void onMoveJCreated() override { states_.movej->setKinematicsSolver(kinematics_); }

        int jointLimitDim() const override { return task_->interface_->getManipulatorModelInfo().armDim; }

        void setupMoveJInterfaces() override
        {
            auto& movej_state = states_.movej;
            movej_state->setupJointTrajectoryAction("joint_trajectory_with_para");
            movej_state->setupLinearTrajectoryAction("execute_linear");
            if (task_ && task_->pose_reference_manager_ && movej_state)
            {
                auto* const reference_manager = task_->pose_reference_manager_.get();
                auto* const movej = movej_state.get();
                movej->setCartesianTfBuffer(reference_manager->getTfBuffer());
                reference_manager->setStampedTargetHandlers(
                    [movej](const geometry_msgs::msg::PoseStamped& msg)
                    {
                        movej->handleCartesianStampedTarget("left", msg);
                    },
                    [movej](const geometry_msgs::msg::PoseStamped& msg)
                    {
                        movej->handleCartesianStampedTarget("right", msg);
                    },
                    [movej](const nav_msgs::msg::Path& msg)
                    {
                        movej->handleDualCartesianStampedTarget(msg);
                    });
            }
            movej_state->setupCircleTrajectoryAction("execute_circle_use_ik");

            kinematics_service_ = get_node()->create_service<robot_control_msgs::srv::KinematicsService>(
                "kinematics_service",
                [this](const std::shared_ptr<robot_control_msgs::srv::KinematicsService::Request> request,
                       const std::shared_ptr<robot_control_msgs::srv::KinematicsService::Response> response)
                {
                    handleKinematicsService(request, response);
                });
        }

        robot_controller_core::WaistCommands::Level waistLevel() const override
        {
            return robot_controller_core::WaistCommands::Level::LIFT_TURN;
        }

        void onAfterFsmStarted() override { syncPoseTargetAcceptance(); }
        void onStateEntered(FSMStateName /*entered*/) override { syncPoseTargetAcceptance(); }

    private:
        void syncPoseTargetAcceptance() const
        {
            if (task_ && task_->pose_reference_manager_ && current_state_)
            {
                task_->pose_reference_manager_->setAcceptingTargets(
                    current_state_->state_name == robot_controller_core::FSMStateName::OCS2);
            }
        }

        void handleKinematicsService(
            const std::shared_ptr<robot_control_msgs::srv::KinematicsService::Request> request,
            const std::shared_ptr<robot_control_msgs::srv::KinematicsService::Response> response) const
        {
            if (kinematics_)
            {
                kinematics_->handleKinematicsService(request, response);
            }
            else
            {
                response->success = false;
                response->message = "Kinematics not initialized";
            }
        }

        std::shared_ptr<ocs2::arm_mpc::ArmMpcTask> task_;
        rclcpp::Service<robot_control_msgs::srv::KinematicsService>::SharedPtr kinematics_service_;
        std::shared_ptr<robot_controller_core::ArmKinematics> kinematics_;
    };
} // namespace arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
