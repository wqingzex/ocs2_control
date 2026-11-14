//
// End-effector controllers: the joint-space state machine plus the open / close commands of a dexterous hand
// (HandController) or of a one-joint gripper (GripperController).
//
// The hand and gripper behaviours are the EndEffector implementations next to this file.
//

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <tf2_ros/buffer.hpp>

#include "EndEffector.h"

namespace end_effector_controller
{
    EndEffectorController::EndEffectorController(std::unique_ptr<EndEffector> end_effector)
        : end_effector_(std::move(end_effector))
    {
    }

    // Initialize limits synchronously; activation must not depend on a subscription callback.
    bool EndEffectorController::prepareConfigure()
    {
        const auto& robot_description = robotDescription();
        if (robot_description.empty())
        {
            RCLCPP_ERROR(node()->get_logger(), "Cannot configure without robot description");
            return false;
        }
        states_.home->updateJointLimitsFromURDF(robot_description, joint_names_);
        states_.movej->updateJointLimitsFromURDF(robot_description);
        return true;
    }

    // The target is one array for all joints of this controller (no body/head/left/right sub-topics).
    void EndEffectorController::setupMoveJSubscriptions()
    {
        auto movej = states_.movej;
        // Relative with controller name -> /<ns>/<controller>/joint_target_position
        const std::string topic = controller_name_ + "/joint_target_position";
        joint_target_position_subscription_ = node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            topic, rclcpp::QoS(10),
            [movej](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                std::vector<double> target_pos(msg->data.begin(), msg->data.end());
                if (!target_pos.empty())
                {
                    movej->setTargetPosition(target_pos);
                }
            });
        RCLCPP_DEBUG(node()->get_logger(), "Subscribed to %s for all joints", topic.c_str());
        movej->setupTrajectorySubscription();
    }

    void EndEffectorController::setupMoveJInterfaces()
    {
        states_.movej->setupJointTrajectoryService("joint_trajectory_with_para");
        end_effector_->setupCommands(*this);
    }

    // A stock buffer + listener: there is no MPC reference manager to borrow one from.
    std::shared_ptr<tf2_ros::Buffer> EndEffectorController::waistTfBuffer()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node()->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        return tf_buffer_;
    }

    bool EndEffectorController::validateOnActivate()
    {
        try
        {
            states_.movej->validateHeadCouplingSetup();
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(node()->get_logger(), "Head coupling setup failed: %s", error.what());
            return false;
        }
        return true;
    }

    void EndEffectorController::onMoveJCreated() { end_effector_->declareParameters(*this); }

    void EndEffectorController::onCommandWritten(const rclcpp::Time& /*time*/)
    {
        end_effector_->onCommandWritten(*this);
    }

    void EndEffectorController::onAfterFsmStarted() { end_effector_->onAfterFsmStarted(*this); }

    void EndEffectorController::onDeactivate()
    {
        end_effector_->onDeactivate();
        if (states_.movej)
        {
            states_.movej->exit();
        }
    }

    /** Dexterous hand: target_command / target_percent blend two HOME configurations. */
    class HandController final : public EndEffectorController
    {
    public:
        HandController() : EndEffectorController(makeHandEndEffector()) {}
    };

    /** One-joint gripper: position / percent commands with force feedback. */
    class GripperController final : public EndEffectorController
    {
    public:
        GripperController() : EndEffectorController(makeGripperEndEffector()) {}
    };
} // namespace end_effector_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(end_effector_controller::HandController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(end_effector_controller::GripperController, controller_interface::ControllerInterface)
