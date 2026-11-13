#include <joint_controller/JointController.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace joint_controller
{
    // Initialize limits synchronously; activation must not depend on a subscription callback.
    bool JointController::prepareConfigure()
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
    void JointController::setupMoveJSubscriptions()
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

    void JointController::setupMoveJInterfaces()
    {
        states_.movej->setupJointTrajectoryService("joint_trajectory_with_para");
    }

    // A stock buffer + listener: there is no MPC reference manager to borrow one from.
    std::shared_ptr<tf2_ros::Buffer> JointController::waistTfBuffer()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node()->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        return tf_buffer_;
    }

    bool JointController::validateOnActivate()
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

    void JointController::onDeactivate()
    {
        if (states_.movej)
        {
            states_.movej->exit();
        }
    }
} // namespace joint_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(joint_controller::JointController, controller_interface::ControllerInterface)
