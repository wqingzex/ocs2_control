#include "robot_controller_core/WaistCommands.h"

#include <cmath>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_controller_core
{
    void WaistCommands::setup(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                              const std::string& controller_name,
                              const std::shared_ptr<StateMoveJ>& movej,
                              std::function<bool()> movej_active,
                              const Level level,
                              const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                              const Config& config)
    {
        if (level == Level::FULL)
        {
            if (movej)
            {
                movej->setupWaistAbsoluteTransform(tf_buffer, config.source_frame, config.target_frame);
                movej->setupWaistLiftingPoseAction("waist_lifting_pose");
            }

            // Relative with controller name -> /<ns>/<controller>/waist_*
            pose_relative_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                controller_name + "/waist_lifting_pose_relative", 10,
                [node, movej, movej_active](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    if (!movej_active() || !movej)
                    {
                        return;
                    }
                    if (msg->data.size() < 3)
                    {
                        RCLCPP_WARN(node->get_logger(),
                                    "waist_lifting_pose_relative expects [dx, dz, dphi], got %zu values",
                                    msg->data.size());
                        return;
                    }

                    const Eigen::Vector3d lifting_delta(msg->data[0], msg->data[1], msg->data[2]);
                    if (!movej->moveWaistLifting(lifting_delta))
                    {
                        RCLCPP_WARN(node->get_logger(), "waist lifting x/z/phi delta command failed");
                    }
                });

            pose_absolute_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                controller_name + "/waist_lifting_pose_absolute", 10,
                [node, movej, movej_active, tf_buffer, config](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    if (!movej_active() || !movej)
                    {
                        return;
                    }
                    if (msg->data.size() < 3)
                    {
                        RCLCPP_WARN(node->get_logger(),
                                    "waist_lifting_pose_absolute expects [x, z, phi], got %zu values",
                                    msg->data.size());
                        return;
                    }
                    if (!tf_buffer)
                    {
                        RCLCPP_WARN(node->get_logger(), "TF buffer is not initialized");
                        return;
                    }

                    geometry_msgs::msg::PointStamped target_base;
                    target_base.header.frame_id = config.source_frame;
                    target_base.header.stamp = rclcpp::Time(0);
                    target_base.point.x = msg->data[0];
                    target_base.point.y = 0.0;
                    target_base.point.z = msg->data[1];

                    try
                    {
                        const auto transform = tf_buffer->lookupTransform(
                            config.target_frame, config.source_frame, tf2::TimePointZero);

                        geometry_msgs::msg::PointStamped target_body;
                        tf2::doTransform(target_base, target_body, transform);
                        if (std::abs(target_body.point.y) > 1.0e-4)
                        {
                            RCLCPP_WARN(node->get_logger(),
                                        "Transformed waist absolute target has non-zero y in %s: %.6f; using x/z only",
                                        config.target_frame.c_str(), target_body.point.y);
                        }

                        const Eigen::Vector3d target_xz_phi(target_body.point.x, target_body.point.z, msg->data[2]);
                        if (!movej->moveWaistLiftingToBodyBaseXz(target_xz_phi))
                        {
                            RCLCPP_WARN(node->get_logger(), "waist absolute x/z/phi command failed");
                        }
                    }
                    catch (const tf2::TransformException& ex)
                    {
                        RCLCPP_WARN(node->get_logger(),
                                    "Failed to transform waist absolute target from %s to %s: %s",
                                    config.source_frame.c_str(), config.target_frame.c_str(), ex.what());
                    }
                });
        }

        lifting_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
            controller_name + "/waist_lifting_command", 10,
            [node, movej, movej_active](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (!movej_active() || !movej)
                {
                    return;
                }
                if (!movej->setWaistLiftingFactor(msg->data))
                {
                    RCLCPP_WARN(node->get_logger(), "waist lifting command failed");
                }
            });

        turning_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
            controller_name + "/waist_turning_command", 10,
            [node, movej, movej_active](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (!movej_active() || !movej)
                {
                    return;
                }
                if (!movej->setWaistTurningFactor(msg->data))
                {
                    RCLCPP_WARN(node->get_logger(), "waist turning command failed");
                }
            });

        if (level == Level::FULL)
        {
            phi_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
                controller_name + "/waist_phi_command", 10,
                [node, movej, movej_active](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    if (node->get_current_state().label() != "active")
                    {
                        return;
                    }
                    if (!movej_active() || !movej)
                    {
                        return;
                    }
                    if (!movej->setWaistPhiFactor(msg->data))
                    {
                        RCLCPP_WARN(node->get_logger(), "waist phi command failed");
                    }
                });
        }
    }

    void WaistCommands::reset()
    {
        pose_relative_subscription_.reset();
        pose_absolute_subscription_.reset();
        lifting_subscription_.reset();
        turning_subscription_.reset();
        phi_subscription_.reset();
    }
} // namespace robot_controller_core
