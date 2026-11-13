//
// Waist lifting / turning / phi command topics, shared by every controller that drives a waist.
//
// The controller keeps the MOVEJ state; this class only declares the waist_* parameters and registers the
// subscriptions that forward commands to it. Commands are ignored unless the controller is in MOVEJ.
//
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.hpp>

#include "robot_controller_core/StateMoveJ.h"

namespace robot_controller_core
{
    class WaistCommands
    {
    public:
        /// LIFT_TURN: lifting factor and turning factor. FULL: also relative / absolute pose, phi, pose action.
        enum class Level
        {
            LIFT_TURN,
            FULL
        };

        struct Config
        {
            bool enabled{false};
            std::string source_frame{"base_footprint"};
            std::string target_frame{"body_base"};
        };

        /**
         * Declares waist_lifting_enabled and, when enabled, every waist_* parameter the waist planner reads.
         * `auto_declare(name, default)` must declare a parameter of the default's type and return its value.
         */
        template <typename AutoDeclareFunc>
        static Config declareParameters(AutoDeclareFunc& auto_declare)
        {
            Config config;
            config.enabled = auto_declare("waist_lifting_enabled", false);
            if (!config.enabled)
            {
                return config;
            }
            auto_declare("waist_lifting_duration", 3.0);
            auto_declare("waist_lifting_default_parameter", std::vector<double>{0.25, 1.0, 5.0});
            auto_declare("waist_turning_default_parameter", std::vector<double>{0.25, 1.0, 5.0});
            auto_declare("waist_turning_direction", 1.0);
            auto_declare("waist_phi_default_parameter", std::vector<double>{0.25, 1.0, 5.0});
            const std::string waist_lifting_type = auto_declare("waist_lifting_type", std::string("three_joint"));
            if (waist_lifting_type == "three_joint")
            {
                auto_declare("waist_l1", 0.322);
                auto_declare("waist_l2", 0.355);
                auto_declare("waist_rotation_direction", std::vector<double>{1.0, 1.0, 1.0});
                auto_declare("waist_angle_offset", std::vector<double>{0.0, 0.0, 0.0});
            }
            else
            {
                auto_declare("waist_single_joint_direction", 1.0);
                auto_declare("waist_single_joint_offset", 0.0);
                auto_declare("waist_single_joint_pitch_joint", std::string("body_joint2"));
                auto_declare("waist_single_joint_pitch_direction", 1.0);
                auto_declare("waist_single_joint_pitch_offset", 0.0);
            }
            config.source_frame = auto_declare("waist_absolute_source_frame", config.source_frame);
            config.target_frame = auto_declare("waist_absolute_target_frame", config.target_frame);
            return config;
        }

        /**
         * Registers the subscriptions as /<ns>/<controller_name>/waist_*.
         * @param movej_active returns true while the controller is in MOVEJ
         * @param tf_buffer only used at Level::FULL for the absolute pose command (may be null: the command warns)
         */
        void setup(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                   const std::string& controller_name,
                   const std::shared_ptr<StateMoveJ>& movej,
                   std::function<bool()> movej_active,
                   Level level,
                   const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                   const Config& config);

        /// Drops every subscription (on_cleanup).
        void reset();

    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_relative_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_absolute_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lifting_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr turning_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr phi_subscription_;
    };
}
