//
// JointController: ros2_control plugin for joint-space control without an MPC task: HOME / HOLD / MOVEJ of the
// waist, the head and the body. The hand and gripper controllers (end_effector_controller) derive from it and add
// their open / close commands.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <robot_controller_core/FsmController.h>

namespace joint_controller
{
    class JointController : public robot_controller_core::FsmController
    {
    public:
        // ---- what the end-effector behaviours may use ----
        /** The robot description published for ros2_control (empty until it is available). */
        const std::string& robotDescription() { return get_robot_description(); }
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node() { return get_node(); }
        StateList& states() { return states_; }
        robot_controller_core::CtrlInterfaces& interfaces() { return ctrl_interfaces_; }
        const std::vector<std::string>& joints() const { return joint_names_; }
        const std::string& name() const { return controller_name_; }
        const std::shared_ptr<robot_controller_core::FSMState>& currentState() const { return current_state_; }
        auto declarer() { return parameterDeclarer(); }

    protected:
        // Several controllers run side by side; only the MPC controllers publish the shared fsm_state topic.
        bool publishesFsmState() const override { return false; }
        double movejTrajectoryBlendRatio() const override { return 0.0; }
        bool prepareConfigure() override;
        void setupMoveJSubscriptions() override;
        void setupMoveJInterfaces() override;
        std::shared_ptr<tf2_ros::Buffer> waistTfBuffer() override;
        bool validateOnActivate() override;
        void onDeactivate() override;

    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_position_subscription_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
} // namespace joint_controller
