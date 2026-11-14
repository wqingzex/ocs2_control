//
// End-effector controllers: the joint-space state machine (HOME / HOLD / MOVEJ) of robot_controller_core plus the
// open / close commands of a dexterous hand or a one-joint gripper.
//
// EndEffectorController is the ros2_control controller. The hand and gripper behaviour is an EndEffector it owns:
// the controller hands it the hook calls and lends it the state machine through the accessors below.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <robot_controller_core/FsmController.h>

namespace end_effector_controller
{
    class EndEffectorController;

    class EndEffector
    {
    public:
        virtual ~EndEffector() = default;

        // Declare the end-effector parameters (called once the MOVEJ state exists).
        virtual void declareParameters(EndEffectorController& c) = 0;
        // Create the end-effector command subscriptions.
        virtual void setupCommands(EndEffectorController& c) = 0;
        // The hardware interface loans are only valid between these two calls.
        virtual void onAfterFsmStarted(EndEffectorController&) {}
        virtual void onDeactivate() {}
        // Real-time thread, once per cycle after the current state wrote its command.
        virtual void onCommandWritten(EndEffectorController&) {}
    };

    std::unique_ptr<EndEffector> makeHandEndEffector();
    std::unique_ptr<EndEffector> makeGripperEndEffector();

    class EndEffectorController : public robot_controller_core::FsmController
    {
    public:
        explicit EndEffectorController(std::unique_ptr<EndEffector> end_effector);

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
        void onMoveJCreated() override;
        void onCommandWritten(const rclcpp::Time& time) override;
        void onAfterFsmStarted() override;
        void onDeactivate() override;

    private:
        std::unique_ptr<EndEffector> end_effector_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_position_subscription_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
} // namespace end_effector_controller
