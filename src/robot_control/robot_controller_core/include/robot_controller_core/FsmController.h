//
// Controller skeleton of the joint-space state machine: the lifecycle callbacks, the HOME / HOLD / MOVEJ states
// and their parameters, the URDF joint limits, the waist command topics and the hooks the concrete controllers
// fill in. MpcFsmController (mpc_core) adds an MPC task and the OCS2 state on top of it.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <robot_dynamics/GravityCompensation.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>

#include "robot_controller_core/FsmControllerBase.h"
#include "robot_controller_core/StateHold.h"
#include "robot_controller_core/StateHome.h"
#include "robot_controller_core/StateMoveJ.h"
#include "robot_controller_core/WaistCommands.h"

namespace robot_controller_core
{
    class FsmController : public FsmControllerBase
    {
    public:
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    protected:
        struct StateList
        {
            std::shared_ptr<FSMState> invalid;
            std::shared_ptr<StateHome> home;
            /** The MPC state; only controllers that own an MPC task create it. */
            std::shared_ptr<FSMState> ocs2;
            std::shared_ptr<StateHold> hold;
            std::shared_ptr<StateMoveJ> movej;
        };

        // ---- hooks -------------------------------------------------------------------------------------------

        /** on_init, after the common parameters: builds the task of the controller (the MPC component). */
        virtual void createTask() {}
        /** Planning model of the task; nullptr when the controller has none (then no gravity compensation). */
        [[nodiscard]] virtual const pinocchio::Model* planningModel() const { return nullptr; }
        /** on_init, right after createTask() (gravity is null when there is no planning model). */
        virtual void onTaskCreated(const std::shared_ptr<robot_dynamics::GravityCompensation>&) {}
        /** on_init, right after HOME is configured. */
        virtual void onHomeStateCreated() {}
        /** on_init, after onHomeStateCreated(): creates the states_ entries the controller adds (the MPC state). */
        virtual void createTaskStates() {}
        /** on_init, right after MOVEJ is created. */
        virtual void onMoveJCreated() {}
        /** on_init, last step. */
        virtual void onInitFinished() {}
        /** Transition rules of HOME / HOLD: MPC when the controller owns an MPC task, JOINT otherwise. */
        virtual FsmFlavor flavor() const { return FsmFlavor::JOINT; }
        /** Whether the current state is published on the (shared) topic fsm_state. */
        virtual bool publishesFsmState() const { return true; }
        /** Default of movej_trajectory_blend_ratio. */
        virtual double movejTrajectoryBlendRatio() const { return 0.2; }
        /** on_configure, first: false makes the controller fail to configure (e.g. no robot description yet). */
        virtual bool prepareConfigure() { return true; }
        /** on_configure: the joint_target_position / joint_target_trajectory subscriptions of MOVEJ. */
        virtual void setupMoveJSubscriptions()
        {
            states_.movej->setupSubscriptions("joint_target_position", true);
            states_.movej->setupTrajectorySubscription();
        }
        /** The TF buffer the absolute waist commands use; null disables the absolute pose command. */
        virtual std::shared_ptr<tf2_ros::Buffer> waistTfBuffer() { return nullptr; }
        /** on_activate, after the interfaces are loaned: false makes the activation fail. */
        virtual bool validateOnActivate() { return true; }
        /** on_configure: robot-specific MOVEJ actions, services and target handlers. */
        virtual void setupMoveJInterfaces() {}
        virtual WaistCommands::Level waistLevel() const { return WaistCommands::Level::FULL; }
        /** on_activate, after the interfaces are loaned and before startRuntime(). */
        virtual void onActivateBeforeVisualization() {}
        /** on_activate, after onActivateBeforeVisualization(): starts background threads of the task. */
        virtual void startRuntime() {}
        /** on_activate, after the FSM entered HOLD. */
        virtual void onAfterFsmStarted() {}
        /** on_deactivate, first: stops the threads startRuntime() started. */
        virtual void stopRuntime() {}
        /** on_deactivate, before the interfaces are released. */
        virtual void onDeactivate() {}
        /** Every cycle, after the observation was updated. */
        virtual void onObservationUpdated(const rclcpp::Time& /*time*/) {}
        /** After the current state wrote its command, before the command motion is recorded. */
        virtual void onCommandWritten(const rclcpp::Time& /*time*/) {}
        /** After a new state was entered (fsm_state already published). */
        virtual void onStateEntered(FSMStateName /*entered*/) {}

        /** Functor with the (name, default) -> value signature the task components expect. */
        auto parameterDeclarer()
        {
            return [this](const std::string& name, const auto& default_value)
            {
                using T = std::decay_t<decltype(default_value)>;
                return this->template auto_declare<T>(name, default_value);
            };
        }

        // ---- FsmControllerBase hooks ---------------------------------------------------------------------------

        std::shared_ptr<FSMState> getNextState(FSMStateName state_name) const override;
        /** Calls onObservationUpdated(); controllers with a task update their observation first. */
        void onCycleBegin(const rclcpp::Time& time) override;
        /** Calls onCommandWritten(). */
        void onCycleEnd(const rclcpp::Time& time, const std::string& fsm_at_cycle_start) override;
        /** Calls onStateEntered(). */
        void onEnterState(FSMStateName entered) override;

        StateList states_;
        std::vector<double> home_pos_;
        std::vector<double> rest_pos_;
        WaistCommands::Config waist_config_;
        WaistCommands waist_commands_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
    };
} // namespace robot_controller_core
