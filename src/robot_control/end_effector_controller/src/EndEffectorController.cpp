//
// End-effector controllers: a joint controller (HOME / HOLD / MOVEJ) plus the open / close commands of a
// dexterous hand (HandController) or of a one-joint gripper (GripperController).
//
// The hand and gripper behaviours are the EndEffector implementations next to this file; this class only hands
// them the hook calls of the controller.
//

#include <memory>
#include <utility>

#include "EndEffector.h"

namespace end_effector_controller
{
    class EndEffectorController : public joint_controller::JointController
    {
    public:
        explicit EndEffectorController(std::unique_ptr<EndEffector> end_effector)
            : end_effector_(std::move(end_effector))
        {
        }

    protected:
        void onMoveJCreated() override { end_effector_->declareParameters(*this); }

        void setupMoveJInterfaces() override
        {
            JointController::setupMoveJInterfaces();
            end_effector_->setupCommands(*this);
        }

        void onCommandWritten(const rclcpp::Time& /*time*/) override { end_effector_->onCommandWritten(*this); }
        void onAfterFsmStarted() override { end_effector_->onAfterFsmStarted(*this); }

        void onDeactivate() override
        {
            end_effector_->onDeactivate();
            JointController::onDeactivate();
        }

    private:
        std::unique_ptr<EndEffector> end_effector_;
    };

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
