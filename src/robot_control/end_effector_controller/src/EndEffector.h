//
// End-effector behaviour (hand or gripper) of an end-effector controller, which is a joint controller plus the
// open / close commands.
//
#pragma once

#include <memory>
#include <string>

#include <joint_controller/JointController.h>

namespace end_effector_controller
{
    class EndEffector
    {
    public:
        virtual ~EndEffector() = default;

        // Declare the end-effector parameters (called once the MOVEJ state exists).
        virtual void declareParameters(joint_controller::JointController& c) = 0;
        // Create the end-effector command subscriptions.
        virtual void setupCommands(joint_controller::JointController& c) = 0;
        // The hardware interface loans are only valid between these two calls.
        virtual void onAfterFsmStarted(joint_controller::JointController&) {}
        virtual void onDeactivate() {}
        // Real-time thread, once per cycle after the current state wrote its command.
        virtual void onCommandWritten(joint_controller::JointController&) {}
    };

    std::unique_ptr<EndEffector> makeHandEndEffector();
    std::unique_ptr<EndEffector> makeGripperEndEffector();
} // namespace end_effector_controller
