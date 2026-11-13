//
// Common FSM State Base Class for Arm Controllers
//
#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "robot_controller_core/CtrlInterfaces.h"

namespace robot_controller_core
{
    // FSM State names enum
    enum class FSMStateName
    {
        INVALID,
        HOME,
        HOLD,
        MOVEJ,  // Move Joint - joint-space target (all RobotController instances)
        OCS2    // For arm_controller
    };

    /**
     * Which transition rules a controller uses.
     * JOINT: joint-space only (no MPC): HOME --2/3/4--> HOLD, HOLD --3/4--> MOVEJ.
     * MPC:   the controller owns an MPC task:  HOME --2--> HOLD,     HOLD --3--> OCS2, HOLD --4--> MOVEJ.
     * Commands: 1 = HOME, 2 = HOLD, 3 = OCS2 (MPC) or MOVEJ (joint), 4 = MOVEJ.
     */
    enum class FsmFlavor
    {
        JOINT,
        MPC
    };

    // FSM Mode enum
    enum class FSMMode
    {
        NORMAL,
        CHANGE
    };

    // Base FSM State class
    class FSMState
    {
    public:
        virtual ~FSMState() = default;

        FSMState(FSMStateName state_name, std::string state_name_string, CtrlInterfaces& ctrl_interfaces)
            : state_name(state_name), 
              state_name_string(std::move(state_name_string)),
              ctrl_interfaces_(ctrl_interfaces)
        {
        }

        virtual void enter() = 0;
        virtual void run(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

        /**
         * Optional async exit for states that join worker threads (must not block RT update).
         * When needsAsyncExit(): controller calls beginExit() once, then polls tryFinishExit()
         * across cycles before enter() of the next state. Default: synchronous exit().
         */
        [[nodiscard]] virtual bool needsAsyncExit() const { return false; }
        virtual void beginExit() { exit(); }
        [[nodiscard]] virtual bool tryFinishExit() { return true; }

        FSMStateName state_name;
        std::string state_name_string;

    protected:
        CtrlInterfaces& ctrl_interfaces_;
    };
} // namespace robot_controller_core

