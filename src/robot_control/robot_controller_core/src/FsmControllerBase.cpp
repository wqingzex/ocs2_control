#include "robot_controller_core/FsmControllerBase.h"

#include <cmath>

namespace robot_controller_core
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration FsmControllerBase::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" + interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration FsmControllerBase::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type FsmControllerBase::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
    {
        robot_log::CycleTimer cycle_timer(cycle_stats_, last_cycle_begin_ns_, late_period_ns_);  // health line, see startHealthReport()
        const std::string fsm_at_cycle_start = current_state_ ? current_state_->state_name_string : "none";
        onCycleBegin(time);
        if (current_state_) { recordCycleState(current_state_->state_name); }

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                if (flight_)
                {
                    flight_->record(robot_log::flight::kCtlFsm,
                                    {static_cast<double>(current_state_->state_name), static_cast<double>(next_state_name_),
                                     static_cast<double>(ctrl_interfaces_.fsm_command_)});
                }
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s (fsm_command=%d)",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str(),
                            static_cast<int>(ctrl_interfaces_.fsm_command_));
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            const FSMStateName leaving = current_state_->state_name;
            if (current_state_->needsAsyncExit())
            {
                if (!async_exit_started_)
                {
                    current_state_->beginExit();
                    async_exit_started_ = true;
                    holdWhileExiting();
                }
                else if (current_state_->tryFinishExit())
                {
                    switchToNextState(leaving);
                    async_exit_started_ = false;
                    mode_ = FSMMode::NORMAL;
                    ctrl_interfaces_.fsm_command_ = 0;
                }
                else
                {
                    holdWhileExiting();
                }
            }
            else
            {
                current_state_->exit();
                switchToNextState(leaving);
                mode_ = FSMMode::NORMAL;
                ctrl_interfaces_.fsm_command_ = 0;
            }
        }

        onCycleEnd(time, fsm_at_cycle_start);
        ctrl_interfaces_.recordCommandMotion(period.seconds());
        if (current_state_) { recordCycleCommand(current_state_->state_name); }
        return controller_interface::return_type::OK;
    }

    void FsmControllerBase::recordCycleState(const FSMStateName state)
    {
        if (!flight_) { return; }
        double positions[robot_log::kFlightMaxValues];
        std::size_t n = 0;
        for (auto& interface : ctrl_interfaces_.joint_position_state_interface_)
        {
            if (n == robot_log::kFlightMaxValues) { break; }   // a record holds 28 values: the first 28 joints
            positions[n++] = interface.get().get_optional().value_or(std::nan(""));
        }
        flight_->record(robot_log::flight::kCtlState, positions, n, static_cast<std::uint32_t>(state));
    }

    void FsmControllerBase::recordCycleCommand(const FSMStateName state)
    {
        if (!flight_) { return; }
        const auto& sent = ctrl_interfaces_.last_sent_joint_positions_;
        flight_->record(robot_log::flight::kCtlCommand, sent.data(), sent.size(), static_cast<std::uint32_t>(state));
    }

    void FsmControllerBase::startHealthReport()
    {
        if (!flight_) { flight_ = robot_log::FlightRecorder::create(get_node()->get_name()); }
        ctrl_interfaces_.flight = flight_;
        // A cycle is late when it starts more than 1.5 periods after the previous one.
        late_period_ns_ = ctrl_interfaces_.frequency_ > 0.0
            ? static_cast<std::uint64_t>(1.5e9 / ctrl_interfaces_.frequency_)
            : 0;
        last_cycle_begin_ns_ = 0;
        cycle_stats_.take();  // drop whatever an earlier activation left
        health_window_s_ = robot_log::healthWindowSeconds();
        health_timer_ = get_node()->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(health_window_s_)),
            [this]() {
                robot_log::logCycleHealth(get_node()->get_logger(), get_node()->get_name(), health_window_s_,
                                          cycle_stats_.take(), "CTL-011");
            });
    }

    void FsmControllerBase::stopHealthReport()
    {
        health_timer_.reset();
    }

    void FsmControllerBase::switchToNextState(const FSMStateName leaving)
    {
        onLeaveState(leaving);
        current_state_ = next_state_;
        current_state_->enter();
        publishCurrentFsmState();
        onEnterState(current_state_->state_name);
    }

    void FsmControllerBase::declareHardwareParameters()
    {
        controller_name_ = get_node()->get_name();

        get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

        command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    }

    void FsmControllerBase::setupFsmCommandSubscription()
    {
        // Relative name: respects the ControllerManager / robot namespace (multi-robot).
        fsm_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
            "fsm_command", 10, [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                ctrl_interfaces_.fsm_command_ = msg->data;
            });
    }

    void FsmControllerBase::setupFsmStatePublisher()
    {
        fsm_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
            "fsm_state", rclcpp::QoS(1).transient_local());
    }

    void FsmControllerBase::assignInterfaces()
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        for (auto& interface : command_interfaces_)
        {
            const std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        for (auto& interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }
    }

    void FsmControllerBase::detectControlModeAndLog()
    {
        ctrl_interfaces_.detectAndSetControlMode();

        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Mixed control mode enabled - detected position, velocity, effort interfaces"
                        " (kp/kd optional)");
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Position control mode enabled - standard position control interfaces detected");
        }
    }

    void FsmControllerBase::startFsm(const std::shared_ptr<FSMState>& initial)
    {
        current_state_ = initial;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;
        publishCurrentFsmState();
    }

    void FsmControllerBase::publishCurrentFsmState() const
    {
        if (!fsm_state_publisher_ || !current_state_)
        {
            return;
        }

        int32_t state_code = 0;
        switch (current_state_->state_name)
        {
        case FSMStateName::HOME:
            state_code = 1;
            break;
        case FSMStateName::HOLD:
            state_code = 2;
            break;
        case FSMStateName::OCS2:
            state_code = 3;
            break;
        case FSMStateName::MOVEJ:
            state_code = 4;
            break;
        case FSMStateName::INVALID:
        default:
            state_code = 0;
            break;
        }

        std_msgs::msg::Int32 msg;
        msg.data = state_code;
        fsm_state_publisher_->publish(msg);
    }
} // namespace robot_controller_core
