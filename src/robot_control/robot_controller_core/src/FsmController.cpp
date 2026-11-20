#include "robot_controller_core/FsmController.h"

#include <robot_log/log_mode.hpp>

namespace robot_controller_core
{
    // ------------------------------------------------------------------------------------------------------------
    // FsmControllerBase hooks
    // ------------------------------------------------------------------------------------------------------------

    std::shared_ptr<FSMState> FsmController::getNextState(const FSMStateName state_name) const
    {
        switch (state_name)
        {
        case FSMStateName::INVALID:
            return states_.invalid;
        case FSMStateName::HOME:
            return states_.home;
        case FSMStateName::OCS2:
            return states_.ocs2;
        case FSMStateName::HOLD:
            return states_.hold;
        case FSMStateName::MOVEJ:
            return states_.movej;
        default:
            return states_.invalid;
        }
    }

    void FsmController::onCycleBegin(const rclcpp::Time& time) { onObservationUpdated(time); }

    void FsmController::onCycleEnd(const rclcpp::Time& time, const std::string& /*fsm_at_cycle_start*/)
    {
        onCommandWritten(time);
    }

    void FsmController::onEnterState(const FSMStateName entered) { onStateEntered(entered); }

    // ------------------------------------------------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------------------------------------------------

    controller_interface::CallbackReturn FsmController::on_init()
    {
        try
        {
            robot_log::applyLogMode();  // debug mode of this run, applied once
            declareHardwareParameters();

            // State machine parameters
            rest_pos_ = auto_declare<std::vector<double>>("rest_pos", rest_pos_);

            // Force control parameters
            ctrl_interfaces_.default_gains_ = auto_declare<std::vector<double>>(
                "default_gains", ctrl_interfaces_.default_gains_);

            // PD control parameters (for the OCS2 state)
            ctrl_interfaces_.pd_gains_ = auto_declare<std::vector<double>>("pd_gains", ctrl_interfaces_.pd_gains_);

            auto declare = parameterDeclarer();

            createTask();

            std::shared_ptr<robot_dynamics::GravityCompensation> gravity_compensation = nullptr;
            if (const auto* model = planningModel())
            {
                gravity_compensation = std::make_shared<robot_dynamics::GravityCompensation>(*model);
                RCLCPP_INFO(get_node()->get_logger(), "Gravity compensation initialized from OCS2 Pinocchio model");
            }
            onTaskCreated(gravity_compensation);

            // Home state
            auto_declare<double>("home_duration", 3.0);
            auto_declare<std::string>("home_interpolation_type", "linear");
            auto_declare<double>("home_tanh_scale", 3.0);
            auto_declare<int>("switch_command_base", 100);
            states_.home = std::make_shared<StateHome>(ctrl_interfaces_, gravity_compensation, get_node(), flavor());

            // home_1 ... home_10; StateHome warns when there is none
            states_.home->init(
                [this](const std::string& name, const std::vector<double>& default_value)
                {
                    return this->auto_declare<std::vector<double>>(name, default_value);
                });

            // Configure rest pose if available
            if (!rest_pos_.empty())
            {
                states_.home->setRestPose(rest_pos_);
                RCLCPP_INFO(get_node()->get_logger(), "Rest pose configured with %zu joints", rest_pos_.size());
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "No rest pose configured, using home pose only");
            }
            onHomeStateCreated();
            createTaskStates();

            // Hold state
            auto_declare<double>("hold_first_check_position_threshold", 0.1);
            auto_declare<double>("hold_position_threshold", 0.1);
            states_.hold = std::make_shared<StateHold>(ctrl_interfaces_, get_node(), gravity_compensation, flavor());

            // MoveJ state
            auto_declare<double>("movej_duration", 3.0);
            auto_declare<std::string>("movej_interpolation_type", "linear");
            auto_declare<double>("movej_tanh_scale", 3.0);
            auto_declare<double>("movej_trajectory_duration", 3.0);
            auto_declare<double>("movej_trajectory_blend_ratio", movejTrajectoryBlendRatio());
            auto_declare<double>("movej_max_velocity", 2.0);
            auto_declare<double>("movej_max_acceleration", 4.0);
            auto_declare<double>("movej_max_jerk", 20.0);
            auto_declare<bool>("movej_auto_extend_duration", true);

            states_.movej = std::make_shared<StateMoveJ>(ctrl_interfaces_, get_node(), joint_names_, gravity_compensation);
            onMoveJCreated();

            waist_config_ = WaistCommands::declareParameters(declare);
            if (waist_config_.enabled)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Waist lifting enabled for controller %s",
                            controller_name_.c_str());
            }

            onInitFinished();

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn FsmController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        if (!prepareConfigure())
        {
            return CallbackReturn::ERROR;
        }
        if (publishesFsmState())
        {
            setupFsmStatePublisher();
        }
        setupFsmCommandSubscription();

        setupMoveJSubscriptions();

        // Feed URDF joint limits into the HOME / MOVEJ states and the waist planner
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (states_.home)
                {
                    states_.home->updateJointLimitsFromURDF(msg->data, joint_names_);
                }
                if (states_.movej)
                {
                    states_.movej->updateJointLimitsFromURDF(msg->data);
                }
            });

        setupMoveJInterfaces();

        if (waist_config_.enabled)
        {
            const auto level = waistLevel();
            tf_buffer_ = waistTfBuffer();
            if (level == WaistCommands::Level::FULL && !tf_buffer_)
            {
                RCLCPP_WARN(get_node()->get_logger(), "waist absolute TF: no TF buffer available");
            }
            waist_commands_.setup(
                get_node(), controller_name_, states_.movej,
                [this]() { return current_state_ && current_state_->state_name == FSMStateName::MOVEJ; },
                level, tf_buffer_, waist_config_);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FsmController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        assignInterfaces();
        if (!validateOnActivate())
        {
            ctrl_interfaces_.clear();
            return CallbackReturn::ERROR;
        }
        detectControlModeAndLog();

        ctrl_interfaces_.initializeLastSentPositions();
        onActivateBeforeVisualization();
        startRuntime();

        startFsm(states_.hold);
        onAfterFsmStarted();
        startHealthReport();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FsmController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        stopHealthReport();
        stopRuntime();
        onDeactivate();
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FsmController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        waist_commands_.reset();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FsmController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FsmController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }
} // namespace robot_controller_core
