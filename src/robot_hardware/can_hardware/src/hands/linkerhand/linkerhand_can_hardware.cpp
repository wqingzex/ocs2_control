// LinkerHand on a CAN bus, ported from ctrl's can_ros2_control (LinkerHandCanHardware): the behaviour is the same
// (asynchronous I/O thread, position feedback at 50 Hz, tool limits, optional tactile), the bus is robot_comm::CanBus
// and the frames come from linkerhand_can_protocol.

#include "can_hardware/hands/linkerhand/linkerhand_can_hardware.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "robot_comm_can/interface_configurators.h"
#include "robot_comm_can/socket_can_bus.h"

namespace can_hardware
{
    namespace lh = linkerhand;

    namespace
    {
        constexpr auto kLoggerName = "LinkerHandCanHardware";

        rclcpp::Logger logger() { return rclcpp::get_logger(kLoggerName); }

        bool parseBool(const std::string& value, bool default_value)
        {
            bool out = default_value;
            return robot_comm::detail::parseBool(value, out) ? out : default_value;
        }

        int parseInt(const std::string& value, int default_value)
        {
            try
            {
                return std::stoi(value, nullptr, 0);
            }
            catch (const std::exception&)
            {
                return default_value;
            }
        }
    } // namespace

    LinkerHandCanHardware::LinkerHandCanHardware() : bus_(std::make_unique<robot_comm::SocketCanBus>()) {}

    LinkerHandCanHardware::~LinkerHandCanHardware()
    {
        stopIoThread();
        if (bus_)
        {
            bus_->close();
        }
    }

    void LinkerHandCanHardware::setBusForTesting(std::unique_ptr<robot_comm::CanBus> bus)
    {
        bus_ = std::move(bus);
        bus_injected_ = true;
    }

    hardware_interface::CallbackReturn LinkerHandCanHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!loadParameters())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (info_.joints.size() != model_->joint_count)
        {
            RCLCPP_ERROR(logger(), "%s CAN hardware expects exactly %zu joints, got %zu", model_->model_name,
                         model_->joint_count, info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_names_.clear();
        for (const auto& joint : info_.joints)
        {
            joint_names_.push_back(joint.name);
        }
        if (!validateJointInterfaces())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        export_effort_ = std::any_of(info_.joints.front().state_interfaces.begin(),
                                     info_.joints.front().state_interfaces.end(),
                                     [](const auto& interface) { return interface.name == hardware_interface::HW_IF_EFFORT; });

        // Classic CAN at 1 Mbit/s by default. Default: `sudo ip link set ...` (must match the sudoers rule that
        // scripts/setup.sh host writes); can_manage_interface=false leaves an interface configured elsewhere alone.
        const auto configurator = robot_comm::makeConfigurator(can_config_);
        if (!configurator->apply(can_config_))
        {
            RCLCPP_ERROR(logger(), "Failed to configure %s: %s", can_config_.describe().c_str(),
                         configurator->lastError().c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        declareToolParameters();
        upper_limits_ = model_->upper_limits;
        for (std::size_t i = 0; i < model_->joint_count; ++i)
        {
            hw_positions_[i] = std::clamp(initialValueForJoint(info_.joints[i]), lower_limits_[i], upper_limits_[i]);
            previous_positions_[i] = hw_positions_[i];
            hw_commands_[i] = hw_positions_[i];
        }

        if (read_tactile_)
        {
            for (std::size_t index = 0; index < lh::kFingerCount; ++index)
            {
                const auto finger = static_cast<Finger>(index);
                const std::string topic = "/" + std::string(model_->model_name) + "_hand/" + hand_side_ + "/tactile/" +
                    lh::fingerName(finger);
                tactile_publishers_[index] = get_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
            }
        }

        RCLCPP_INFO(logger(), "Configured %s CAN hardware: %s, side=%s, can_id=0x%X, feedback=%s, tactile=%s",
                    model_->model_name, can_config_.describe().c_str(), hand_side_.c_str(), can_id_,
                    read_feedback_ ? "true" : "false", read_tactile_ ? "true" : "false");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool LinkerHandCanHardware::loadParameters()
    {
        const auto get = [this](const std::string& name, const std::string& fallback)
        {
            const auto it = info_.hardware_parameters.find(name);
            return it == info_.hardware_parameters.end() ? fallback : it->second;
        };

        const std::string hand_type = get("hand_type", "");
        model_ = lh::findModel(hand_type);
        if (model_ == nullptr)
        {
            RCLCPP_ERROR(logger(), "hand_type '%s' is not supported by the LinkerHand CAN driver (supported: linkerhand_l6)",
                         hand_type.c_str());
            return false;
        }

        robot_comm::CanConfig defaults;
        defaults.bitrate = lh::kCanBitrate;
        defaults.data_bitrate = 0;
        defaults.restart_ms = 100;
        std::string error;
        if (!robot_comm::parseCanConfig(info_.hardware_parameters, "", defaults, can_config_, error))
        {
            RCLCPP_ERROR(logger(), "Invalid CAN parameters: %s", error.c_str());
            return false;
        }
        if (can_config_.fd())
        {
            RCLCPP_ERROR(logger(), "LinkerHand uses classic CAN: can_data_bitrate must be 0");
            return false;
        }
        if (bus_injected_)
        {
            can_config_.manage_interface = false;
        }

        hand_side_ = get("hand_side", hand_side_);
        if (hand_side_ != "left" && hand_side_ != "right")
        {
            RCLCPP_ERROR(logger(), "hand_side must be 'left' or 'right', got '%s'", hand_side_.c_str());
            return false;
        }
        can_id_ = lh::canIdForSide(hand_side_);
        if (const auto it = info_.hardware_parameters.find("can_id"); it != info_.hardware_parameters.end())
        {
            can_id_ = static_cast<uint32_t>(parseInt(it->second, static_cast<int>(can_id_)));
        }

        read_feedback_ = parseBool(get("read_feedback", "true"), read_feedback_);
        read_tactile_ = parseBool(get("read_tactile", "false"), read_tactile_);
        send_initial_command_ = parseBool(get("send_initial_command", "false"), send_initial_command_);
        tactile_timeout_ms_ = std::max(1, parseInt(get("tactile_timeout_ms", "100"), tactile_timeout_ms_));
        tactile_period_ms_ = std::max(1, parseInt(get("tactile_period_ms", "20"), tactile_period_ms_));

        tool_torque_parameter_name_ = hand_side_ + "_tool_torque";
        tool_velocity_parameter_name_ = hand_side_ + "_tool_velocity";
        const auto parse_scale = [&get](const std::string& name)
        {
            try
            {
                return std::clamp(std::stod(get(name, "1.0")), 0.0, 1.0);
            }
            catch (const std::exception&)
            {
                return 1.0;
            }
        };
        tool_torque_scale_.store(parse_scale(tool_torque_parameter_name_));
        tool_velocity_scale_.store(parse_scale(tool_velocity_parameter_name_));
        return true;
    }

    void LinkerHandCanHardware::declareToolParameters()
    {
        const auto node = get_node();
        if (!node)
        {
            return;
        }
        if (!node->has_parameter(tool_torque_parameter_name_))
        {
            node->declare_parameter<double>(tool_torque_parameter_name_, tool_torque_scale_.load());
        }
        if (!node->has_parameter(tool_velocity_parameter_name_))
        {
            node->declare_parameter<double>(tool_velocity_parameter_name_, tool_velocity_scale_.load());
        }
        parameter_callback_handle_ = node->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) { return onToolParameters(parameters); });
    }

    rcl_interfaces::msg::SetParametersResult LinkerHandCanHardware::onToolParameters(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        double next_torque = tool_torque_scale_.load();
        double next_velocity = tool_velocity_scale_.load();
        bool changed = false;
        for (const auto& parameter : parameters)
        {
            const auto& name = parameter.get_name();
            if (name != tool_torque_parameter_name_ && name != tool_velocity_parameter_name_)
            {
                continue;
            }
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
                parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                result.successful = false;
                result.reason = name + " must be numeric";
                return result;
            }
            const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                ? static_cast<double>(parameter.as_int())
                : parameter.as_double();
            if (value < 0.0 || value > 1.0)
            {
                result.successful = false;
                result.reason = name + " must be in [0.0, 1.0]";
                return result;
            }
            (name == tool_torque_parameter_name_ ? next_torque : next_velocity) = value;
            changed = true;
        }
        if (changed)
        {
            tool_torque_scale_.store(next_torque);
            tool_velocity_scale_.store(next_velocity);
            tool_settings_pending_.store(true);
        }
        return result;
    }

    bool LinkerHandCanHardware::validateJointInterfaces() const
    {
        bool expected_effort = false;
        bool first = true;
        for (const auto& joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1 ||
                joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_ERROR(logger(), "Joint '%s' must expose exactly one position command interface", joint.name.c_str());
                return false;
            }
            const auto has_state = [&joint](const std::string& name)
            {
                return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                                   [&name](const auto& interface) { return interface.name == name; });
            };
            const bool has_effort = has_state(hardware_interface::HW_IF_EFFORT);
            if (first)
            {
                expected_effort = has_effort;
                first = false;
            }
            if (!has_state(hardware_interface::HW_IF_POSITION) || !has_state(hardware_interface::HW_IF_VELOCITY) ||
                has_effort != expected_effort)
            {
                RCLCPP_ERROR(logger(),
                             "Joint '%s' must expose position and velocity state interfaces, with effort declared "
                             "consistently for all %s joints",
                             joint.name.c_str(), model_->model_name);
                return false;
            }
        }
        return true;
    }

    hardware_interface::CallbackReturn LinkerHandCanHardware::on_activate(const rclcpp_lifecycle::State&)
    {
        bus_->close();
        if (!bus_->open(can_config_))
        {
            RCLCPP_ERROR(logger(), "Failed to open %s: %s", can_config_.interface.c_str(), bus_->lastError().c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        tool_settings_pending_.store(true);
        position_feedback_initialized_.store(false);
        io_failed_.store(false);
        io_position_initialized_ = false;
        position_request_pending_ = false;
        tool_setting_phase_ = ToolSettingPhase::Idle;
        tactile_request_pending_ = false;
        next_tactile_request_time_ = std::chrono::steady_clock::now();
        tactile_batch_complete_.fill(false);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = StateSnapshot{};
        }
        consumed_state_sequence_ = 0;
        last_consumed_state_time_ = SteadyTime{};
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_ = CommandSnapshot{};
        }
        sent_command_sequence_ = 0;
        write_backoff_until_ = SteadyTime{};
        last_write_error_log_ = SteadyTime{};
        suppressed_write_errors_ = 0;
        write_failed_ = false;
        for (std::size_t index = 0; index < lh::kFingerCount; ++index)
        {
            resetTactile(static_cast<Finger>(index));
        }
        hw_commands_ = hw_positions_;

        if (send_initial_command_ && !read_feedback_)
        {
            RawJoints raw{};
            for (std::size_t slot = 0; slot < model_->joint_count; ++slot)
            {
                const auto joint = model_->protocol_to_ros[slot];
                raw[slot] = toRaw(hw_commands_[joint], joint);
            }
            if (!sendCommand(raw))
            {
                bus_->close();
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else if (send_initial_command_)
        {
            RCLCPP_WARN(logger(), "Ignoring send_initial_command because position feedback is enabled");
        }

        RCLCPP_INFO(logger(), "%s CAN hardware activated on %s", model_->model_name, can_config_.interface.c_str());
        startIoThread();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LinkerHandCanHardware::on_deactivate(const rclcpp_lifecycle::State&)
    {
        stopIoThread();
        bus_->close();
        RCLCPP_INFO(logger(), "%s CAN hardware deactivated", model_ ? model_->model_name : "LinkerHand");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> LinkerHandCanHardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
        for (std::size_t i = 0; i < model_->joint_count; ++i)
        {
            interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            if (export_effort_)
            {
                interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
            }
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> LinkerHandCanHardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
        for (std::size_t i = 0; i < model_->joint_count; ++i)
        {
            interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
                joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return interfaces;
    }

    hardware_interface::return_type LinkerHandCanHardware::read(const rclcpp::Time&, const rclcpp::Duration& period)
    {
        if (!bus_->isOpen() || io_failed_.load())
        {
            return hardware_interface::return_type::ERROR;
        }
        previous_positions_ = hw_positions_;

        StateSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            snapshot = latest_state_;
        }
        if (read_feedback_ && snapshot.valid && snapshot.sequence != consumed_state_sequence_)
        {
            hw_positions_ = snapshot.positions;
            const double dt = last_consumed_state_time_ == SteadyTime{}
                ? 0.0
                : std::chrono::duration<double>(snapshot.timestamp - last_consumed_state_time_).count();
            for (std::size_t i = 0; i < model_->joint_count; ++i)
            {
                hw_efforts_[i] = 0.0;
                hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                    ? (hw_positions_[i] - previous_positions_[i]) / dt
                    : 0.0;
            }
            last_consumed_state_time_ = snapshot.timestamp;
            consumed_state_sequence_ = snapshot.sequence;
            if (!position_feedback_initialized_.exchange(true))
            {
                hw_commands_ = hw_positions_;
                RCLCPP_INFO(logger(), "Initialized %s commands from position feedback", model_->model_name);
            }
        }
        else if (!read_feedback_)
        {
            hw_positions_ = hw_commands_;
            const double dt = period.seconds();
            for (std::size_t i = 0; i < model_->joint_count; ++i)
            {
                hw_efforts_[i] = 0.0;
                hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                    ? (hw_positions_[i] - previous_positions_[i]) / dt
                    : 0.0;
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LinkerHandCanHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        if (!bus_->isOpen() || io_failed_.load())
        {
            return hardware_interface::return_type::ERROR;
        }
        // Wait for the first measured pose: the URDF's initial values would otherwise make the hand jump.
        if (read_feedback_ && !position_feedback_initialized_.load())
        {
            return hardware_interface::return_type::OK;
        }
        RawJoints raw{};
        for (std::size_t slot = 0; slot < model_->joint_count; ++slot)
        {
            const auto joint = model_->protocol_to_ros[slot];
            hw_commands_[joint] = std::clamp(hw_commands_[joint], lower_limits_[joint], upper_limits_[joint]);
            raw[slot] = toRaw(hw_commands_[joint], joint);
        }
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_.raw = raw;
            ++latest_command_.sequence;
        }
        if (!read_feedback_)
        {
            hw_positions_ = hw_commands_;
        }
        return hardware_interface::return_type::OK;
    }

    void LinkerHandCanHardware::startIoThread()
    {
        stop_io_.store(false);
        io_thread_ = std::thread(&LinkerHandCanHardware::ioLoop, this);
    }

    void LinkerHandCanHardware::stopIoThread()
    {
        stop_io_.store(true);
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
    }

    void LinkerHandCanHardware::ioLoop()
    {
        constexpr auto kPositionPeriod = std::chrono::milliseconds(20);
        constexpr auto kPositionTimeout = std::chrono::milliseconds(40);
        auto next_position_request = std::chrono::steady_clock::now();
        io_position_initialized_ = !read_feedback_;

        while (!stop_io_.load())
        {
            const auto now = std::chrono::steady_clock::now();
            if (read_feedback_ && now >= next_position_request)
            {
                if (!position_request_pending_ || now - position_request_time_ >= kPositionTimeout)
                {
                    if (sendFrame(lh::makePositionRequest(can_id_), "position request"))
                    {
                        position_request_pending_ = true;
                        position_request_time_ = now;
                    }
                }
                do
                {
                    next_position_request += kPositionPeriod;
                } while (next_position_request <= now);
            }

            if (io_position_initialized_)
            {
                processPendingToolSettings(now);
                const bool tool_settings_ready =
                    tool_setting_phase_ == ToolSettingPhase::Idle && !tool_settings_pending_.load();
                if (read_tactile_ && tool_settings_ready)
                {
                    scheduleTactileRequest();
                }
                CommandSnapshot command;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    command = latest_command_;
                }
                if (tool_settings_ready && command.sequence != 0 && command.sequence != sent_command_sequence_)
                {
                    // Always send the latest target, also when unchanged: a successful send does not mean the hand
                    // received it, and skipping repeats could leave the hand stuck after one lost frame.
                    sendCommand(command.raw);
                    sent_command_sequence_ = command.sequence;
                }
            }

            const auto before_wait = std::chrono::steady_clock::now();
            auto wake_time = before_wait + std::chrono::milliseconds(5);
            if (read_feedback_)
            {
                wake_time = std::min(wake_time, next_position_request);
            }
            if (tool_setting_phase_ != ToolSettingPhase::Idle)
            {
                wake_time = std::min(wake_time, next_tool_setting_time_);
            }
            if (read_tactile_)
            {
                wake_time = std::min(wake_time, tactile_request_pending_
                                                    ? tactile_request_time_ + std::chrono::milliseconds(tactile_timeout_ms_)
                                                    : next_tactile_request_time_);
            }
            const auto timeout = std::chrono::milliseconds(std::clamp<int64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(wake_time - before_wait).count(), 0, 5));
            if (bus_->waitReadable(timeout) && !drainFeedback())
            {
                io_failed_.store(true);
                break;
            }
        }
    }

    bool LinkerHandCanHardware::drainFeedback()
    {
        constexpr std::size_t kMaxFramesPerDrain = 128;
        for (std::size_t i = 0; i < kMaxFramesPerDrain; ++i)
        {
            robot_comm::CanFrame frame;
            const auto result = bus_->receive(frame);
            if (result == robot_comm::CanReceive::Empty)
            {
                break;
            }
            if (result == robot_comm::CanReceive::Error)
            {
                RCLCPP_ERROR(logger(), "Failed to read CAN frame on %s: %s", can_config_.interface.c_str(),
                             bus_->lastError().c_str());
                return false;
            }
            // The socket is shared with other hands on the same interface: only frames from this hand's ID count.
            if (frame.id != can_id_ || !frame.isStandardData())
            {
                continue;
            }
            if (read_tactile_ && processTactileFrame(frame))
            {
                continue;
            }
            RawJoints raw{};
            if (!lh::parsePositionFeedback(frame, can_id_, model_->joint_count, raw))
            {
                continue;
            }
            StateSnapshot snapshot;
            for (std::size_t slot = 0; slot < model_->joint_count; ++slot)
            {
                const auto joint = model_->protocol_to_ros[slot];
                snapshot.positions[joint] = toRadians(raw[slot], joint);
            }
            snapshot.timestamp = std::chrono::steady_clock::now();
            snapshot.valid = true;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                snapshot.sequence = latest_state_.sequence + 1;
                latest_state_ = snapshot;
            }
            io_position_initialized_ = true;
            position_request_pending_ = false;
        }
        return true;
    }

    bool LinkerHandCanHardware::sendFrame(const robot_comm::CanFrame& frame, const char* operation)
    {
        constexpr auto kBackoff = std::chrono::milliseconds(100);
        constexpr auto kErrorLogPeriod = std::chrono::seconds(2);
        const auto now = std::chrono::steady_clock::now();
        if (now < write_backoff_until_)
        {
            ++suppressed_write_errors_;
            return false;
        }
        if (bus_->send(frame))
        {
            if (write_failed_)
            {
                RCLCPP_INFO(logger(), "%s CAN writes recovered on %s (suppressed %llu repeated failures)",
                            model_->model_name, can_config_.interface.c_str(),
                            static_cast<unsigned long long>(suppressed_write_errors_));
                write_failed_ = false;
                suppressed_write_errors_ = 0;
            }
            return true;
        }
        // A full transmit queue (no ACK on the bus, hand off) is the usual cause: back off instead of spinning.
        write_backoff_until_ = now + kBackoff;
        ++suppressed_write_errors_;
        if (!write_failed_ || now - last_write_error_log_ >= kErrorLogPeriod)
        {
            RCLCPP_ERROR(logger(),
                         "Failed to write %s %s to ID 0x%X on %s: %s (%llu failures since last recovery; further "
                         "errors throttled)",
                         model_->model_name, operation, can_id_, can_config_.interface.c_str(),
                         bus_->lastError().c_str(), static_cast<unsigned long long>(suppressed_write_errors_));
            last_write_error_log_ = now;
        }
        write_failed_ = true;
        return false;
    }

    bool LinkerHandCanHardware::sendCommand(const RawJoints& raw_command)
    {
        return sendFrame(lh::makeJointFrame(can_id_, lh::kPositionCommand, raw_command, model_->joint_count),
                         "position command");
    }

    void LinkerHandCanHardware::processPendingToolSettings(SteadyTime now)
    {
        // Each limit frame is sent twice, 6 ms apart (the hand drops a setting that arrives while it is busy).
        const auto send = [this](uint8_t command, const RawJoints& values)
        {
            return sendFrame(lh::makeJointFrame(can_id_, command, values, model_->joint_count), "joint setting");
        };
        if (tool_setting_phase_ == ToolSettingPhase::Idle)
        {
            if (!tool_settings_pending_.exchange(false))
            {
                return;
            }
            pending_tool_torques_.fill(lh::scaleToRaw(tool_torque_scale_.load()));
            pending_tool_velocities_.fill(lh::scaleToRaw(tool_velocity_scale_.load()));
            if (!send(lh::kMaxTorqueCommand, pending_tool_torques_))
            {
                tool_settings_pending_.store(true);
                return;
            }
            next_tool_setting_time_ = now + std::chrono::milliseconds(6);
            tool_setting_phase_ = ToolSettingPhase::WaitTorqueSecond;
            return;
        }
        if (now < next_tool_setting_time_)
        {
            return;
        }
        if (tool_setting_phase_ == ToolSettingPhase::WaitTorqueSecond)
        {
            if (!send(lh::kMaxTorqueCommand, pending_tool_torques_) ||
                !send(lh::kMaxVelocityCommand, pending_tool_velocities_))
            {
                tool_settings_pending_.store(true);
                tool_setting_phase_ = ToolSettingPhase::Idle;
                return;
            }
            next_tool_setting_time_ = now + std::chrono::milliseconds(6);
            tool_setting_phase_ = ToolSettingPhase::WaitVelocitySecond;
            return;
        }
        if (!send(lh::kMaxVelocityCommand, pending_tool_velocities_))
        {
            tool_settings_pending_.store(true);
            tool_setting_phase_ = ToolSettingPhase::Idle;
            return;
        }
        tool_setting_phase_ = ToolSettingPhase::Idle;
        RCLCPP_INFO(logger(), "Applied %s %s tool limits: torque=%u, velocity=%u", model_->model_name,
                    hand_side_.c_str(), pending_tool_torques_.front(), pending_tool_velocities_.front());
    }

    void LinkerHandCanHardware::scheduleTactileRequest()
    {
        const auto now = std::chrono::steady_clock::now();
        if (tactile_request_pending_)
        {
            if (now - tactile_request_time_ <= std::chrono::milliseconds(tactile_timeout_ms_))
            {
                return;
            }
            tactile_request_pending_ = false;
        }
        if (now < next_tactile_request_time_)
        {
            return;
        }
        tactile_batch_complete_.fill(false);
        for (std::size_t index = 0; index < lh::kFingerCount; ++index)
        {
            resetTactile(static_cast<Finger>(index));
        }
        for (std::size_t index = 0; index < lh::kFingerCount; ++index)
        {
            if (!sendFrame(lh::makeTactileRequest(can_id_, static_cast<Finger>(index), model_->tactile_layout),
                           "tactile request"))
            {
                return;
            }
        }
        tactile_request_time_ = now;
        next_tactile_request_time_ = now + std::chrono::milliseconds(tactile_period_ms_);
        tactile_request_pending_ = true;
    }

    bool LinkerHandCanHardware::processTactileFrame(const robot_comm::CanFrame& frame)
    {
        lh::TactileChunk chunk;
        if (!lh::parseTactileFrame(frame, can_id_, *model_, chunk))
        {
            return false;
        }
        const auto index = static_cast<std::size_t>(chunk.finger);
        auto& state = tactile_states_[index];
        std::copy_n(chunk.values, model_->tactile_values_per_frame, state.matrix[chunk.row].begin() + chunk.column);
        if (!state.received_rows[chunk.row])
        {
            state.received_rows[chunk.row] = true;
            ++state.received_frame_count;
        }
        if (state.received_frame_count == model_->tactile_frame_count && !tactile_batch_complete_[index])
        {
            tactile_batch_complete_[index] = true;
            publishTactile(chunk.finger);
            if (std::all_of(tactile_batch_complete_.begin(), tactile_batch_complete_.end(), [](bool done) { return done; }))
            {
                tactile_request_pending_ = false;
            }
        }
        return true;
    }

    void LinkerHandCanHardware::resetTactile(Finger finger)
    {
        auto& state = tactile_states_[static_cast<std::size_t>(finger)];
        for (auto& row : state.matrix)
        {
            row.fill(0);
        }
        state.received_rows.fill(false);
        state.received_frame_count = 0;
    }

    void LinkerHandCanHardware::publishTactile(Finger finger)
    {
        const auto index = static_cast<std::size_t>(finger);
        if (!tactile_publishers_[index])
        {
            return;
        }
        std_msgs::msg::UInt8MultiArray message;
        const std::size_t point_count = model_->tactile_frame_count * model_->tactile_values_per_frame;
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "row";
        message.layout.dim[0].size = model_->tactile_frame_count;
        message.layout.dim[0].stride = point_count;
        message.layout.dim[1].label = "column";
        message.layout.dim[1].size = model_->tactile_values_per_frame;
        message.layout.dim[1].stride = model_->tactile_values_per_frame;
        message.data.reserve(point_count);
        for (std::size_t row = 0; row < model_->tactile_frame_count; ++row)
        {
            const auto& values = tactile_states_[index].matrix[row];
            message.data.insert(message.data.end(), values.begin(), values.begin() + model_->tactile_values_per_frame);
        }
        tactile_publishers_[index]->publish(message);
    }

    uint8_t LinkerHandCanHardware::toRaw(double radians, std::size_t joint) const
    {
        return lh::radiansToRaw(radians, lower_limits_[joint], upper_limits_[joint]);
    }

    double LinkerHandCanHardware::toRadians(uint8_t raw, std::size_t joint) const
    {
        return lh::rawToRadians(raw, lower_limits_[joint], upper_limits_[joint]);
    }

    double LinkerHandCanHardware::initialValueForJoint(const hardware_interface::ComponentInfo& joint)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
            {
                try
                {
                    return std::stod(interface.initial_value);
                }
                catch (const std::exception&)
                {
                    return 0.0;
                }
            }
        }
        return 0.0;
    }
} // namespace can_hardware

PLUGINLIB_EXPORT_CLASS(can_hardware::LinkerHandCanHardware, hardware_interface::SystemInterface)
