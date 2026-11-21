#include "juxie_hardware/jx_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <robot_log/log_mode.hpp>

#include "robot_comm_can/interface_configurators.h"
#include "robot_comm_can/socket_can_bus.h"

namespace juxie_hardware {

JxHardware::JxHardware() : bus_(std::make_unique<robot_comm::SocketCanBus>()) {}

JxHardware::~JxHardware() {
    stopControl();
}

void JxHardware::setBusForTesting(std::unique_ptr<robot_comm::CanBus> bus) {
    bus_ = std::move(bus);
    bus_injected_ = true;
}

hardware_interface::CallbackReturn JxHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto get_param = [&](const std::string& name, const std::string& default_val) {
        auto it = info_.hardware_parameters.find(name);
        return it != info_.hardware_parameters.end() ? it->second : default_val;
    };
    error_pub_ = get_node()->create_publisher<std_msgs::msg::Int64>("/Base_HardwareError", 10);
    RCLCPP_INFO(get_node()->get_logger(), "Error publisher created on /Base_HardwareError");

    // CAN 参数：所有 CAN 驱动共用同一组参数名（can_interface、can_bitrate、can_data_bitrate、can_restart_ms、
    // can_manage_interface）；Juxie 的默认值是 1M / 5M、bus-off 后 100 ms 自动恢复。
    robot_comm::CanConfig can_defaults;
    can_defaults.bitrate = 1'000'000;
    can_defaults.data_bitrate = 5'000'000;
    can_defaults.restart_ms = 100;
    std::string can_error;
    if (!robot_comm::parseCanConfig(info_.hardware_parameters, "", can_defaults, can_config_, can_error)) {
        RCLCPP_ERROR(get_node()->get_logger(), "[HW-008] Invalid CAN parameters: %s", can_error.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!can_config_.fd()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Juxie needs CAN FD (64-byte control frames): can_data_bitrate must be greater than 0");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (bus_injected_) {
        can_config_.manage_interface = false;
    }

    control_period_ms_ = std::stof(get_param("control_period_ms", "1")); // 默认1ms=1000Hz
    // Debug mode (the whole run): everything at DEBUG, and the control loop statistics on by default (every 10 s).
    const bool debug_mode = robot_log::applyLogMode();
    blackbox_ = robot_log::BlackboxRecorder::create("juxie_hardware");
    // A cycle is late when it starts more than 1.5 periods after the previous one; the period is the component's
    // rw_rate (0 = the rate of the controller manager, which this plugin does not see: then no cycle is called late,
    // the maxima are still reported).
    health_window_s_ = robot_log::healthWindowSeconds();
    const unsigned int rw_rate = get_hardware_info().rw_rate;
    late_period_ns_ = rw_rate > 0 ? static_cast<std::uint64_t>(1.5e9 / rw_rate) : UINT64_MAX;
    stats_log_period_s_ = std::stod(get_param("stats_log_period_s", debug_mode ? "10" : "0"));
    if (stats_log_period_s_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'stats_log_period_s' must be >= 0, got %.2f", stats_log_period_s_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    max_position_step_nct_ = static_cast<int32_t>(
        std::stoi(get_param("max_position_step_nct", "90")));
    if (max_position_step_nct_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_position_step_nct' must be > 0, got %d",
                     max_position_step_nct_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 加速度限幅：限制相邻两帧 delta 的变化量；0 关闭
    max_position_accel_nct_ = static_cast<int32_t>(
        std::stoi(get_param("max_position_accel_nct", "10")));
    if (max_position_accel_nct_ < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_position_accel_nct' must be >= 0, got %d",
                     max_position_accel_nct_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 解析电机ID列表
    std::string motor_ids_str = get_param("motor_ids", "");
    if (motor_ids_str.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'motor_ids' is required (e.g. '1,2' or '[1,2]')");
        return hardware_interface::CallbackReturn::ERROR;
    }
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), '['), motor_ids_str.end());
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), ']'), motor_ids_str.end());
    std::istringstream iss_ids(motor_ids_str);
    std::string token;
    while (std::getline(iss_ids, token, ',')) {
        motor_ids_.push_back(static_cast<uint8_t>(std::stoi(token)));
    }
    if (motor_ids_.empty() || motor_ids_.size() > protocol::kMaxMotors) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'motor_ids' must list 1 to %zu motors (the multi-motor frame carries at most %zu), got %zu",
                     protocol::kMaxMotors, protocol::kMaxMotors, motor_ids_.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化数据存储
    size_t motor_count = motor_ids_.size();
    joint_positions_.resize(motor_count, 0.0);
    joint_velocities_.resize(motor_count, 0.0);
    joint_efforts_.resize(motor_count, 0.0);
    joint_position_commands_.resize(motor_count, 0.0);
    raw_position_commands_.resize(motor_count, 0);
    target_raw_position_commands_.resize(motor_count, 0);
    last_sent_raw_commands_.resize(motor_count, 0);
    last_sent_deltas_.resize(motor_count, 0);

    running_ = false;
    is_first_command_ = true;

    // 打印初始化信息
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware initialized:");
    RCLCPP_INFO(get_node()->get_logger(), "  CAN Interface: %s", can_config_.describe().c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Control Period: %.2f ms", control_period_ms_);
    RCLCPP_INFO(get_node()->get_logger(), "  Max position step: %d nct/cycle", max_position_step_nct_);
    RCLCPP_INFO(get_node()->get_logger(), "  Max position accel: %d nct/cycle^2%s",
                max_position_accel_nct_,
                max_position_accel_nct_ == 0 ? " (disabled)" : "");
    RCLCPP_INFO(get_node()->get_logger(), "  Motor IDs (count: %zu):", motor_ids_.size());

    for (uint8_t id : motor_ids_) {
        RCLCPP_INFO(get_node()->get_logger(), "    - %d", id);
    }

    // 配置 CAN FD 接口：默认 `sudo ip link set ...`（命令必须与 setup.sh host 的 sudoers 规则完全一致），
    // can_manage_interface=false 时不动接口（已由别处配好）。
    configurator_ = robot_comm::makeConfigurator(can_config_);
    if (!configurator_->apply(can_config_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "[HW-007] [CAN FD] Failed to configure %s! %s",
                     can_config_.interface.c_str(), configurator_->lastError().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// === 激活硬件（初始化CAN+使能电机+获取初始状态）===
hardware_interface::CallbackReturn JxHardware::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating JxHardware...");
    locked_feedback_len_ = 0;

    // 打开 CAN FD 总线
    if (!initCanFd()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 发送同步帧→获取初始状态（仅初始化时发送一次）
    sendSyncFrame();
    if (!readMotorStates(true)) { 
        RCLCPP_ERROR(get_node()->get_logger(), "Incomplete motor state initialization");
        bus_->close();
        return hardware_interface::CallbackReturn::ERROR;
    }   
    
    // 初始化命令缓冲区（从当前反馈起步，避免首帧跃迁）
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        joint_position_commands_[i] = joint_positions_[i]; // 弧度
        double angle_deg = joint_positions_[i] * 180.0 / M_PI; // 弧度→角度
        raw_position_commands_[i] = protocol::angleToRaw(angle_deg); // 角度→电机计数
        target_raw_position_commands_[i] = raw_position_commands_[i];
        last_sent_raw_commands_[i] = raw_position_commands_[i];
        last_sent_deltas_[i] = 0;
    }

    // One health line per window for the read and the write cycle
    last_read_begin_ns_ = 0;
    last_write_begin_ns_ = 0;
    read_stats_.take();
    write_stats_.take();
    health_timer_ = get_node()->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(health_window_s_)),
        [this]() {
            const std::string name = get_node()->get_name();
            robot_log::logCycleHealth(get_node()->get_logger(), name + ".read", health_window_s_, read_stats_.take(), "HW-013");
            robot_log::logCycleHealth(get_node()->get_logger(), name + ".write", health_window_s_, write_stats_.take(), "HW-013");
        });

    // 启动控制线程
    loop_stats_ = LoopStats{};
    read_calls_ = 0;
    missed_feedback_ = 0;
    running_ = true;
    control_thread_ = std::thread(&JxHardware::controlLoop, this);

    RCLCPP_INFO(get_node()->get_logger(), "JxHardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// === 停用硬件（停止线程+关闭CAN）===
hardware_interface::CallbackReturn JxHardware::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating JxHardware...");
    stopControl();
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JxHardware::on_error(
    const rclcpp_lifecycle::State & /* previous_state */) {
    stopControl();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JxHardware::on_shutdown(
    const rclcpp_lifecycle::State & /* previous_state */) {
    stopControl();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void JxHardware::stopControl() {
    health_timer_.reset();

    // 停止控制线程（运行中因错误自行停止时 running_ 已是 false，线程仍要回收）
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (bus_) {
        bus_->close();
    }
}

// === 导出接口 ===
std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
JxHardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        // 位置状态接口（弧度）
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
        // 速度状态接口（弧度/秒）
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));

    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu state interfaces (position+velocity)", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> 
JxHardware::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        // 位置命令接口（弧度）
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu command interfaces (position)", command_interfaces.size());
    return command_interfaces;
}

// === ROS2 Control 读写接口 ===
hardware_interface::return_type JxHardware::read(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    robot_log::CycleTimer cycle_timer(read_stats_, last_read_begin_ns_, late_period_ns_);
    if (!readMotorStates(false))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "[HW-006] motor error ,return");
        return hardware_interface::return_type::ERROR;
    }
    if (blackbox_)
    {
        blackbox_->record(robot_log::blackbox::kHwRead, joint_positions_.data(), joint_positions_.size());
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type JxHardware::write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    robot_log::CycleTimer cycle_timer(write_stats_, last_write_begin_ns_, late_period_ns_);
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 只更新最新目标；实际下发由控制线程按 max_position_step_nct_ 限速追赶
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        double angle_deg = joint_position_commands_[i] * 180.0 / M_PI;
        target_raw_position_commands_[i] = protocol::angleToRaw(angle_deg);
    }
    if (blackbox_)
    {
        blackbox_->record(robot_log::blackbox::kHwWrite, joint_position_commands_.data(), joint_position_commands_.size());
    }

    return hardware_interface::return_type::OK;
}

// === 打开 CAN FD 总线（接口已由 configurator 配好）===
bool JxHardware::initCanFd() {
    if (!bus_->open(can_config_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s", bus_->lastError().c_str());
        return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "CAN FD initialized: %s (non-blocking)", can_config_.describe().c_str());
    return true;
}

// === 电机使能 ===
bool JxHardware::enableMotors() {
    for (uint8_t id : motor_ids_) {
        if (!bus_->send(protocol::makeEnableFrame(id))) {
            RCLCPP_ERROR(get_node()->get_logger(), "Enable motor %d failed: %s", id, bus_->lastError().c_str());
            bus_->close();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

// === 发送同步帧 ===
bool JxHardware::sendSyncFrame() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!bus_->send(protocol::makeSyncFrame())) {
        RCLCPP_ERROR(get_node()->get_logger(), "Sync frame send failed: %s", bus_->lastError().c_str());
        return false;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Sync frame sent (ID: 0x%X)", protocol::kSyncFrameId);
    return true;
}

// === 发送多控帧（协议0x200，控制多个电机）===
bool JxHardware::sendMultiMotorCommand() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 限速 + 限加速追赶最新目标：
    // 1) 速度：每周期 |delta| <= max_position_step_nct_
    // 2) 加速度：|delta - last_delta| <= max_position_accel_nct_（为 0 则跳过）
    // 3) 不越过当前目标（remaining 夹紧），避免指令超调
    const protocol::StepLimits limits{max_position_step_nct_, max_position_accel_nct_};
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        const protocol::StepResult step = protocol::limitStep(
            target_raw_position_commands_[i], last_sent_raw_commands_[i], last_sent_deltas_[i], limits);
        raw_position_commands_[i] = step.sent;
        last_sent_deltas_[i] = step.delta;
        last_sent_raw_commands_[i] = step.sent;
    }

    // 根据锁定的反馈帧长度选择控制字协议：
    // 7字节协议：首帧0xD8，后续0xD0；12字节协议：CSP控制字（首帧带清错位）
    const uint8_t control_byte = protocol::controlByte(locked_feedback_len_, is_first_command_);
    if (!bus_->send(protocol::makeMultiMotorFrame(control_byte, raw_position_commands_, motor_ids_))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Multi-motor command failed: %s", bus_->lastError().c_str());
        return false;
    }
    
    // 第一次发送后更新标志
    if (is_first_command_) {
        RCLCPP_INFO(get_node()->get_logger(), "Sent first CSP command with control byte 0x%02X", control_byte);
        is_first_command_ = false;
    }

    return true;
}

// === 读取电机状态（非阻塞读取+缓存所有反馈）+ 错误处理 ===
bool JxHardware::readMotorStates(bool strict_check) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 创建临时映射表存储最新反馈
    std::unordered_map<uint8_t, robot_comm::CanFrame> feedback_map;

    robot_comm::CanFrame frame;

    // strict_check（仅 on_activate 首次读）：同步帧发出后反馈可能尚未到达，非阻塞 read 会立刻空返回。
    // 在此模式下轮询等待收齐所有电机反馈；正常运行 read(false) 仍只 drain 一次，不拖 CM 周期。
    const auto strict_deadline =
        strict_check ? (std::chrono::steady_clock::now() + std::chrono::milliseconds(200))
                     : std::chrono::steady_clock::now();

    for (;;) {
        // 读取当前 RX 队列中所有可用帧（非阻塞，无数据即退出内层循环）
        while (true) {
            const robot_comm::CanReceive received = bus_->receive(frame);
            if (received == robot_comm::CanReceive::Error) {
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                      1000, "%s", bus_->lastError().c_str());
                break;
            }
            if (received == robot_comm::CanReceive::Empty) {
                break;
            }

            const protocol::Classified classified = protocol::classify(frame, motor_ids_);
            if (classified.kind == protocol::FrameKind::Feedback) {
                feedback_map[classified.motor_id] = frame;
            } else if (classified.kind == protocol::FrameKind::Emergency) {
                parseAndReportEmergencyFrame(classified.motor_id, frame);
                running_ = false;
                return false;
            }
        }

        if (!strict_check) {
            break;
        }
        if (feedback_map.size() >= motor_ids_.size()) {
            break;
        }
        if (std::chrono::steady_clock::now() >= strict_deadline) {
            break;
        }
        (void)bus_->waitReadable(std::chrono::milliseconds(2));
    }

    if(feedback_map.size() != motor_ids_.size() && strict_check) {
        if (error_pub_) {
            std_msgs::msg::Int64 msg;
            msg.data = protocol::kFeedbackTimeoutError;
            error_pub_->publish(msg);
            RCLCPP_INFO(get_node()->get_logger(), "Published hardware error: 0x%06X", 0x300200);
        }

        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000, "feedback_map size is %zu, motor ids size is: %zu", 
                            feedback_map.size(), motor_ids_.size());
        return false;
    }

    if (strict_check && locked_feedback_len_ == 0 && !feedback_map.empty()) {
        const uint8_t first_len = feedback_map.begin()->second.len;
        for (const auto& item : feedback_map) {
            if (item.second.len != first_len) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Strict check failed: inconsistent feedback frame lengths (%u vs %u)",
                             first_len, item.second.len);
                running_ = false;
                return false;
            }
        }
        if (first_len != 7 && first_len != 12) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Strict check failed: unsupported feedback frame length %u (expected 7 or 12)",
                         first_len);
            running_ = false;
            return false;
        }
        locked_feedback_len_ = first_len;
        RCLCPP_INFO(get_node()->get_logger(),
                    "Locked feedback frame length to %u bytes", locked_feedback_len_);
    }

    // 处理缓存的反馈数据
    if (!strict_check) {
        ++read_calls_;
    }
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        uint8_t id = motor_ids_[i];
        auto it = feedback_map.find(id);
        
        if (it == feedback_map.end()) {
            if (!strict_check) {
                ++missed_feedback_;
            }
            RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                1000, "No feedback for motor %d", id);
            continue;
        }

        const robot_comm::CanFrame& motor_frame = it->second;
        if (locked_feedback_len_ > 0 && motor_frame.len != locked_feedback_len_) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                               1000, "Feedback for motor %d frame length changed: %d (expected %u)",
                               id, motor_frame.len, locked_feedback_len_);
            running_ = false;
            return false;
        }

        const uint8_t protocol_len = (locked_feedback_len_ > 0) ? locked_feedback_len_ : motor_frame.len;
        protocol::MotorFeedback feedback;
        const protocol::DecodeResult decoded = protocol::decodeFeedback(motor_frame, protocol_len, feedback);
        if (decoded == protocol::DecodeResult::UnsupportedLen) {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                1000, "Unsupported feedback protocol length %u for motor %d",
                                protocol_len, id);
            running_ = false;
            return false;
        }
        if (decoded == protocol::DecodeResult::TooShort) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                               1000, "Feedback for motor %d has insufficient data length: %d (<%u)",
                               id, motor_frame.len, protocol_len);
            continue;
        }

        joint_positions_[i] = feedback.position_rad;
        joint_velocities_[i] = feedback.velocity_rad_s;
        joint_efforts_[i] = feedback.effort; // 12字节协议反馈Iq电流（mA）；7字节协议为 0

        if (!feedback.enabled) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                               5000, "Motor %d is not enabled", id);
        }
        if (protocol_len == 12 && feedback.mode != 0x03) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                5000, "Motor %d mode feedback is 0x%02X, expected CSP(0x03)",
                                id, feedback.mode);
        }
        if (feedback.fault) {
            if (protocol_len == 12) {
                if (feedback.error_code != 0) {
                    parseAndReportErrorCode(id, feedback.error_code, "feedback frame");
                }
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                    1000, "Motor %d has error, error code: 0x%04X, status: 0x%02X",
                                    id, feedback.error_code, feedback.status);
            } else {
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                    1000, "Motor %d has error, error code: 0x%02X", id, feedback.status);
            }
            running_ = false;
            return false;
        }
    }

    return true;
}

// === 发布一个故障码 ===
void JxHardware::publishFault(uint8_t motor_id, uint16_t fault_code, bool known) {
    const int64_t error_code = protocol::hardwareErrorCode(motor_id, fault_code);
    if (!error_pub_) {
        return;
    }
    std_msgs::msg::Int64 msg;
    msg.data = error_code;
    error_pub_->publish(msg);
    if (known) {
        RCLCPP_INFO(get_node()->get_logger(), "Published hardware error: 0x%06X", static_cast<uint32_t>(error_code));
    } else {
        RCLCPP_WARN(get_node()->get_logger(), "Published unknown hardware error: 0x%06X", static_cast<uint32_t>(error_code));
    }
}

// === 解析并发布 16bit 错误码（12 字节协议）===
void JxHardware::parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source) {
    std::vector<std::string> error_messages;
    const std::vector<protocol::FaultInfo> faults = protocol::decodeErrorWord(error_code_raw);
    for (const auto& fault : faults) {
        publishFault(motor_id, fault.fault_code, true);
        error_messages.push_back(fault.message);
    }

    // 如果检测到紧急帧但没有任何已知错误位，发布未知错误（0x300100）
    if (faults.empty()) {
        publishFault(motor_id, protocol::kUnknownFault, false);
        error_messages.push_back("未知错误");
    }

    std::string error_str = "Motor " + std::to_string(motor_id) + " " + source + " errors: ";
    for (size_t i = 0; i < error_messages.size(); ++i) {
        error_str += error_messages[i];
        if (i < error_messages.size() - 1) {
            error_str += ", ";
        }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "%s (raw: 0x%04X, SEVERE)", error_str.c_str(), error_code_raw);
}

// === 解析并报告紧急错误帧 ===
void JxHardware::parseAndReportEmergencyFrame(uint8_t motor_id, const robot_comm::CanFrame& frame) {
    // 确保数据长度足够
    if (frame.len < 2) {
        RCLCPP_WARN(get_node()->get_logger(),
                   "Emergency frame for motor %d has insufficient data length: %d",
                   motor_id, frame.len);
        return;
    }

    if (locked_feedback_len_ == 7) {
        std::vector<std::string> error_messages;
        const std::vector<protocol::FaultInfo> faults = protocol::decodeEmergencyBytes(frame.data[0], frame.data[1]);
        for (const auto& fault : faults) {
            publishFault(motor_id, fault.fault_code, true);
            error_messages.push_back(fault.message);
        }
        if (faults.empty()) {
            publishFault(motor_id, protocol::kUnknownFault, false);
            error_messages.push_back("未知错误");
        }

        std::string error_str = "Motor " + std::to_string(motor_id) + " emergency errors: ";
        for (size_t i = 0; i < error_messages.size(); ++i) {
            error_str += error_messages[i];
            if (i < error_messages.size() - 1) {
                error_str += ", ";
            }
        }
        RCLCPP_ERROR(get_node()->get_logger(), "%s (SEVERE)", error_str.c_str());
        return;
    }

    uint16_t error_code_raw = frame.data[0] | (static_cast<uint16_t>(frame.data[1]) << 8);
    parseAndReportErrorCode(motor_id, error_code_raw, "emergency frame");
}

// === 控制线程：周期性发送多控帧 ===
void JxHardware::controlLoop() {
    RCLCPP_INFO(get_node()->get_logger(), "Control thread started (period: %.2f ms)", control_period_ms_);
    auto period = std::chrono::microseconds(static_cast<int>(control_period_ms_ * 1000));
    const double period_us = std::chrono::duration<double, std::micro>(period).count();
    const auto loop_start = std::chrono::steady_clock::now();
    auto next_time = loop_start + period;

    // 统计窗口（stats_log_period_s > 0 时每个窗口打印一次）
    LoopStats window;
    auto window_start = loop_start;
    uint64_t window_reads = 0, window_missed = 0;
    const auto log_stats = [&](const char* title, const LoopStats& stats, double seconds, uint64_t reads,
                               uint64_t missed) {
        const robot_comm::CanStats bus_stats = bus_->stats();
        const double mean_us = stats.cycles ? stats.sum_lateness_us / static_cast<double>(stats.cycles) : 0.0;
        const double miss_pct = reads ? 100.0 * static_cast<double>(missed) /
                                            (static_cast<double>(reads) * static_cast<double>(motor_ids_.size()))
                                      : 0.0;
        RCLCPP_INFO(get_node()->get_logger(),
                    "[HW-009] %s (%.1f s): %llu cycles, wake-up lateness mean %.1f us / max %.1f us, %llu late (> half a period), "
                    "%llu failed sends; %llu reads, feedback missing in %.3f %% of motor-reads; "
                    "CAN tx %llu (errors %llu), rx %llu (errors %llu)",
                    title, seconds, static_cast<unsigned long long>(stats.cycles), mean_us, stats.max_lateness_us,
                    static_cast<unsigned long long>(stats.late_cycles),
                    static_cast<unsigned long long>(stats.send_failures), static_cast<unsigned long long>(reads),
                    miss_pct, static_cast<unsigned long long>(bus_stats.tx_frames),
                    static_cast<unsigned long long>(bus_stats.tx_errors),
                    static_cast<unsigned long long>(bus_stats.rx_frames),
                    static_cast<unsigned long long>(bus_stats.rx_errors));
    };
    const auto record = [&](LoopStats& stats, double lateness_us, bool send_failed) {
        ++stats.cycles;
        stats.sum_lateness_us += lateness_us;
        stats.max_lateness_us = std::max(stats.max_lateness_us, lateness_us);
        if (lateness_us > 0.5 * period_us) {
            ++stats.late_cycles;
        }
        if (send_failed) {
            ++stats.send_failures;
        }
    };

    while (running_) {
        const bool send_failed = !sendMultiMotorCommand();

        // 精确睡眠到下一个时间点
        std::this_thread::sleep_until(next_time);
        const auto woke = std::chrono::steady_clock::now();
        const double lateness_us = std::max(0.0, std::chrono::duration<double, std::micro>(woke - next_time).count());
        record(loop_stats_, lateness_us, send_failed);
        record(window, lateness_us, send_failed);
        next_time += period;

        // 防止累积误差：如果落后了，调整到下个周期
        if (next_time < woke) {
            next_time = woke + period;
        }

        if (stats_log_period_s_ > 0.0) {
            const double elapsed = std::chrono::duration<double>(woke - window_start).count();
            if (elapsed >= stats_log_period_s_) {
                const uint64_t reads = read_calls_.load();
                const uint64_t missed = missed_feedback_.load();
                log_stats("Control loop stats", window, elapsed, reads - window_reads, missed - window_missed);
                window = LoopStats{};
                window_start = woke;
                window_reads = reads;
                window_missed = missed;
            }
        }
    }

    log_stats("Control thread stopped, totals", loop_stats_,
              std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count(),
              read_calls_.load(), missed_feedback_.load());
}

} // namespace juxie_hardware

// 导出插件
PLUGINLIB_EXPORT_CLASS(juxie_hardware::JxHardware, hardware_interface::SystemInterface)
