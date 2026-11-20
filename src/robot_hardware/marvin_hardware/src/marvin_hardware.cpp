#include "marvin_hardware.h"
#include "marvin_hardware/param_parsing.h"
#include <robot_log/log_mode.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <cstdint>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace marvin_hardware
{
    // 辅助函数：标准化字符串（转大写，去除空格）
    static std::string normalizeString(const std::string& str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::toupper);
        result.erase(std::remove_if(result.begin(), result.end(), ::isspace), result.end());
        return result;
    }

    // 默认参数（单一来源）：同时用于 declare_node_parameters / on_init / applyRobotConfiguration
    static const std::vector<double> kDefaultJointKGains = {2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0};
    static const std::vector<double> kDefaultJointDGains = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
    static const std::vector<double> kDefaultCartKGains  = {1800.0, 1800.0, 1800.0, 40.0, 40.0, 40.0, 20.0};
    static const std::vector<double> kDefaultCartDGains  = {0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4};
    // PD 模式（关节阻抗+速度前馈）推荐增益：刚度 N·m/deg 常用值，阻尼系数 0.3×7
    static const std::vector<double> kDefaultPDKGains = {14.0, 14.0, 14.0, 10.5, 5.6, 5.6, 5.6};
    static const std::vector<double> kDefaultPDDGains = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

    static const std::vector<double> kDefaultLeftKineParam  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultLeftDynParam   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightKineParam = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightDynParam  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // ctrl_mode 相关：字符串 <-> 内部 mode(int) 的单一转换来源
    // mode: 1=POSITION, 2=JOINT_IMPEDANCE, 3=CART_IMPEDANCE, 4=POWER_OFF (左右臂均下使能)
    static int ctrlModeStringToMode(const std::string& ctrl_mode_raw, const rclcpp::Logger& logger)
    {
        const std::string normalized = normalizeString(ctrl_mode_raw);
        if (normalized == "POSITION") return 1;
        if (normalized == "JOINT_IMPEDANCE") return 2;
        if (normalized == "CART_IMPEDANCE") return 3;
        if (normalized == "POWER_OFF") return 4;
        if (normalized == "PD") return 5;   // PD：关节阻抗 + 速度前馈（极低延时跟踪，遥操场景）
        RCLCPP_WARN(logger, "Invalid ctrl_mode value: %s, defaulting to POSITION", ctrl_mode_raw.c_str());
        return 1;
    }

    static const char* modeToCtrlModeString(int mode)
    {
        switch (mode) {
            case 1: return "POSITION";
            case 2: return "JOINT_IMPEDANCE";
            case 3: return "CART_IMPEDANCE";
            case 4: return "POWER_OFF";
            case 5: return "PD";
            default: return "POSITION";
        }
    }

    // 辅助函数：解析 hardware_parameters 中的数组字符串为 vector<double>
    // 支持形如 "[1,2,3]"、"1, 2, 3"、"1 2 3" 等格式
    static std::vector<double> parseDoubleArrayLoose(const std::string& str, const std::vector<double>& default_val)
    {
        if (str.empty()) {
            return default_val;
        }
        std::string cleaned = str;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

        std::istringstream iss(cleaned);
        std::vector<double> result;
        double v = 0.0;
        while (iss >> v) {
            result.push_back(v);
        }
        return result.empty() ? default_val : result;
    }

    // rclcpp 参数类型映射（用于 declare_node_parameters 的类型检查）
    template<typename T>
    static constexpr rclcpp::ParameterType paramTypeOf();

    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::string>() { return rclcpp::ParameterType::PARAMETER_STRING; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<int>() { return rclcpp::ParameterType::PARAMETER_INTEGER; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<double>() { return rclcpp::ParameterType::PARAMETER_DOUBLE; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<bool>() { return rclcpp::ParameterType::PARAMETER_BOOL; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::vector<double>>() { return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY; }

    // hardware_parameters arrive as text. A value that is not a number, is out of range or is not a recognized word
    // makes on_init fail with a [HW-005] message (ros2_control logs it and the component does not activate): a limit
    // such as max_joint_speed must never silently become another value.
    [[noreturn]] static void failParameter(const std::string& what)
    {
        throw std::invalid_argument("[HW-005] hardware parameter " + what);
    }

    static auto intParser(const char* name, int lo = std::numeric_limits<int>::min(),
                          int hi = std::numeric_limits<int>::max())
    {
        return [name, lo, hi](const std::string& s, int) {
            int value = 0;
            if (!param_parsing::parseInt(s, value)) {
                failParameter("'" + std::string(name) + "' = '" + s + "' is not an integer");
            }
            if (value < lo || value > hi) {
                failParameter("'" + std::string(name) + "' = " + std::to_string(value) + " is outside [" +
                              std::to_string(lo) + ", " + std::to_string(hi) + "]");
            }
            return value;
        };
    }

    static auto doubleParser(const char* name, bool must_be_positive = false)
    {
        return [name, must_be_positive](const std::string& s, double) {
            double value = 0.0;
            if (!param_parsing::parseDouble(s, value)) {
                failParameter("'" + std::string(name) + "' = '" + s + "' is not a finite number");
            }
            if (must_be_positive && value <= 0.0) {
                failParameter("'" + std::string(name) + "' = " + s + " must be > 0");
            }
            return value;
        };
    }

    static auto boolParser(const char* name)
    {
        return [name](const std::string& str, bool) {
            const std::string s = normalizeString(str);
            if (s == "TRUE" || s == "1" || s == "YES" || s == "ON") return true;
            if (s == "FALSE" || s == "0" || s == "NO" || s == "OFF") return false;
            failParameter("'" + std::string(name) + "' = '" + str + "' is not true/false (1/0, yes/no, on/off)");
        };
    }

    void MarvinHardware::declare_node_parameters()
    {
        // 目标：优先使用 hardware_interface 的 info_.hardware_parameters 作为“初始值来源”，
        // 然后用该初始值声明为 ROS2 node 参数，便于后续 ros2 param set 动态修改。
        //
        // 规则：
        // - 如果参数已存在且类型正确：尊重现有值（可能来自 launch 覆盖/之前声明），不覆盖
        // - 如果参数已存在但类型不对：undeclare 后按正确类型重新 declare（避免类型冲突）
        // - 如果参数不存在：从 hardware_parameters 取值（若有）否则用默认值 declare

        const auto hw_find = [this](const std::string& name) -> const std::string* {
            auto it = info_.hardware_parameters.find(name);
            if (it == info_.hardware_parameters.end()) {
                return nullptr;
            }
            return &it->second;
        };

        const auto ensure_param = [this](const std::string& name, const auto& default_val, const std::string* hw_val, const auto& parse_fn) {
            using T = std::decay_t<decltype(default_val)>;
            if (node_->has_parameter(name)) {
                if (node_->get_parameter(name).get_type() != paramTypeOf<T>()) {
                    node_->undeclare_parameter(name);  // declared with another type: declare it again as ours
                } else {
                    return; // 已存在且类型正确：尊重现有值
                }
            }

            if (!node_->has_parameter(name)) {
                const T val = hw_val ? parse_fn(*hw_val, default_val) : default_val;
                node_->declare_parameter<T>(name, val);
            }
        };

        // 对 array 参数额外校验长度：类型正确但长度不对也会强制重声明，避免后续运行期兜底/越界风险
        const auto ensure_double_array_sized = [this, &hw_find](const std::string& name,
                                                               const std::vector<double>& default_val,
                                                               size_t expected_size) {
            const std::string* hw_val = hw_find(name);

            if (node_->has_parameter(name)) {
                if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                    node_->undeclare_parameter(name);  // declared with another type: declare it again as ours
                } else {
                    const auto current = node_->get_parameter(name).get_value<std::vector<double>>();
                    if (current.size() != expected_size) {
                        failParameter("'" + name + "' already declared with " + std::to_string(current.size()) +
                                      " numbers, needs exactly " + std::to_string(expected_size));
                    }
                    return; // 已存在且长度正确：尊重现有值
                }
            }

            if (!node_->has_parameter(name)) {
                std::vector<double> val = default_val;
                if (hw_val && !param_parsing::trimmed(*hw_val).empty()) {  // an empty value means "use the default"
                    std::vector<double> parsed = parseDoubleArrayLoose(*hw_val, {});
                    if (parsed.size() != expected_size) {
                        failParameter("'" + name + "' = '" + *hw_val + "' must hold exactly " +
                                      std::to_string(expected_size) + " numbers (got " +
                                      std::to_string(parsed.size()) + ")");
                    }
                    val = std::move(parsed);
                }
                node_->declare_parameter<std::vector<double>>(name, val);
            }
        };

        // 基础配置（之前通过 get_param 从 hardware_parameters 读的，也顺便声明成 ROS2 参数）
        ensure_param("arm_type", std::string("LEFT"), hw_find("arm_type"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("device_ip", std::string("192.168.1.190"), hw_find("device_ip"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("device_port", 8080, hw_find("device_port"),
                     intParser("device_port", 1, 65535));
        // Robot operation mode: "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE"
        ensure_param("ctrl_mode", std::string("POSITION"), hw_find("ctrl_mode"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        // Drag mode: -1=don't update, 0=disable, 1=joint space drag, 2=cartesian space drag
        ensure_param("drag_mode", -1, hw_find("drag_mode"),
                     intParser("drag_mode"));
        // Joint impedance K gains (7 values)
        ensure_double_array_sized("joint_k_gains", kDefaultJointKGains, 7);
        // Joint impedance D gains (7 values)
        ensure_double_array_sized("joint_d_gains", kDefaultJointDGains, 7);
        // Cartesian impedance K gains (7 values)
        ensure_double_array_sized("cart_k_gains", kDefaultCartKGains, 7);
        // Cartesian impedance D gains (7 values)
        ensure_double_array_sized("cart_d_gains", kDefaultCartDGains, 7);
        // Cartesian impedance type
        ensure_param("cart_type", 2, hw_find("cart_type"),
                     intParser("cart_type"));
        // PD mode K/D gains (7 values, N·m/deg；常用推荐值见 kDefaultPDKGains/kDefaultPDDGains)
        ensure_double_array_sized("pd_k_gains", kDefaultPDKGains, 7);
        ensure_double_array_sized("pd_d_gains", kDefaultPDDGains, 7);
        // PD 速度前馈周期 ms：0~20（<1 不添加前馈；建议 5）
        ensure_param("pd_control_period_ms", 5, hw_find("pd_control_period_ms"),
                     intParser("pd_control_period_ms", 0, 20));
        // Joint speed and acceleration limits
        ensure_param("max_joint_speed", 10.0, hw_find("max_joint_speed"),
                     doubleParser("max_joint_speed", true));
        ensure_param("max_joint_acceleration", 10.0, hw_find("max_joint_acceleration"),
                     doubleParser("max_joint_acceleration", true));
        ensure_param("use_drag_mode", false, hw_find("use_drag_mode"),
                     boolParser("use_drag_mode"));
        // Per-arm brake: true=force brake release (松闸); false=engage brake (抱闸)
        ensure_param("left_brake_release", false, hw_find("left_brake_release"),
                     boolParser("left_brake_release"));
        ensure_param("right_brake_release", false, hw_find("right_brake_release"),
                     boolParser("right_brake_release"));
        // Tool (payload) parameters - declared as std::vector<double> (array of doubles)
        // kineParam: kinematic parameters (6 values), dynPara: dynamic parameters (10 values)
        ensure_double_array_sized("left_kine_param", kDefaultLeftKineParam, 6);
        ensure_double_array_sized("left_dyn_param", kDefaultLeftDynParam, 10);
        ensure_double_array_sized("right_kine_param", kDefaultRightKineParam, 6);
        ensure_double_array_sized("right_dyn_param", kDefaultRightDynParam, 10);

    }
    
    hardware_interface::CallbackReturn MarvinHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        node_ = get_node();
        logger_ = get_node()->get_logger();
        RCLCPP_INFO(get_logger(), "Initializing Marvin Hardware Interface...");

        // Hardware error topic (heartbeat offline, etc.)
        hardware_error_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/Base_HardwareError", rclcpp::SystemDefaultsQoS());
        fsm_command_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", rclcpp::SystemDefaultsQoS());

        // 先把 hardware_parameters 的初始值灌入并声明成 ROS2 参数（便于后续动态修改）
        declare_node_parameters();
        // Debug mode (the whole run): everything at DEBUG.
        robot_log::applyLogMode();
        blackbox_ = robot_log::BlackboxRecorder::create("marvin_hardware");

        // Health of the read / write cycles: one line per window each. A cycle is late when it starts more than 1.5
        // periods after the previous one; the period is the component's rw_rate (0 = the rate of the controller
        // manager, which this plugin does not see: then no cycle is ever called late, the maxima are still reported).
        health_window_s_ = robot_log::healthWindowSeconds();
        const unsigned int rw_rate = get_hardware_info().rw_rate;
        late_period_ns_ = rw_rate > 0 ? static_cast<std::uint64_t>(1.5e9 / rw_rate) : UINT64_MAX;
        health_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(health_window_s_)),
            [this]() {
                const std::string name = get_node()->get_name();
                robot_log::logCycleHealth(get_logger(), name + ".read", health_window_s_, read_stats_.take(), "HW-013");
                robot_log::logCycleHealth(get_logger(), name + ".write", health_window_s_, write_stats_.take(), "HW-013");
            });

        // Get the number of joints from the hardware info
        size_t num_joints = params.hardware_info.joints.size();

        // 获取机器人配置参数
        std::string arm_config_raw = get_node_param("arm_type", std::string("LEFT"));
        std::string arm_config_normalized = normalizeString(arm_config_raw);

        if (arm_config_normalized == "LEFT")
        {
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }
        else if (arm_config_normalized == "RIGHT")
        {
            robot_arm_config_ = "RIGHT";
            robot_arm_index_ = ARM_RIGHT;
        }
        else if (arm_config_normalized == "DUAL")
        {
            robot_arm_config_ = "DUAL";
            robot_arm_index_ = ARM_DUAL;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown arm_type '%s', using default LEFT. Valid options: LEFT, RIGHT, DUAL", arm_config_raw.c_str());
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }

        // Control mode will be read from ROS2 parameter ctrl_mode ("POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE")
        // after declare_node_parameters() is called

        // 获取硬件连接参数
        device_ip_ = get_node_param("device_ip", std::string("192.168.1.190"));
        device_port_ = get_node_param("device_port", 8080);
        RCLCPP_INFO(get_logger(), "Device IP: %s, Port: %d", device_ip_.c_str(), device_port_);

        // 获取其他参数（将在 declare_node_parameters() 之后从节点参数读取）
   
        RCLCPP_INFO(get_logger(), "Initializing %zu joints", num_joints);

        // Resize all arrays based on the number of joints
        hw_position_commands_.resize(num_joints, 0.0);
        hw_velocity_commands_.resize(num_joints, 0.0);
        hw_position_states_.resize(num_joints, 0.0);
        hw_velocity_states_.resize(num_joints, 0.0);
        hw_effort_states_.resize(num_joints, 0.0);
        // Pre-allocate command buffer used in write() to avoid per-cycle allocations
        hw_commands_deg_buffer_.resize(num_joints, 0.0);

        // The arms system holds the arm joints only: the end effectors are components of their own
        for (const auto& joint : info_.joints)
        {
            joint_names_.push_back(joint.name);
        }
        RCLCPP_INFO(get_logger(), "Arm joints: %zu", joint_names_.size());

        // Flange ports as buses for the end effector and force sensor components (end_effector_hardware), on each arm.
        arm_buses_.clear();
        {
            const bool left = robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL;
            const bool right = robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL;
            if (left)
            {
                const auto channels = SdkChannels::left();
                arm_buses_.push_back(std::make_unique<ArmBuses>(device_ip_, "left", channels.send, channels.receive));
            }
            if (right)
            {
                const auto channels = SdkChannels::right();
                arm_buses_.push_back(std::make_unique<ArmBuses>(device_ip_, "right", channels.send, channels.receive));
            }
            for (auto& buses : arm_buses_)
            {
                std::string error;
                if (buses->provide(error))
                {
                    RCLCPP_INFO(get_logger(), "Flange buses of the %s arm: arm_rs485://%s/%s/port1|port2, arm_canfd://%s/%s",
                                buses->side().c_str(), device_ip_.c_str(), buses->side().c_str(), device_ip_.c_str(),
                                buses->side().c_str());
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Flange buses of the %s arm not provided: %s", buses->side().c_str(), error.c_str());
                }
            }
        }

        // Initialize hardware connection status
        hardware_connected_ = false;
        
        // 读取关节速度和加速度限制参数（如果不存在则自动声明）
        max_joint_speed_ = get_node_param("max_joint_speed", 10.0);
        max_joint_acceleration_ = get_node_param("max_joint_acceleration", 10.0);
        
        // Read ctrl_mode parameter and set robot_ctrl_mode_ string accordingly
        std::string ctrl_mode_raw = get_node_param("ctrl_mode", std::string("POSITION"));
        robot_ctrl_mode_ = normalizeString(ctrl_mode_raw);
        RCLCPP_INFO(get_logger(), "Robot control mode set to: %s (from ctrl_mode parameter: %s)", robot_ctrl_mode_.c_str(), ctrl_mode_raw.c_str());

        // 初始化阻抗参数缓存（供 setArmCtrlInternal 使用）
        joint_k_gains_ = get_node_param("joint_k_gains", kDefaultJointKGains);
        joint_d_gains_ = get_node_param("joint_d_gains", kDefaultJointDGains);
        cart_k_gains_  = get_node_param("cart_k_gains", kDefaultCartKGains);
        cart_d_gains_  = get_node_param("cart_d_gains", kDefaultCartDGains);

        RCLCPP_INFO(get_logger(), "Robot arm configuration: %s", robot_arm_config_.c_str());
        
        // Initialize kineParam and dynPara - read from node parameters (如果不存在则自动声明)
        leftkineParam_ = get_node_param("left_kine_param", kDefaultLeftKineParam);
        if (leftkineParam_.size() != 6) leftkineParam_.resize(6, 0.0);
        
        leftdynParam_ = get_node_param("left_dyn_param", kDefaultLeftDynParam);
        if (leftdynParam_.size() != 10) leftdynParam_.resize(10, 0.0);
        
        rightkineParam_ = get_node_param("right_kine_param", kDefaultRightKineParam);
        if (rightkineParam_.size() != 6) rightkineParam_.resize(6, 0.0);
        
        rightdynParam_ = get_node_param("right_dyn_param", kDefaultRightDynParam);
        if (rightdynParam_.size() != 10) rightdynParam_.resize(10, 0.0);
        
        param_callback_handle_ = node_->add_on_set_parameters_callback(
                std::bind(&MarvinHardware::paramCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Robot configuration parameters ready. Use 'ros2 param set' to change configuration dynamically.");
            
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface initialized successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

rcl_interfaces::msg::SetParametersResult
MarvinHardware::paramCallback(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Track if we need to apply configuration
    bool need_config_update = false;
    bool ctrl_mode_changed = false;
    std::string ctrl_mode = "";
    int drag_mode = -1;
    int cart_type = -1;
    double max_joint_speed = -1.0;
    double max_joint_acceleration = -1.0;
    std::vector<double> joint_k_gains;
    std::vector<double> joint_d_gains;
    std::vector<double> cart_k_gains;
    std::vector<double> cart_d_gains;

    for (const auto & param : params) {
        if (param.get_name() == "left_brake_release" || param.get_name() == "right_brake_release") {
            if (!hardware_connected_) {
                result.successful = false;
                result.reason = "Hardware not connected";
                return result;
            }
            const bool is_left = (param.get_name() == "left_brake_release");
            const bool arm_configured = is_left
                ? (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                : (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            if (!arm_configured) {
                RCLCPP_WARN(get_logger(), "%s ignored: arm not configured (arm_type=%s)",
                            param.get_name().c_str(), robot_arm_config_.c_str());
                continue;
            }

            if (robot_ctrl_mode_ != "POWER_OFF") {
                result.successful = false;
                result.reason = "ctrl_mode must be POWER_OFF before brake operation (current: "
                                  + robot_ctrl_mode_ + ")";
                RCLCPP_WARN(get_logger(), "Rejected %s: %s",
                            param.get_name().c_str(), result.reason.c_str());
                return result;
            }

            const bool release = param.as_bool();
            // 刹车请求收拢到 write() 单线程执行（SDK clear/send 单线程要求）。
            // 若控制循环未运行（无活动控制器），等待超时后兜底直发。
            PendingArmAction action;
            action.kind = PendingArmActionKind::kBrake;
            action.left = is_left;
            action.right = !is_left;
            action.brake_release = release;
            action.total_steps = 1;

            requestPendingAction(action);
            if (!waitPendingActionDone(500))
            {
                RCLCPP_WARN(get_logger(),
                            "%s arm brake: control loop not running, direct send fallback",
                            is_left ? "Left" : "Right");
                executePendingActionDirect(action);
            }

            RCLCPP_INFO(get_logger(), "%s arm brake: %s",
                        is_left ? "Left" : "Right",
                        release ? "BRAKE_RELEASE" : "BRAKE");
            continue;
        }

        // Other parameters require hardware connection
        if (!hardware_connected_) {
            result.successful = false;
            result.reason = "Hardware not connected";
            return result;
        }
        if (param.get_name() == "ctrl_mode") {
            std::string ctrl_mode_str = param.as_string();
            const int mode = ctrlModeStringToMode(ctrl_mode_str, get_logger());
            if (mode != 4) {
                const bool left_released = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                    && left_brake_released_;
                const bool right_released = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL)
                    && right_brake_released_;
                if (left_released || right_released) {
                    std::string arms;
                    if (left_released) arms += "left";
                    if (right_released) {
                        if (!arms.empty()) arms += "+";
                        arms += "right";
                    }
                    result.successful = false;
                    result.reason = "brake released on " + arms + " arm(s), only POWER_OFF is allowed";
                    RCLCPP_WARN(get_logger(), "Rejected ctrl_mode=%s: %s",
                                ctrl_mode_str.c_str(), result.reason.c_str());
                    return result;
                }
            }
            ctrl_mode = modeToCtrlModeString(mode);
            robot_ctrl_mode_ = ctrl_mode;
            if (mode != 4) last_active_ctrl_mode_ = ctrl_mode;
            need_config_update = true;
            ctrl_mode_changed = true;
            RCLCPP_INFO(get_logger(), "ctrl_mode parameter changed to: %s", ctrl_mode.c_str());
            if (mode != 4) {
                if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) left_brake_released_ = false;
                if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) right_brake_released_ = false;
            }
        }
        else if (param.get_name() == "drag_mode") {
            drag_mode = param.as_int();
            need_config_update = true;
            RCLCPP_INFO(get_logger(), "Drag mode parameter changed to: %d", drag_mode);
        }
        else if (param.get_name() == "joint_k_gains") {
            joint_k_gains = param.as_double_array();
            if (joint_k_gains.size() != 7) {
                result.successful = false;
                result.reason = "joint_k_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "joint_d_gains") {
            joint_d_gains = param.as_double_array();
            if (joint_d_gains.size() != 7) {
                result.successful = false;
                result.reason = "joint_d_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_k_gains") {
            cart_k_gains = param.as_double_array();
            if (cart_k_gains.size() != 7) {
                result.successful = false;
                result.reason = "cart_k_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_d_gains") {
            cart_d_gains = param.as_double_array();
            if (cart_d_gains.size() != 7) {
                result.successful = false;
                result.reason = "cart_d_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_type") {
            cart_type = param.as_int();
            need_config_update = true;
        }
        else if (param.get_name() == "max_joint_speed") {
            max_joint_speed = param.as_double();
            if (max_joint_speed <= 0.0) {
                result.successful = false;
                result.reason = "max_joint_speed must be > 0";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "max_joint_acceleration") {
            max_joint_acceleration = param.as_double();
            if (max_joint_acceleration <= 0.0) {
                result.successful = false;
                result.reason = "max_joint_acceleration must be > 0";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "left_dyn_param") {
            std::vector<double> new_left_dyn = param.as_double_array();
            if (new_left_dyn.size() != 10) {
                result.successful = false;
                result.reason = "left_dyn_param must have exactly 10 values";
                return result;
            }
            leftdynParam_ = new_left_dyn;
            RCLCPP_INFO(get_logger(), "Left dynParam updated via parameter");
            // Update the tool parameters on the hardware
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_LEFT);
            }
        }
        else if (param.get_name() == "right_dyn_param") {
            std::vector<double> new_right_dyn = param.as_double_array();
            if (new_right_dyn.size() != 10) {
                result.successful = false;
                result.reason = "right_dyn_param must have exactly 10 values";
                return result;
            }
            rightdynParam_ = new_right_dyn;
            RCLCPP_INFO(get_logger(), "Right dynParam updated via parameter");
            // Update the tool parameters on the hardware
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_RIGHT);
            }
        }
        else if (param.get_name() == "left_kine_param") {
            std::vector<double> new_left_kine = param.as_double_array();
            if (new_left_kine.size() != 6) {
                result.successful = false;
                result.reason = "left_kine_param must have exactly 6 values";
                return result;
            }
            leftkineParam_ = new_left_kine;
            RCLCPP_INFO(get_logger(), "Left kineParam updated via parameter");
        }
        else if (param.get_name() == "right_kine_param") {
            std::vector<double> new_right_kine = param.as_double_array();
            if (new_right_kine.size() != 6) {
                result.successful = false;
                result.reason = "right_kine_param must have exactly 6 values";
                return result;
            }
            rightkineParam_ = new_right_kine;
            RCLCPP_INFO(get_logger(), "Right kineParam updated via parameter");
        }
    }

    if (need_config_update) {
        const int mode = ctrl_mode.empty() ? -1 : ctrlModeStringToMode(ctrl_mode, get_logger());
        applyRobotConfiguration(mode, drag_mode, cart_type,
                              max_joint_speed, max_joint_acceleration,
                              joint_k_gains, joint_d_gains, cart_k_gains, cart_d_gains);

        if (ctrl_mode_changed) {
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_LEFT);
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_RIGHT);
            }
        }
    }

    return result;
}

void MarvinHardware::applyRobotConfiguration(int mode, int drag_mode, int cart_type,
                                              double max_joint_speed, double max_joint_acceleration,
                                              const std::vector<double>& joint_k_gains,
                                              const std::vector<double>& joint_d_gains,
                                              const std::vector<double>& cart_k_gains,
                                              const std::vector<double>& cart_d_gains)
{
    const auto send_and_sleep = []() {
        OnSetSend();
        usleep(100000);
    };
    const auto apply_drag_mode_if_needed = [&](bool update_left, bool update_right) {
        if (drag_mode >= 0) {
            if (update_left) {
                OnSetDragSpace_A(drag_mode);
            }
            if (update_right) {
                OnSetDragSpace_B(drag_mode);
            }
        }
    };
    const auto apply_joint_limits = [&](bool update_left, bool update_right) {
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        if (update_right) {
            OnSetJointLmt_B(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        send_and_sleep();
    };

    // Get current parameter values if not provided
    if (mode == -1) {
        const std::string ctrl_mode_str = get_node_param("ctrl_mode", std::string("POSITION"));
        mode = ctrlModeStringToMode(ctrl_mode_str, get_logger());
    }
    robot_ctrl_mode_ = modeToCtrlModeString(mode);
    if (drag_mode == -1) {
        drag_mode = get_node_param("drag_mode", -1);
    }
    if (cart_type == -1) {
        cart_type = get_node_param("cart_type", 2);
    }
    if (max_joint_speed <= 0.0) {
        max_joint_speed = get_node_param("max_joint_speed", 10.0);
    }
    if (max_joint_acceleration <= 0.0) {
        max_joint_acceleration = get_node_param("max_joint_acceleration", 10.0);
    }

    // Use provided KD or get from parameters or defaults
    std::vector<double> final_joint_k = (!joint_k_gains.empty()) ? joint_k_gains :
        (get_node_param("joint_k_gains", kDefaultJointKGains).size() == 7) ?
        get_node_param("joint_k_gains", kDefaultJointKGains) : kDefaultJointKGains;
    std::vector<double> final_joint_d = (!joint_d_gains.empty()) ? joint_d_gains :
        (get_node_param("joint_d_gains", kDefaultJointDGains).size() == 7) ?
        get_node_param("joint_d_gains", kDefaultJointDGains) : kDefaultJointDGains;
    std::vector<double> final_cart_k = (!cart_k_gains.empty()) ? cart_k_gains :
        (get_node_param("cart_k_gains", kDefaultCartKGains).size() == 7) ?
        get_node_param("cart_k_gains", kDefaultCartKGains) : kDefaultCartKGains;
    std::vector<double> final_cart_d = (!cart_d_gains.empty()) ? cart_d_gains :
        (get_node_param("cart_d_gains", kDefaultCartDGains).size() == 7) ?
        get_node_param("cart_d_gains", kDefaultCartDGains) : kDefaultCartDGains;

    // 同步缓存：供 setArmCtrlInternal 使用（避免只从 hardware_parameters 读一次导致无法动态更新）
    max_joint_speed_ = max_joint_speed;
    max_joint_acceleration_ = max_joint_acceleration;
    joint_k_gains_ = final_joint_k;
    joint_d_gains_ = final_joint_d;
    cart_k_gains_ = final_cart_k;
    cart_d_gains_ = final_cart_d;

    // Determine which arms to update (统一使用 arm_type / robot_arm_index_)
    bool update_left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
    bool update_right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);

    // Process based on selected mode
    if (mode == 1) {
        // Position mode
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(1);  // Position mode
        }
        if (update_right) {
            OnSetTargetState_B(1);  // Position mode
        }
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to position mode with speed=%.1f, acceleration=%.1f", 
                    max_joint_speed, max_joint_acceleration);
        
    } else if (mode == 2) {
        // Joint impedance mode
        double K[7], D[7];
        for (int i = 0; i < 7; i++) {
            K[i] = final_joint_k[i];
            D[i] = final_joint_d[i];
        }
        
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(3);  // Torque mode
            OnSetImpType_A(1);      // Joint impedance
            OnSetJointKD_A(K, D);
        }
        if (update_right) {
            OnSetTargetState_B(3);  // Torque mode
            OnSetImpType_B(1);      // Joint impedance
            OnSetJointKD_B(K, D);
        }
        apply_drag_mode_if_needed(update_left, update_right);
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to joint impedance mode with KD=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], speed=%.1f, acceleration=%.1f",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6], max_joint_speed, max_joint_acceleration);
        
    } else if (mode == 3) {
        // Cartesian impedance mode
        double K[7], D[7];
        for (int i = 0; i < 7; i++) {
            K[i] = final_cart_k[i];
            D[i] = final_cart_d[i];
        }
        
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(3);  // Torque mode
            OnSetImpType_A(2);      // Cartesian impedance
            OnSetCartKD_A(K, D, cart_type);
        }
        if (update_right) {
            OnSetTargetState_B(3);  // Torque mode
            OnSetImpType_B(2);      // Cartesian impedance
            // Right arm uses 6 values for cartesian
            double K_right[6], D_right[6];
            for (int i = 0; i < 6; i++) {
                K_right[i] = final_cart_k[i];
                D_right[i] = final_cart_d[i];
            }
            OnSetCartKD_B(K_right, D_right, cart_type);
        }
        apply_drag_mode_if_needed(update_left, update_right);
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to cartesian impedance mode with KD=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], speed=%.1f, acceleration=%.1f",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6], max_joint_speed, max_joint_acceleration);
    } else if (mode == 4) {
        // POWER_OFF: 下使能；A/B 分两次下发，避免同包卡顿
        if (update_left) {
            OnClearSet();
            OnSetTargetState_A(0);
            OnSetSend();
            usleep(100000);
        }
        if (update_right) {
            OnClearSet();
            OnSetTargetState_B(0);
            OnSetSend();
            usleep(100000);
        }
        RCLCPP_INFO(get_logger(), "Set to POWER_OFF mode (左右臂下使能)");
    } else if (mode == 5) {
        // PD 模式：关节阻抗 + 速度前馈（极低延时跟踪，主要适用于遥操场景）
        // 前置条件：控制器 robot.ini 中 JointPIDCtlType=1；关节轨迹速度 ≤180°/s
        const auto pd_k = get_node_param("pd_k_gains", kDefaultPDKGains);
        const auto pd_d = get_node_param("pd_d_gains", kDefaultPDDGains);
        const std::vector<double>& final_pd_k = (pd_k.size() == 7) ? pd_k : kDefaultPDKGains;
        const std::vector<double>& final_pd_d = (pd_d.size() == 7) ? pd_d : kDefaultPDDGains;
        const long pd_step = static_cast<long>(get_node_param("pd_control_period_ms", 5));

        double K[7], D[7];
        for (int i = 0; i < 7; i++) { K[i] = final_pd_k[i]; D[i] = final_pd_d[i]; }

        // 切换关节阻抗：速度/加速度设为最大（100，以免限制轨迹），设置刚度阻尼参数
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(100, 100);
            OnSetJointKD_A(K, D);
            OnSetTargetState_A(3);   // Torque 模式
            OnSetImpType_A(1);       // 关节阻抗
        }
        if (update_right) {
            OnSetJointLmt_B(100, 100);
            OnSetJointKD_B(K, D);
            OnSetTargetState_B(3);
            OnSetImpType_B(1);
        }
        send_and_sleep();

        // 开启 PD 速度前馈：step 为轨迹发送周期(ms)，0~20，<1 则不添加前馈（建议 5ms）
        // 仅 43 版及以上 SDK（SDK100343）提供 FX_OnSetVelEstStep；旧版退化为纯关节阻抗
#if SDK_VERSION >= 100343
        const char* pd_fb_suffix = "";
        OnClearSet();
        if (update_left) {
            FX_OnSetVelEstStep('A', pd_step);
        }
        if (update_right) {
            FX_OnSetVelEstStep('B', pd_step);
        }
        OnSetSend();
        usleep(200000);
#else
        const char* pd_fb_suffix = " (SDK<100343: vel feedforward unsupported, joint impedance only)";
#endif

        RCLCPP_INFO(get_logger(),
                    "Set to PD mode (joint impedance + vel feedforward): K=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], "
                    "D=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], velest_step=%ldms, speed/acc=100%s",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6],
                    D[0], D[1], D[2], D[3], D[4], D[5], D[6], pd_step, pd_fb_suffix);
    }
}


    void MarvinHardware::setArmCtrlInternal(int arm_index)
    {
        const bool is_left = (arm_index == ARM_LEFT);
        if (!is_left && arm_index != ARM_RIGHT)
        {
            RCLCPP_WARN(get_logger(), "setArmCtrlInternal called with invalid arm_index=%d", arm_index);
            return;
        }

        auto& kine = is_left ? leftkineParam_ : rightkineParam_;
        auto& dyn = is_left ? leftdynParam_ : rightdynParam_;

        // Ensure parameters are initialized
        if (kine.size() != 6) kine.resize(6, 0.0);
        if (dyn.size() != 10) dyn.resize(10, 0.0);

        const auto clear_err = is_left ? OnClearErr_A : OnClearErr_B;
        const auto set_target_state = is_left ? OnSetTargetState_A : OnSetTargetState_B;
        const auto set_tool = is_left ? OnSetTool_A : OnSetTool_B;
        const auto set_joint_kd = is_left ? OnSetJointKD_A : OnSetJointKD_B;
        const auto set_cart_kd = is_left ? OnSetCartKD_A : OnSetCartKD_B;
        const auto set_imp_type = is_left ? OnSetImpType_A : OnSetImpType_B;
        const auto set_joint_lmt = is_left ? OnSetJointLmt_A : OnSetJointLmt_B;

        OnClearSet();
        clear_err();
        OnSetSend();
        usleep(100000);

        if (robot_ctrl_mode_ == "POSITION")
        {
            OnClearSet();
            set_tool(kine.data(), dyn.data());
            set_target_state(1); // 1:position mode
            OnSetSend();
            usleep(100000);
        }
        else if (robot_ctrl_mode_ == "POWER_OFF")
        {
            OnClearSet();
            set_target_state(0); // 0:下使能 / servo off
            OnSetSend();
            usleep(100000);
        }
        else
        {
            OnClearSet();
            set_tool(kine.data(), dyn.data());
            set_target_state(3); // 3:torque mode

            if (robot_ctrl_mode_ == "JOINT_IMPEDANCE")
            {
                const bool valid = (joint_k_gains_.size() >= 7 && joint_d_gains_.size() >= 7);
                if (!valid)
                {
                    RCLCPP_WARN(get_logger(), "Joint impedance parameters size mismatch, using defaults");
                }
                const auto& K_src = valid ? joint_k_gains_ : kDefaultJointKGains;
                const auto& D_src = valid ? joint_d_gains_ : kDefaultJointDGains;
                double K[7];
                double D[7];
                for (size_t i = 0; i < 7; i++)
                {
                    K[i] = K_src[i];
                    D[i] = D_src[i];
                }
                set_joint_kd(K, D);
                set_imp_type(1);
            }
            else if (robot_ctrl_mode_ == "CART_IMPEDANCE")
            {
                const bool valid = (cart_k_gains_.size() >= 7 && cart_d_gains_.size() >= 7);
                if (!valid)
                {
                    RCLCPP_WARN(get_logger(), "Cartesian impedance parameters size mismatch, using defaults");
                }
                const auto& K_src = valid ? cart_k_gains_ : kDefaultCartKGains;
                const auto& D_src = valid ? cart_d_gains_ : kDefaultCartDGains;
                double K[7];
                double D[7];
                for (size_t i = 0; i < 7; i++)
                {
                    K[i] = K_src[i];
                    D[i] = D_src[i];
                }
                set_cart_kd(K, D, 2);
                set_imp_type(2);
            }

            OnSetSend();
            usleep(100000);
        }

        // set maximum speed and acceleration
        OnClearSet();
        set_joint_lmt(static_cast<int>(max_joint_speed_), static_cast<int>(max_joint_acceleration_));
        OnSetSend();
        usleep(100000);
    }

    hardware_interface::CallbackReturn MarvinHardware::on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring Marvin Hardware Interface...");


        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up Marvin Hardware Interface...");

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn MarvinHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating Marvin Hardware Interface...");

        if (hardware_connected_) {
            // Resume from POWER_OFF: re-engage brake if it was released (via existing brake params)
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                const bool param_released = node_->has_parameter("left_brake_release")
                    && node_->get_parameter("left_brake_release").as_bool();
                if (left_brake_released_ || param_released) {
                    node_->set_parameter(rclcpp::Parameter("left_brake_release", false));
                }
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                const bool param_released = node_->has_parameter("right_brake_release")
                    && node_->get_parameter("right_brake_release").as_bool();
                if (right_brake_released_ || param_released) {
                    node_->set_parameter(rclcpp::Parameter("right_brake_release", false));
                }
            }

            robot_ctrl_mode_ = last_active_ctrl_mode_;
            int mode = ctrlModeStringToMode(robot_ctrl_mode_, get_logger());
            RCLCPP_INFO(get_logger(), "Already connected, re-enabling arms (resume to %s)", robot_ctrl_mode_.c_str());
            applyRobotConfiguration(mode, -1, -1, -1.0, -1.0,
                                    std::vector<double>(), std::vector<double>(),
                                    std::vector<double>(), std::vector<double>());
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                setArmCtrlInternal(ARM_LEFT);
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL)
                setArmCtrlInternal(ARM_RIGHT);
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // First-time activation: connect to real hardware
        if (!connectToHardware())
        {
            RCLCPP_ERROR(get_logger(), "[HW-002] Failed to connect to Marvin hardware");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Read initial joint states from hardware with validation
        // Try multiple times to ensure data is ready and stable
        const int max_read_attempts = 10;
        const int read_interval_ms = 100;  // 100ms between attempts
        bool initial_read_success = false;
        bool all_zeros_detected = false;
        
        for (int attempt = 0; attempt < max_read_attempts; attempt++)
        {
            if (!readFromHardware(true))
            {
                RCLCPP_WARN(get_logger(), "Failed to read initial joint states (attempt %d/%d)", 
                           attempt + 1, max_read_attempts);
                usleep(read_interval_ms * 1000);
                continue;
            }
            
            // Validate that we didn't get all zeros (which likely indicates uninitialized data)
            bool all_zeros = true;
            for (size_t i = 0; i < hw_position_states_.size(); i++)
            {
                if (std::abs(hw_position_states_[i]) > 1e-6)  // Check if significantly non-zero
                {
                    all_zeros = false;
                    break;
                }
            }
            
            if (all_zeros)
            {
                all_zeros_detected = true;
                RCLCPP_WARN(get_logger(), "Initial joint positions are all zeros (attempt %d/%d), "
                           "this may indicate uninitialized hardware data. Retrying...", 
                           attempt + 1, max_read_attempts);
                usleep(read_interval_ms * 1000);
                continue;
            }
            
            initial_read_success = true;
            RCLCPP_INFO(get_logger(), "Successfully read initial joint states (attempt %d/%d)", 
                       attempt + 1, max_read_attempts);
            break;
        }
        
        if (!initial_read_success)
        {
            RCLCPP_ERROR(get_logger(), "[HW-003] Failed to read initial joint states after %d attempts", 
                        max_read_attempts);
            if (all_zeros_detected)
            {
                RCLCPP_ERROR(get_logger(), "Hardware appears to be returning all zeros. "
                           "This may indicate: 1) Hardware not ready, 2) Communication issue, "
                           "3) Hardware initialization problem. Please check hardware status.");
            }
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize commands with current positions
        for (size_t i = 0; i < hw_position_commands_.size(); i++)
        {
            hw_position_commands_[i] = hw_position_states_[i];
            hw_velocity_commands_[i] = hw_velocity_states_[i];
        }
        
        // Log initial positions for debugging
        // Handle both single arm (7 joints) and dual arm (14 joints) cases
        if (hw_position_states_.size() == 7)
        {
            // Single arm: 7 joints
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[0], hw_position_states_[1], hw_position_states_[2],
                       hw_position_states_[3], hw_position_states_[4], hw_position_states_[5],
                       hw_position_states_[6]);
        }
        else if (hw_position_states_.size() == 14)
        {
            // Dual arm: 14 joints (left 7 + right 7)
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad) - Left arm: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[0], hw_position_states_[1], hw_position_states_[2],
                       hw_position_states_[3], hw_position_states_[4], hw_position_states_[5],
                       hw_position_states_[6]);
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad) - Right arm: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[7], hw_position_states_[8], hw_position_states_[9],
                       hw_position_states_[10], hw_position_states_[11], hw_position_states_[12],
                       hw_position_states_[13]);
        }
        else
        {
            // Generic case: log all joints
            std::stringstream ss;
            ss << "Initial joint positions (rad): [";
            for (size_t i = 0; i < hw_position_states_.size(); i++)
            {
                ss << std::fixed << std::setprecision(3) << hw_position_states_[i];
                if (i < hw_position_states_.size() - 1)
                {
                    ss << ", ";
                }
            }
            ss << "]";
            RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        }
        OnClearSet();
        OnLogOff();
        OnLocalLogOff();
        OnSetSend();
        usleep(100000);

        // Initialize the tool (payload) parameters (all zeros until they are set)
        // This must be done before setArmCtrlInternal
        leftkineParam_.resize(6, 0.0);
        leftdynParam_.resize(10, 0.0);
        rightkineParam_.resize(6, 0.0);
        rightdynParam_.resize(10, 0.0);
        
        if (robot_arm_index_ == ARM_LEFT)
        {
            setArmCtrlInternal(ARM_LEFT);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            setArmCtrlInternal(ARM_RIGHT);
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            setArmCtrlInternal(ARM_LEFT);
            setArmCtrlInternal(ARM_RIGHT);
        }

        // Apply robot operation mode from ROS2 parameter ctrl_mode (default is "POSITION")
        // This reads the ctrl_mode parameter and applies the configuration
        // Convert robot_ctrl_mode_ string to integer mode locally
        int mode = -1;
        if (robot_ctrl_mode_ == "POSITION") {
            mode = 1;
        } else if (robot_ctrl_mode_ == "JOINT_IMPEDANCE") {
            mode = 2;
        } else if (robot_ctrl_mode_ == "CART_IMPEDANCE") {
            mode = 3;
        }
        applyRobotConfiguration(mode, -1, -1, -1.0, -1.0, 
                                std::vector<double>(), std::vector<double>(), 
                                std::vector<double>(), std::vector<double>());

        // Read DCSS once at init and log current arm state and error codes
        OnGetBuf(&frame_data_);
        if (robot_arm_index_ == ARM_LEFT)
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Left arm  CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[0].m_CurState, frame_data_.m_State[0].m_CmdState,
                        frame_data_.m_State[0].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Left arm vel/acc ratio: %d, %d",
                        frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Right arm CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[1].m_CurState, frame_data_.m_State[1].m_CmdState,
                        frame_data_.m_State[1].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Right arm vel/acc ratio: %d, %d",
                        frame_data_.m_In[1].m_Joint_Vel_Ratio, frame_data_.m_In[1].m_Joint_Acc_Ratio);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Left arm  CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[0].m_CurState, frame_data_.m_State[0].m_CmdState,
                        frame_data_.m_State[0].m_ERRCode);
            RCLCPP_INFO(get_logger(), "DCSS init: Right arm CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[1].m_CurState, frame_data_.m_State[1].m_CmdState,
                        frame_data_.m_State[1].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Left vel/acc: %d, %d; Right vel/acc: %d, %d",
                        frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio,
                        frame_data_.m_In[1].m_Joint_Vel_Ratio, frame_data_.m_In[1].m_Joint_Acc_Ratio);
        }

        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface activated successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Marvin Hardware Interface (power off, keep connection)...");


        // 下使能请求收拢到 write() 单线程执行；write() 未运行时兜底直发。
        if (hardware_connected_)
        {
            PendingArmAction action;
            action.kind = PendingArmActionKind::kPowerOff;
            action.left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
            action.right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            action.total_steps = (action.left ? 1 : 0) + (action.right ? 1 : 0);

            // 直发，不交给 write()：ros2_control 在这个回调期间不会再调用 write()，交给它的请求从不会被执行，等待只会
            // 耗掉超时（2026-11-17 两次真机停机，每次空等 2 秒）。直发和 write() 通过 sdk_access_mutex_ 互斥，
            // write() 恰好在跑也安全。
            executePendingActionDirect(action);
        }

        robot_ctrl_mode_ = "POWER_OFF";

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down Marvin Hardware Interface...");


        if (hardware_connected_) {
            // 急停序列（按需抱闸 + 下使能）收拢到 write() 单线程执行；未运行时兜底直发。
            PendingArmAction action;
            action.kind = PendingArmActionKind::kEmergencyStop;
            action.left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
            action.right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            action.total_steps = 4;

            // 直发，理由同 on_deactivate。
            executePendingActionDirect(action);
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_ERROR(get_logger(), "[HW-001] Error in Marvin Hardware Interface, emergency brake...");

        // 立即停发关节指令；急停序列由 write() 单线程执行（write() 已报错停止时兜底直发）。
        robot_ctrl_mode_ = "POWER_OFF";

        if (hardware_connected_) {
            PendingArmAction action;
            action.kind = PendingArmActionKind::kEmergencyStop;
            action.left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
            action.right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            action.total_steps = 4;

            // 直发，理由同 on_deactivate；on_error 是急停，更不该等。
            executePendingActionDirect(action);
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> MarvinHardware::on_export_state_interfaces()
    {
        RCLCPP_DEBUG(get_logger(), "Exporting state interfaces for %zu joints",
                     info_.joints.size());
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting state interfaces for %zu joints", joint_name_sz);

        for (size_t i = 0; i < joint_name_sz; i++)
        {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_POSITION,
                    &hw_position_states_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_velocity_states_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_EFFORT,
                    &hw_effort_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> MarvinHardware::on_export_command_interfaces()
    {
        RCLCPP_DEBUG(get_logger(), "Exporting command interfaces for %zu joints",
                     info_.joints.size());

        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting command interfaces for %zu joints",
                    joint_name_sz);
        for (size_t i = 0; i < joint_name_sz; i++)
        {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_POSITION,
                    &hw_position_commands_[i]));

            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_velocity_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type MarvinHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        robot_log::CycleTimer cycle_timer(read_stats_, last_read_begin_ns_, late_period_ns_);
        if (!hardware_connected_)
        {
            return hardware_interface::return_type::ERROR;
        }

        if (!readFromHardware(false))
        {
            return hardware_interface::return_type::ERROR;
        }
        for (auto& buses : arm_buses_) buses->pull();
        if (blackbox_)
        {
            blackbox_->record(robot_log::blackbox::kHwRead, hw_position_states_.data(), hw_position_states_.size());
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MarvinHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        robot_log::CycleTimer cycle_timer(write_stats_, last_write_begin_ns_, late_period_ns_);
        if (!hardware_connected_)
        {
            return hardware_interface::return_type::ERROR;
        }

        /// convert rad to degree (use pre-allocated buffer to avoid allocations in control loop)
        if (hw_commands_deg_buffer_.size() != hw_position_commands_.size())
        {
            hw_commands_deg_buffer_.resize(hw_position_commands_.size(), 0.0);
        }
        for (size_t i = 0; i < hw_position_commands_.size(); i++)
        {
            hw_commands_deg_buffer_[i] = radToDegree(hw_position_commands_[i]);
        }


        if (blackbox_)
        {
            blackbox_->record(robot_log::blackbox::kHwWrite, hw_position_commands_.data(), hw_position_commands_.size());
        }

        if (!writeToHardware(hw_commands_deg_buffer_))
        {
            return hardware_interface::return_type::ERROR;
        }
        for (auto& buses : arm_buses_) buses->push();  // after the arm frame: one end effector frame per arm
        return hardware_interface::return_type::OK;
    }

    bool MarvinHardware::connectToHardware()
    {
        RCLCPP_INFO(get_logger(), "Connecting to Marvin at %s:%d", device_ip_.c_str(), device_port_);

        // 解析 IP 地址
        std::vector<int> ip_parts;
        std::string ip_str = device_ip_;
        std::istringstream iss(ip_str);
        std::string token;
        while (std::getline(iss, token, '.'))
        {
            if (!token.empty())
            {
                ip_parts.push_back(std::stoi(token));
            }
        }
        
        if (ip_parts.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "Invalid IP address format: %s", device_ip_.c_str());
            return false;
        }
        
        // 尝试连接
        hardware_connected_ = OnLinkTo(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]);

        if (!hardware_connected_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to connect to Marvin hardware: OnLinkTo returned false");
            return false;
        }

        // 等待连接建立并清错
        usleep(100000);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        usleep(100000);

        // 验证连接：通过检查帧序列号是否在变化来确认连接是否真的有效
        // 参考 SDK demo 中的验证方法
        DCSS test_frame_data;
        int motion_tag = 0;
        int frame_update = 0;
        
        for (int i = 0; i < 5; i++)
        {
            OnGetBuf(&test_frame_data);
            int current_frame = test_frame_data.m_Out[0].m_OutFrameSerial;
            
            if (current_frame != 0 && frame_update != current_frame)
            {
                motion_tag++;
                frame_update = current_frame;
            }
            usleep(100000);  // 等待100ms
        }

        if (motion_tag > 0)
        {
            RCLCPP_INFO(get_logger(), "Successfully connected to Marvin hardware (verified by frame updates)");

            const long sdk_major = OnGetSDKVersion();
            char version_param[30] = "VERSION";
            long ctrl_version = 0;
            if (OnGetIntPara(version_param, &ctrl_version) == 0)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Marvin controller version: %ld (SDK major %ld, sub %ld)",
                    ctrl_version, sdk_major, ctrl_version % 100);
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Failed to read Marvin controller VERSION parameter (SDK major %ld)",
                    sdk_major);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to verify connection: No frame updates received from robot. Please check network connection and robot status.");
            hardware_connected_ = false;
            OnRelease();  // 释放连接
        }

        for (auto& buses : arm_buses_) buses->setConnected(hardware_connected_);
        return hardware_connected_;
    }

    void MarvinHardware::disconnectFromHardware()
    {
        RCLCPP_INFO(get_logger(), "Disconnecting from Marvin hardware");

        for (auto& buses : arm_buses_) buses->setConnected(false);
        OnRelease();

        hardware_connected_ = false;
        RCLCPP_INFO(get_logger(), "Disconnected from Marvin hardware");
    }

    bool MarvinHardware::readFromHardware(bool initial_frame)
    {
        OnGetBuf(&frame_data_);

        // Publish encoded error codes when non-zero:
        //  - Left arm:  0x600000 + m_ERRCode
        //  - Right arm: 0x610000 + m_ERRCode
        // read() returns ERROR when arm_ok is false → ResourceManager calls on_error().
        if (hardware_error_pub_)
        {
            auto publish_error_code = [&](int arm_index, int err_code)
            {
                if (err_code == 0)
                    return;

                int base = 0;
                const char* arm_name = "Unknown";
                if (arm_index == ARM_LEFT)
                {
                    base = 0x600000;
                    arm_name = "Left";
                }
                else if (arm_index == ARM_RIGHT)
                {
                    base = 0x610000;
                    arm_name = "Right";
                }
                else
                {
                    return;
                }

                const int encoded = base + err_code;

                std_msgs::msg::Int64 msg;
                msg.data = static_cast<int64_t>(encoded);
                hardware_error_pub_->publish(msg);

                RCLCPP_ERROR(get_logger(),
                             "Tianji robot error on %s arm: raw_err=%d, encoded_err=0x%X",
                             arm_name, err_code, encoded);
            };

            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
            {
                int err_code = frame_data_.m_State[robot_arm_index_].m_ERRCode;
                publish_error_code(robot_arm_index_, err_code);
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                int left_err_code = frame_data_.m_State[ARM_LEFT].m_ERRCode;
                int right_err_code = frame_data_.m_State[ARM_RIGHT].m_ERRCode;
                publish_error_code(ARM_LEFT, left_err_code);
                publish_error_code(ARM_RIGHT, right_err_code);
            }
        }
        
        if (initial_frame)
        {
            previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;
        }
        const auto copyArmFeedback = [&](int arm_idx, size_t dst_offset, const char* name_prefix) {
            for (size_t i = 0; i < 7; i++)
            {
                const double pos_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_Pos[i];
                const double vel_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_Vel[i];
                const double effort_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_SToq[i];

                const size_t dst_i = dst_offset + i;

                if (std::isnan(pos_raw) || std::isinf(pos_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu position is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_position_states_[dst_i]);
                }
                else
                {
                    hw_position_states_[dst_i] = degreeToRad(pos_raw);
                }

                if (std::isnan(vel_raw) || std::isinf(vel_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu velocity is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_velocity_states_[dst_i]);
                }
                else
                {
                    hw_velocity_states_[dst_i] = vel_raw;
                }

                if (std::isnan(effort_raw) || std::isinf(effort_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu effort is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_effort_states_[dst_i]);
                }
                else
                {
                    hw_effort_states_[dst_i] = effort_raw;
                }
            }
        };

        if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
        {
            copyArmFeedback(robot_arm_index_, 0, "Joint");
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            copyArmFeedback(ARM_LEFT, 0, "Left arm joint");
            copyArmFeedback(ARM_RIGHT, 7, "Right arm joint");
        }
        previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;

        const bool arm_ok = (robot_arm_index_ == ARM_DUAL)
            ? (frame_data_.m_State[ARM_LEFT].m_ERRCode == 0 &&
               frame_data_.m_State[ARM_RIGHT].m_ERRCode == 0)
            : (frame_data_.m_State[robot_arm_index_].m_ERRCode == 0);
        if (previous_message_frame_ - frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial < 2)
        {
            return arm_ok;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Missing more than 2 frames");
            return arm_ok;
        }
    }

    bool MarvinHardware::writeToHardware(std::vector<double>& hw_commands)
    {
        // 整段 SDK 访问（主帧 + 法兰口通道）统一在控制线程内串行执行；
        // 与生命周期兜底直发（executePendingActionDirect）通过 sdk_access_mutex_ 互斥。
        std::lock_guard<std::mutex> sdk_lock(sdk_access_mutex_);

        // ① 待执行动作（优先级最高，占本周期主帧槽）：刹车（含 POWER_OFF 模式）、
        //    下使能、急停。其余线程只置动作，由这里每周期执行一步。
        if (tryExecutePendingActionStep())
        {
            return true;
        }

        if (robot_ctrl_mode_ == "POWER_OFF")
        {
            return true;
        }

        // 主帧槽未释放（SDK 上一帧尚未发走）：暂时只累计连续 false 次数并跳过本周期，
        // 不因 OnClearSet false 触发硬件错误。
        if (!OnClearSet())
        {
            ++clear_set_consecutive_false_count_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_node()->get_clock(), 1000,
                "OnClearSet returned false for %llu consecutive cycle(s); continuing to skip write cycles",
                static_cast<unsigned long long>(clear_set_consecutive_false_count_));
            return true;
        }
        if (clear_set_consecutive_false_count_ > 0)
        {
            RCLCPP_WARN(
                get_logger(), "OnClearSet recovered after %llu consecutive false result(s)",
                static_cast<unsigned long long>(clear_set_consecutive_false_count_));
            clear_set_consecutive_false_count_ = 0;
        }

        bool result = true;

        // ② 主帧：机械臂关节指令（每帧）
        if (robot_arm_index_ == ARM_LEFT)
        {
            if (!left_brake_released_)
                result = OnSetJointCmdPos_A(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            if (!right_brake_released_)
                result = OnSetJointCmdPos_B(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            if (!left_brake_released_)
                result = OnSetJointCmdPos_A(hw_commands.data());
            if (!right_brake_released_)
                result = OnSetJointCmdPos_B(hw_commands.data() + 7) && result;
        }

        // OnSetSend 只提交机械臂主帧；法兰口通道由 ArmBuses 在 write() 末尾单独发送。
        // 提交失败（主帧槽仍被占用）同样计入连续失败，达到阈值才判定脱钩急停。
        if (OnSetSend())
        {
            main_frame_submit_failure_count_ = 0;
            return result;
        }
        ++main_frame_submit_failure_count_;
        if (main_frame_submit_failure_count_ >= kMainFrameBusyFatalThreshold)
        {
            RCLCPP_ERROR(get_logger(),
                         "SDK main-frame submit failed after %d consecutive cycles, declaring system disconnected",
                         main_frame_submit_failure_count_);
            return false;
        }
        RCLCPP_WARN(get_logger(),
                    "SDK main-frame submit failed (%d/%d), skipping this write cycle",
                    main_frame_submit_failure_count_, kMainFrameBusyFatalThreshold);
        return true;
    }

    const char* MarvinHardware::pendingActionKindName(PendingArmActionKind k)
    {
        switch (k)
        {
        case PendingArmActionKind::kBrake: return "BRAKE";
        case PendingArmActionKind::kPowerOff: return "POWER_OFF";
        case PendingArmActionKind::kEmergencyStop: return "EMERGENCY_STOP";
        default: return "NONE";
        }
    }

    bool MarvinHardware::tryExecutePendingActionStep()
    {
        std::lock_guard<std::mutex> lock(pending_action_mutex_);
        auto& a = pending_action_;
        if (a.kind == PendingArmActionKind::kNone)
        {
            return false;
        }

        const bool update_left = a.left && (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
        const bool update_right = a.right && (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
        const auto side_str = [&](bool l, bool r) -> const char* {
            return (l && r) ? "both" : (l ? "left" : (r ? "right" : "none"));
        };

        RCLCPP_INFO(get_logger(),
                    "[pending_action] step %d/%d kind=%s sides=%s robot_arm_config=%s",
                    a.step + 1, a.total_steps, pendingActionKindName(a.kind),
                    side_str(update_left, update_right), robot_arm_config_.c_str());

        switch (a.kind)
        {
        case PendingArmActionKind::kBrake:
        {
            const long sdk_value = a.brake_release ? 2 : 1;  // 2=松闸, 1=抱闸
            auto send_brake = [&](int arm_index) {
                char name[30] = {};
                std::snprintf(name, sizeof(name), "BRAK%d", arm_index);
                OnClearSet();
                if (a.brake_release)
                {
                    if (arm_index == ARM_LEFT) OnSetTargetState_A(0);
                    else OnSetTargetState_B(0);
                }
                OnSetIntPara(name, sdk_value);
                OnSetSend();
                RCLCPP_INFO(get_logger(),
                            "[pending_action] BRAKE %s: %s sent (sdk_value=%ld)",
                            arm_index == ARM_LEFT ? "left" : "right",
                            a.brake_release ? "release" : "engage", sdk_value);
            };
            if (update_left) send_brake(ARM_LEFT);
            if (update_right) send_brake(ARM_RIGHT);
            if (a.brake_release)
            {
                if (update_left) left_brake_released_ = true;
                if (update_right) right_brake_released_ = true;
            }
            else
            {
                if (update_left) left_brake_released_ = false;
                if (update_right) right_brake_released_ = false;
            }
            a.step = a.total_steps;
            break;
        }
        case PendingArmActionKind::kPowerOff:
            // 步骤 0：左臂下使能；步骤 1：右臂下使能
            if (a.step == 0)
            {
                if (update_left)
                {
                    OnClearSet();
                    OnSetTargetState_A(0);
                    OnSetSend();
                    RCLCPP_INFO(get_logger(), "[pending_action] POWER_OFF left arm: target state 0 sent");
                }
                a.step = update_right ? 1 : a.total_steps;
            }
            else if (a.step == 1 && update_right)
            {
                OnClearSet();
                OnSetTargetState_B(0);
                OnSetSend();
                RCLCPP_INFO(get_logger(), "[pending_action] POWER_OFF right arm: target state 0 sent");
                a.step = a.total_steps;
            }
            else
            {
                a.step = a.total_steps;
            }
            break;
        case PendingArmActionKind::kEmergencyStop:
            // 固定 4 步：抱闸左、抱闸右、下使能左、下使能右（未配置的一侧直接跳过）
            if (a.step == 0)
            {
                if (update_left) engageBrakeIfNeeded(ARM_LEFT);
                else RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step1: left not configured, skipped");
                a.step = 1;
            }
            else if (a.step == 1)
            {
                if (update_right) engageBrakeIfNeeded(ARM_RIGHT);
                else RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step2: right not configured, skipped");
                a.step = 2;
            }
            else if (a.step == 2)
            {
                if (update_left)
                {
                    OnClearSet();
                    OnSetTargetState_A(0);
                    OnSetSend();
                    RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step3: left arm target state 0 sent");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step3: left not configured, skipped");
                }
                a.step = 3;
            }
            else if (a.step == 3)
            {
                if (update_right)
                {
                    OnClearSet();
                    OnSetTargetState_B(0);
                    OnSetSend();
                    RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step4: right arm target state 0 sent");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "[pending_action] EMERGENCY_STOP step4: right not configured, skipped");
                }
                a.step = 4;
            }
            else
            {
                a.step = a.total_steps;
            }
            break;
        default:
            RCLCPP_WARN(get_logger(), "[pending_action] unknown kind=%d, aborting",
                        static_cast<int>(a.kind));
            a.step = a.total_steps;
            break;
        }

        if (a.step >= a.total_steps)
        {
            RCLCPP_INFO(get_logger(), "[pending_action] %s finished (steps=%d)",
                        pendingActionKindName(a.kind), a.step);
            a.kind = PendingArmActionKind::kNone;
            pending_action_cv_.notify_all();
        }
        return true;
    }

    void MarvinHardware::engageBrakeIfNeeded(int arm_index)
    {
        char name[30] = {};
        std::snprintf(name, sizeof(name), "BRAK%d", arm_index);
        long value = 0;
        OnGetIntPara(name, &value);
        RCLCPP_INFO(get_logger(), "[pending_action] %s brake status: value=%ld", name, value);
        if (value != 1)
        {
            OnClearSet();
            OnSetIntPara(name, 1);
            OnSetSend();
            RCLCPP_WARN(get_logger(), "Emergency brake engaged: %s", name);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Emergency brake already engaged: %s", name);
        }
    }

    void MarvinHardware::requestPendingAction(PendingArmAction action)
    {
        std::unique_lock<std::mutex> lock(pending_action_mutex_);
        // 等待上一个动作完成（如正在执行的急停），避免被后续请求覆盖。
        const bool waited = pending_action_cv_.wait_for(
            lock, std::chrono::milliseconds(2000),
            [this]() { return pending_action_.kind == PendingArmActionKind::kNone; });
        if (!waited && pending_action_.kind != PendingArmActionKind::kNone)
        {
            RCLCPP_WARN(get_logger(),
                        "[pending_action] previous action %s still in progress after 2s, overwriting it",
                        pendingActionKindName(pending_action_.kind));
        }
        pending_action_ = action;
        pending_action_cv_.notify_all();
        RCLCPP_INFO(get_logger(), "[pending_action] requested kind=%s left=%d right=%d brake_release=%d",
                    pendingActionKindName(action.kind), action.left ? 1 : 0,
                    action.right ? 1 : 0, action.brake_release ? 1 : 0);
    }

    bool MarvinHardware::waitPendingActionDone(int wait_ms)
    {
        std::unique_lock<std::mutex> lock(pending_action_mutex_);
        const bool done = pending_action_cv_.wait_for(
            lock, std::chrono::milliseconds(wait_ms),
            [this]() { return pending_action_.kind == PendingArmActionKind::kNone; });
        if (!done)
        {
            RCLCPP_WARN(get_logger(),
                        "[pending_action] wait timeout after %d ms (kind=%s still pending)",
                        wait_ms, pendingActionKindName(pending_action_.kind));
        }
        return done;
    }

    void MarvinHardware::executePendingActionDirect(PendingArmAction action)
    {
        // 兜底直发：write() 未运行（如 on_error 后控制循环已停）时，在互斥锁内同步执行全部步骤。
        RCLCPP_WARN(get_logger(), "[pending_action][fallback] direct execution of kind=%s left=%d right=%d",
                    pendingActionKindName(action.kind), action.left ? 1 : 0, action.right ? 1 : 0);
        std::lock_guard<std::mutex> sdk_lock(sdk_access_mutex_);
        const bool update_left = action.left && (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
        const bool update_right = action.right && (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);

        if (action.kind == PendingArmActionKind::kBrake)
        {
            const long sdk_value = action.brake_release ? 2 : 1;
            auto send_brake = [&](int arm_index) {
                char name[30] = {};
                std::snprintf(name, sizeof(name), "BRAK%d", arm_index);
                OnClearSet();
                if (action.brake_release)
                {
                    if (arm_index == ARM_LEFT) OnSetTargetState_A(0);
                    else OnSetTargetState_B(0);
                }
                OnSetIntPara(name, sdk_value);
                OnSetSend();
                usleep(100000);
                RCLCPP_INFO(get_logger(),
                            "[pending_action][fallback] BRAKE %s: %s sent (sdk_value=%ld)",
                            arm_index == ARM_LEFT ? "left" : "right",
                            action.brake_release ? "release" : "engage", sdk_value);
            };
            if (update_left) send_brake(ARM_LEFT);
            if (update_right) send_brake(ARM_RIGHT);
            if (action.brake_release)
            {
                if (update_left) left_brake_released_ = true;
                if (update_right) right_brake_released_ = true;
            }
            else
            {
                if (update_left) left_brake_released_ = false;
                if (update_right) right_brake_released_ = false;
            }
        }
        else if (action.kind == PendingArmActionKind::kPowerOff)
        {
            if (update_left)
            {
                OnClearSet();
                OnSetTargetState_A(0);
                OnSetSend();
                usleep(100000);
                RCLCPP_INFO(get_logger(), "[pending_action][fallback] POWER_OFF left arm: target state 0 sent");
            }
            if (update_right)
            {
                OnClearSet();
                OnSetTargetState_B(0);
                OnSetSend();
                usleep(100000);
                RCLCPP_INFO(get_logger(), "[pending_action][fallback] POWER_OFF right arm: target state 0 sent");
            }
        }
        else if (action.kind == PendingArmActionKind::kEmergencyStop)
        {
            if (update_left) engageBrakeIfNeeded(ARM_LEFT);
            if (update_right) engageBrakeIfNeeded(ARM_RIGHT);
            if (update_left)
            {
                OnClearSet();
                OnSetTargetState_A(0);
                OnSetSend();
                usleep(100000);
                RCLCPP_INFO(get_logger(), "[pending_action][fallback] EMERGENCY_STOP left arm: target state 0 sent");
            }
            if (update_right)
            {
                OnClearSet();
                OnSetTargetState_B(0);
                OnSetSend();
                usleep(100000);
                RCLCPP_INFO(get_logger(), "[pending_action][fallback] EMERGENCY_STOP right arm: target state 0 sent");
            }
        }
        RCLCPP_INFO(get_logger(), "[pending_action][fallback] %s direct execution finished",
                    pendingActionKindName(action.kind));

        // 动作已经执行完：槽里如果还是它，就清掉并通知。否则下一次 requestPendingAction 会把它当成「上一个动作还在
        // 进行」白等 2 秒再覆盖，write() 以后恢复运行时还会把这个过期的动作再执行一遍。
        std::lock_guard<std::mutex> lock(pending_action_mutex_);
        if (pending_action_.kind == action.kind)
        {
            pending_action_ = PendingArmAction{};
            pending_action_cv_.notify_all();
        }
    }

} // namespace marvin_hardware


PLUGINLIB_EXPORT_CLASS(marvin_hardware::MarvinHardware, hardware_interface::SystemInterface)
