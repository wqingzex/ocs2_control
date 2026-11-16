#include "end_effector_hardware/hands/linkerhand/linkerhand_rs485_hardware.h"
#include "end_effector_hardware/hands/dexterous_hand_wrapper_template.h"
#include "robot_comm/modbus.h"
#include "robot_comm_modbus/libmodbus_rtu_master.h"
#include "end_effector_hardware/common/serial_notices.h"
#include "end_effector_hardware/gripper_protocol/utils/ModbusConfig.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <string>

using namespace gripper_protocol;

namespace end_effector_hardware
{
    hardware_interface::CallbackReturn LinkerHandRs485Hardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params
    )
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 保存硬件参数（用于后续传递给hand_->initialize）
        // 注意：关节限位从ModbusConfig中读取，不再需要robot_description
        hardware_parameters_ = params.hardware_info.hardware_parameters;
        
        // 调试：打印所有硬件参数
        RCLCPP_INFO(get_node()->get_logger(), "Hardware parameters received:");
        for (const auto& [key, value] : hardware_parameters_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  %s = %s", key.c_str(), value.c_str());
        }
        
        // 读取配置参数
        loadParameters(params.hardware_info.hardware_parameters);

        // 检测灵巧手关节（需要7个关节）
        std::vector<std::string> hand_joint_names = DexterousHandBase::detectHandJoints(
            params.hardware_info.joints
        );

        // 调试信息：打印所有关节名称
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Total joints in hardware info: %zu",
            params.hardware_info.joints.size()
        );
        for (const auto& joint : params.hardware_info.joints)
        {
            RCLCPP_INFO(
                get_node()->get_logger(),
                "  Joint: %s",
                joint.name.c_str()
            );
        }
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Detected hand joints: %zu",
            hand_joint_names.size()
        );
        for (const auto& joint_name : hand_joint_names)
        {
            RCLCPP_INFO(
                get_node()->get_logger(),
                "  Hand joint: %s",
                joint_name.c_str()
            );
        }

        // LinkerHand L6 only (the only LinkerHand model in the robot descriptions): 6 joints, hand_type containing L6.
        std::string product_type = hand_type_;
        std::transform(product_type.begin(), product_type.end(), product_type.begin(), ::toupper);
        if (hand_joint_names.size() != L6DexterousHandWrapper::JOINT_COUNT || product_type.find("L6") == std::string::npos)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Only the LinkerHand L6 is supported (6 joints, hand_type linkerhand_l6); got hand_type=%s with %zu joints",
                hand_type_.c_str(),
                hand_joint_names.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 创建 Modbus 通信器
        // 使用 ModbusConfig 中的默认参数（固定值）
        modbus_communicator_ = std::make_unique<robot_comm::LibmodbusRtuMaster>(
            serial_port_,
            ModbusConfig::DexterousHand::DEFAULT_BAUDRATE,      // 固定波特率115200
            modbus_slave_id_,
            ModbusConfig::DexterousHand::DEFAULT_PARITY,        // 无校验 'N'
            ModbusConfig::DexterousHand::DEFAULT_DATA_BITS,    // 数据位8
            ModbusConfig::DexterousHand::DEFAULT_STOP_BITS,    // 停止位1
            true                                                // FTDI latency_timer -> 1 ms when supported
        );
        modbus_communicator_->setTimeouts(response_timeout_ms_, byte_timeout_ms_);

        hand_ = std::make_unique<L6DexterousHandWrapper>(
            get_node()->get_logger(),
            get_node()->get_clock(),
            hand_joint_names
        );
        RCLCPP_INFO(get_node()->get_logger(), "Creating L6 dexterous hand (6-DOF)");

        tool_torque_parameter_name_ = hand_side_ == "left" ?
            "left_tool_torque" : "right_tool_torque";
        tool_velocity_parameter_name_ = hand_side_ == "left" ?
            "left_tool_velocity" : "right_tool_velocity";
        const auto node = get_node();
        if (!node->has_parameter(tool_torque_parameter_name_))
        {
            node->declare_parameter<double>(tool_torque_parameter_name_, 1.0);
        }
        if (!node->has_parameter(tool_velocity_parameter_name_))
        {
            node->declare_parameter<double>(tool_velocity_parameter_name_, 1.0);
        }
        tool_torque_.store(node->get_parameter(tool_torque_parameter_name_).as_double());
        tool_velocity_.store(node->get_parameter(tool_velocity_parameter_name_).as_double());
        if (tool_torque_.load() < 0.0 || tool_torque_.load() > 1.0 ||
            tool_velocity_.load() < 0.0 || tool_velocity_.load() > 1.0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "LinkerHand tool torque/velocity must be in [0, 1]");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hand_->setToolRatios(tool_torque_.load(), tool_velocity_.load());
        parameter_callback_handle_ = node->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                double torque = tool_torque_.load();
                double velocity = tool_velocity_.load();
                for (const auto& parameter : parameters)
                {
                    if (parameter.get_name() != tool_torque_parameter_name_ &&
                        parameter.get_name() != tool_velocity_parameter_name_) continue;
                    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                        parameter.as_double() < 0.0 || parameter.as_double() > 1.0)
                    {
                        result.successful = false;
                        result.reason = parameter.get_name() + " must be a double in [0, 1]";
                        return result;
                    }
                    if (parameter.get_name() == tool_torque_parameter_name_) torque = parameter.as_double();
                    else velocity = parameter.as_double();
                }
                tool_torque_.store(torque);
                tool_velocity_.store(velocity);
                if (hand_) hand_->setToolRatios(torque, velocity);
                return result;
            });

        RCLCPP_INFO(
            get_node()->get_logger(),
            "LinkerHandRs485Hardware initialized:"
        );
        const std::string product_name = "L6";

        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Hand type: %s (Product: %s, %zu-DOF)",
            hand_type_.c_str(),
            product_name.c_str(),
            hand_joint_names.size()
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Serial port: %s",
            serial_port_.c_str()
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Modbus ID: 0x%02X (%s)",
            modbus_slave_id_,
            (modbus_slave_id_ == 0x27) ? "RIGHT_HAND" : "LEFT_HAND"
        );
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  Joints: %zu",
            hand_joint_names.size()
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LinkerHandRs485Hardware::on_activate(
        const rclcpp_lifecycle::State& /* previous_state */
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating LinkerHandRs485Hardware...");

        // 连接 Modbus
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Attempting to connect Modbus: port=%s, slave_id=0x%02X",
            serial_port_.c_str(),
            modbus_slave_id_
        );
        
        if (!connect_master_and_log(*modbus_communicator_))
        {
            std::string error_msg = modbus_communicator_->getLastError();
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to connect Modbus: %s",
                error_msg.c_str()
            );
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Please check: 1) Serial port exists and has correct permissions, 2) Device is powered on, 3) Serial port is not used by another process"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ Modbus connected");

        // 初始化灵巧手
        // 使用保存的硬件参数（包含设备力矩/速度比例等参数）
        // 关节限位从ModbusConfig中读取（参考Jodell RG75的实现方式）
        if (!hand_->initialize(
            modbus_communicator_.get(),
            hardware_parameters_
        ))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to initialize dexterous hand"
            );
            modbus_communicator_->disconnect();
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ Dexterous hand initialized");

        // 启动后台读取线程
        hand_->startBackgroundReading();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LinkerHandRs485Hardware::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */
    )
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating LinkerHandRs485Hardware...");

        if (hand_)
        {
            // 停止后台读取线程
            hand_->stopBackgroundReading();
            hand_->shutdown();
        }

        if (modbus_communicator_)
        {
            modbus_communicator_->disconnect();
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ LinkerHandRs485Hardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
    LinkerHandRs485Hardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        if (hand_ && hand_->hasHand())
        {
            hand_->exportStateInterfaces(state_interfaces);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr>
    LinkerHandRs485Hardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        if (hand_ && hand_->hasHand())
        {
            hand_->exportCommandInterfaces(command_interfaces);
        }

        return command_interfaces;
    }

    hardware_interface::return_type LinkerHandRs485Hardware::read(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& period
    )
    {
        if (!hand_ || !hand_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // 根据控制循环的周期设置后台线程的循环间隔（只设置一次）
        if (hand_->isBackgroundReadingActive())
        {
            hand_->updateBackgroundReadingInterval(period);
        }

        hand_->publishFeedbackToStateInterfaces();

        // 检查后台读取线程是否在运行
        if (!hand_->isBackgroundReadingActive())
        {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                2000,
                "Background reading thread is not active"
            );
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LinkerHandRs485Hardware::write(
        const rclcpp::Time& /* time */,
        const rclcpp::Duration& /* period */
    )
    {
        if (!hand_ || !hand_->isInitialized())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Publish one coherent command snapshot. The background thread keeps only
        // the newest snapshot and performs write-before-read RTU communication.
        hand_->publishCommands();
        return hardware_interface::return_type::OK;
    }

    void LinkerHandRs485Hardware::loadParameters(
        const std::unordered_map<std::string, std::string>& params
    )
    {
        // 读取配置参数的辅助函数
        const auto get_param = [&params](const std::string& name, const std::string& default_val)
        {
            auto it = params.find(name);
            return (it != params.end()) ? it->second : default_val;
        };

        // 读取手部类型
        hand_type_ = get_param("hand_type", "simple_dexterous_hand");

        // 读取手部配置（left/right）
        hand_side_ = get_param("hand_side", "right");
        
        // 根据手部配置设置 Modbus ID
        std::string side_lower = hand_side_;
        std::transform(side_lower.begin(), side_lower.end(), side_lower.begin(), ::tolower);
        
        if (side_lower == "left" || side_lower == "left_hand")
        {
            modbus_slave_id_ = ModbusConfig::DexterousHand::LEFT_HAND_SLAVE_ID;  // 左手
        }
        else
        {
            modbus_slave_id_ = ModbusConfig::DexterousHand::RIGHT_HAND_SLAVE_ID;  // 右手（默认）
        }

        // 读取串口配置
        serial_port_ = get_param("serial_port", "/dev/ttyUSB0");
        try
        {
            response_timeout_ms_ = std::stoi(get_param("response_timeout_ms", "20"));
            byte_timeout_ms_ = std::stoi(get_param("byte_timeout_ms", "5"));
            if (response_timeout_ms_ <= 0 || byte_timeout_ms_ <= 0)
            {
                throw std::out_of_range("timeouts must be positive");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid LinkerHand timeout parameter: %s", e.what());
            response_timeout_ms_ = 20;
            byte_timeout_ms_ = 5;
        }
        
        // 波特率固定为115200，不支持修改
        // 其他串口参数也固定：停止位1, 数据位8, 校验位N
    }

    bool LinkerHandRs485Hardware::createHand(const std::vector<hardware_interface::ComponentInfo>& /* joints */)
    {
        // 已在 on_init 中创建
        return true;
    }
} // namespace end_effector_hardware

// 导出插件
PLUGINLIB_EXPORT_CLASS(
    end_effector_hardware::LinkerHandRs485Hardware,
    hardware_interface::SystemInterface
)
