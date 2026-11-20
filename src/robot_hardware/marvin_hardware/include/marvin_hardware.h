#ifndef MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_
#define MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_

#include <memory>
#include <string>
#include <robot_log/blackbox.hpp>
#include <robot_log/blackbox_types.hpp>
#include <robot_log/health.hpp>
#include <vector>
#include <unordered_map>
#include <thread>
#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"

// SDK include chain (Robot.h → TCPFileClient.h → FileOP.h) leaves #pragma pack(4) active
// without pop, shrinking sizeof() for all types below in this TU vs the other translation units.
#include "MarvinSDK.h"
#pragma pack()
#include <cmath>
#include "marvin_hardware/comm/arm_bus.h"
#include "marvin_hardware/comm/sdk_channels.h"

namespace marvin_hardware
{
    // Arm configuration constants
    constexpr int ARM_LEFT = 0;
    constexpr int ARM_RIGHT = 1;
    constexpr int ARM_DUAL = 2;

    class MarvinHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware)

        // Destructor
        virtual ~MarvinHardware(){};

        // Hardware interface lifecycle methods
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

        // Hardware interface methods
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
        rclcpp::Logger get_logger() const 
        { 
            return logger_.value();  // Safe to use value() since logger_ is initialized in on_init()
        }
        // Logger (initialized in on_init using get_node()->get_logger())
        std::optional<rclcpp::Logger> logger_;

        // Hardware parameters
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr hardware_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_pub_;
        std::string device_ip_;
        int device_port_ = 8080;
        std::string robot_arm_config_;  // "LEFT", "RIGHT", "DUAL"
        int robot_arm_index_ = 0;       // 0=LEFT, 1=RIGHT, 2=DUAL (在初始化时设定)
        std::string robot_ctrl_mode_;   // "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE", "POWER_OFF", "PD"
        std::string last_active_ctrl_mode_ = "POSITION";
        int previous_message_frame_ = 0;
        DCSS frame_data_;

        // OnClearSet false 暂时只计数并跳过本周期，不触发硬件错误；成功后清零并报告。
        std::uint64_t clear_set_consecutive_false_count_ = 0;
        // OnSetSend 连续失败仍保留阈值保护。
        int main_frame_submit_failure_count_ = 0;
        static constexpr int kMainFrameBusyFatalThreshold = 10;

        // --- SDK 单线程访问约束 ---
        // 底层 SDK 的 clear/send（主帧槽）与通道发送要求单线程串行访问。因此所有 SDK
        // 请求（机械臂主帧、法兰口总线）一律收拢到控制线程 write()
        // 按优先级每周期发送；其它线程（参数回调、生命周期）只置"待执行动作"，由 write()
        // 执行。sdk_access_mutex_ 用于 write() 的 SDK 段与生命周期兜底直发之间互斥
        // （write() 已停转的极端场景），保证 SDK 永不被并发访问。
        std::mutex sdk_access_mutex_;

        // 待执行的主帧动作（同一时刻至多一个；write() 每周期执行一步）
        enum class PendingArmActionKind : std::uint8_t
        {
            kNone = 0,
            kBrake,          // paramCallback：松闸/抱闸（仅在 POWER_OFF 模式允许）
            kPowerOff,       // on_deactivate：目标状态置 0（下使能）
            kEmergencyStop,  // on_error/on_shutdown：抱闸 + 目标状态置 0
        };
        struct PendingArmAction
        {
            PendingArmActionKind kind = PendingArmActionKind::kNone;
            bool left = false;          // 是否更新左臂
            bool right = false;         // 是否更新右臂
            bool brake_release = false; // kBrake：true=松闸(2)，false=抱闸(1)
            int step = 0;               // 已执行步数
            int total_steps = 0;        // 总步数（达此值即完成）
        };
        std::mutex pending_action_mutex_;
        std::condition_variable pending_action_cv_;
        PendingArmAction pending_action_;

        // write() 每周期执行一步待执行动作；返回 true 表示本周期主帧已被动作占用。
        bool tryExecutePendingActionStep();
        // 动作类型可读名（日志用）。
        static const char* pendingActionKindName(PendingArmActionKind k);
        // 其它线程请求执行动作；完成后通过 cv 通知。
        void requestPendingAction(PendingArmAction action);
        // 等待动作完成（wait_ms 超时返回 false，调用方据此走兜底直发）。
        bool waitPendingActionDone(int wait_ms);
        // 兜底：write() 未运行时直接串行执行全部步骤（需先获取 sdk_access_mutex_）。
        void executePendingActionDirect(PendingArmAction action);
        // 单步执行 kEmergencyStop 中"按需抱闸"（OnGetIntPara 检查 + 发送）。
        void engageBrakeIfNeeded(int arm_index);

        // Control parameters
        // 这些成员变量是 ROS2 参数的缓存值，命名与参数保持一致，减少歧义
        double max_joint_speed_;
        double max_joint_acceleration_;
        std::vector<double> joint_k_gains_;
        std::vector<double> joint_d_gains_;
        std::vector<double> cart_k_gains_;
        std::vector<double> cart_d_gains_;
        std::vector<double> leftdynParam_;
        std::vector<double> leftkineParam_;
        std::vector<double> rightdynParam_;
        std::vector<double> rightkineParam_;
        // Joint data storage
        std::vector<double> hw_position_commands_;
        std::vector<double> hw_velocity_commands_;
        std::vector<double> hw_position_states_;
        std::vector<double> hw_velocity_states_;
        std::vector<double> hw_effort_states_;
        // 复用的写入缓存（避免在 write() 控制回路中频繁分配/扩容导致抖动）
        std::vector<double> hw_commands_deg_buffer_;

        // Joint limits from URDF
        std::vector<double> position_lower_limits_;
        std::vector<double> position_upper_limits_;
        std::vector<double> velocity_limits_;
        std::vector<double> effort_limits_;

        // Connection status
        bool hardware_connected_;
        /** The flange ports as robot_comm buses (arm_rs485:// / arm_canfd://) for the end effector and force sensor
         *  components (end_effector_hardware): this driver does not run an end effector of its own. */
        std::vector<std::unique_ptr<ArmBuses>> arm_buses_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        std::vector<std::string> joint_names_;

        // Helper methods
        bool connectToHardware();
        void disconnectFromHardware();
        bool readFromHardware(bool initial_frame);
        bool writeToHardware(std::vector<double>& hw_commands);
        void setArmCtrlInternal(int arm_index);
        bool left_brake_released_ = false;
        bool right_brake_released_ = false;

        static double degreeToRad(const double degree)
        {                
            return degree * M_PI / 180.0;
        }

        static double radToDegree(const double rad)
        {
            return rad * 180.0 / M_PI;
        }
        
        // 辅助方法：获取节点参数，如果不存在则自动声明
        template<typename T>
        T get_node_param(const std::string& name, const T& default_val)
        {
            if (!node_->has_parameter(name)) {
                node_->declare_parameter<T>(name, default_val);
            }
            // rclcpp::Parameter 使用 get_value<T>() 获取强类型值
            return node_->get_parameter(name).get_value<T>();
        }
        
        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);
        void applyRobotConfiguration(int mode, int drag_mode, int cart_type,
                                    double max_joint_speed, double max_joint_acceleration,
                                    const std::vector<double>& joint_k_gains,
                                    const std::vector<double>& joint_d_gains,
                                    const std::vector<double>& cart_k_gains,
                                    const std::vector<double>& cart_d_gains);
        void declare_node_parameters();
        // Health of the read / write cycles (src/robot_utils/README.md): filled by read() and write(), reported by health_timer_.
        // What was read from and written to the motors in each cycle, for the diagnosis (src/robot_utils/README.md).
        std::shared_ptr<robot_log::BlackboxRecorder> blackbox_;
        robot_log::CycleStats read_stats_;
        robot_log::CycleStats write_stats_;
        std::uint64_t last_read_begin_ns_{0};
        std::uint64_t last_write_begin_ns_{0};
        std::uint64_t late_period_ns_{0};   // 0 until the component's rate is known: then nothing is recorded
        double health_window_s_{10.0};
        rclcpp::TimerBase::SharedPtr health_timer_;

    };
} // namespace marvin_hardware

#endif  // MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_
