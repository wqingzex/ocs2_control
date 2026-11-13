#pragma once
#include <hardware_interface/system_interface.hpp>
#include <robot_log/flight.hpp>
#include <robot_log/flight_types.hpp>
#include <robot_log/health.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>
#include <cmath>
#include <std_msgs/msg/int64.hpp>

#include "juxie_hardware/juxie_protocol.h"
#include "robot_comm/can.h"

namespace juxie_hardware {
/// Juxie joint modules (waist, head) over CAN FD. The CAN link is a robot_comm::CanBus, the frames are built and
/// read by juxie_hardware::protocol; this class is the ros2_control side.
///
/// Parameters: motor_ids (required), control_period_ms, max_position_step_nct, max_position_accel_nct, and the
/// shared CAN parameters of robot_comm::parseCanConfig: can_interface (default can0), can_bitrate (1000000),
/// can_data_bitrate (5000000, Juxie needs CAN FD), can_restart_ms (100), can_manage_interface (true: the driver
/// runs `sudo ip link set ...`; false: the interface is already up, e.g. set up by systemd-networkd).
/// stats_log_period_s (default 0): every N seconds log how steadily the control thread and the bus run (wake-up
/// lateness of the control loop, late cycles, failed sends, CAN errors, motors that did not answer a read); 0 logs
/// the totals once, when the driver is deactivated.
class JxHardware : public hardware_interface::SystemInterface {
public:
    JxHardware();

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
        on_export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface::SharedPtr> 
        on_export_command_interfaces() override;
    
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /// Replace the CAN bus (tests and benches). Call before on_init; the interface is then not configured.
    void setBusForTesting(std::unique_ptr<robot_comm::CanBus> bus);

private:
    // 配置参数
    robot_comm::CanConfig can_config_;                       // CAN 接口配置（接口名、速率、是否由驱动配置）
    std::vector<uint8_t> motor_ids_;   // 电机ID列表（最多8个）
    float control_period_ms_;            // 控制周期（默认1ms）
    // CSP 单帧最大位置增量（nct）。文档写 1ms 下 >100 报错；实测 2ms 下约 100+ 也会报，
    // 按每帧绝对值限幅，默认 90。
    int32_t max_position_step_nct_ = 25;
    // 相邻两帧指令速度(delta)的最大变化量（nct/cycle²）；0 表示关闭加速度限幅。
    int32_t max_position_accel_nct_ = 5;

    // What was read from and written to the motors in each cycle, for the diagnosis (docs/日志.md).
    std::shared_ptr<robot_log::FlightRecorder> flight_;

    // Health of the read / write cycles (docs/日志.md): filled by read() and write(), reported once per window by
    // health_timer_ while the hardware is active.
    robot_log::CycleStats read_stats_;
    robot_log::CycleStats write_stats_;
    std::uint64_t last_read_begin_ns_{0};
    std::uint64_t last_write_begin_ns_{0};
    std::uint64_t late_period_ns_{0};   // 0: the rate is not known, nothing is called late
    double health_window_s_{10.0};
    rclcpp::TimerBase::SharedPtr health_timer_;

    // 状态和命令存储
    std::vector<double> joint_positions_;     // 实际位置（弧度）
    std::vector<double> joint_velocities_;    // 实际速度（弧度/秒）
    std::vector<double> joint_efforts_;       // 实际力矩
    std::vector<double> joint_position_commands_; // 位置命令（弧度）
    std::vector<int16_t> raw_position_commands_;  // 位置命令（电机int16计数）

    // 限速/限加速追赶：始终朝最新目标追
    std::vector<int16_t> target_raw_position_commands_; // 最新目标（来自 ROS2 控制器）
    std::vector<int16_t> last_sent_raw_commands_;       // 上一次实际发送的命令
    std::vector<int32_t> last_sent_deltas_;             // 上一次实际发送的位置增量（速度）

    // 控制标志
    bool is_first_command_ = true;    // 标记是否是第一次发送命令
    std::atomic<bool> running_;       // 线程运行标志

    // CAN FD通信相关
    std::unique_ptr<robot_comm::CanBus> bus_;
    std::unique_ptr<robot_comm::CanInterfaceConfigurator> configurator_;
    bool bus_injected_ = false;         // 总线由 setBusForTesting 给定：不配置接口
    std::mutex state_mutex_;            // 状态数据互斥锁

    // 控制线程相关
    std::thread control_thread_;        // 周期性发送多控帧的线程
    // 异常发布相关
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr error_pub_;
    // 在严格检查阶段首次锁定反馈帧长度，后续沿用该长度做校验
    uint8_t locked_feedback_len_ = 0;

    // 控制线程的周期统计（线程停止时打印一次，用来判断稳定性）
    struct LoopStats {
        uint64_t cycles = 0;
        uint64_t late_cycles = 0;      // 醒来时已经晚了半个周期以上
        uint64_t send_failures = 0;
        double max_lateness_us = 0.0;
        double sum_lateness_us = 0.0;
    };
    LoopStats loop_stats_;
    double stats_log_period_s_ = 0.0;                       // 0: 只在停止时打印一次
    std::atomic<uint64_t> read_calls_{0};                   // read() 次数（非严格模式）
    std::atomic<uint64_t> missed_feedback_{0};              // 某次 read() 里没有收到反馈的电机数之和

    // 辅助函数
    bool initCanFd();                   // 配置并打开 CAN 总线
    bool enableMotors();                // 使能电机
    void controlLoop();                 // 控制线程循环
    bool readMotorStates(bool strict_check); // 读取所有电机状态
    void publishFault(uint8_t motor_id, uint16_t fault_code, bool known);
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    void parseAndReportEmergencyFrame(uint8_t motor_id, const robot_comm::CanFrame& frame);
    bool sendMultiMotorCommand();       // 发送多控帧
    bool sendSyncFrame();               // 发送同步帧
};
} // namespace juxie_hardware
