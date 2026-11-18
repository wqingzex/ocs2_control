//
// Created for Arms ROS2 Control - XrInputHandler
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <robot_log/blackbox.hpp>
#include <robot_log/blackbox_types.hpp>
#include <robot_log/health.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "target_generator/TargetGenerator.h"
#include "target_generator/xr_alignment.h"
#include "target_generator/target_guard.h"
#include <optional>
#include "robot_controller_core/FSMCommandPublisher.h"

namespace target_generator
{
    /**
     * XrInputHandler - XR 手柄输入处理
     *
     * 按键（事件码来自 xr_teleop 的 /teleop/controller_state，按键表见 docs/使用手册.md 第 6.1 节）：
     * - 右A：FSM 前进（HOME→HOLD→OCS2）
     * - 左X：FSM 后退（OCS2/MOVEJ→HOLD→HOME，HOME 下切换 REST）
     * - 右摇杆按下：UPDATE（双臂跟随手柄）↔ STORAGE（停止），仅 OCS2；离开 OCS2 自动回到 STORAGE
     * 扳机按比例控制手/夹爪开合（/teleop/trigger_values → <controller>/target_percent）。
     */
    class XrInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetGenerator TargetGenerator指针
         * @param pub_left_target 外部传入的左臂目标位姿发布器（统一管理）
         * @param pub_right_target 外部传入的右臂目标位姿发布器（统一管理）
         * @param handControllers 手部/夹爪控制器名称列表（名字里带 left / right，用于映射扳机）
         * @param xr_pose_scale 手柄位姿位置缩放因子（左右共用）
         * @param reference_link 参考link名称（例如 head_link2），用于XR/头显关联
         */
        XrInputHandler(
            rclcpp::Node::SharedPtr node,
            TargetGenerator* targetGenerator,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
            const std::vector<std::string>& handControllers,
            double xr_pose_scale,
            const std::string& reference_link);

        ~XrInputHandler() = default;

        /**
         * 启用XR控制
         */
        void enable();

        /**
         * 禁用XR控制
         */
        void disable();

        /**
         * 检查XR控制是否启用
         * @return true如果启用，false否则
         */
        bool isEnabled() const { return enabled_.load(); }

        /// 头显 yaw 对齐（xr_align_head_yaw）：true 时位移按头显水平朝向解释（操作员正前方 = 机器人 +X），
        /// 不受房间原点和手柄倾角影响；false 沿用手柄基准姿态（旧行为）。
        void setAlignHeadYaw(bool enable);

        /**
         * 检查节点是否存在
         * @param node ROS节点指针
         * @param targetNodeName 目标节点名称
         * @return true如果节点存在，false否则
         */
        bool checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName);

        /**
         * FSM 状态回调（/fsm_state，控制器实际所处的状态）
         * 用状态而不是命令：命令可能被控制器忽略（例如 MOVEJ 下发 HOME），按命令判断会和真实状态不一致。
         * @param msg 1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ；其他值忽略
         */
        void fsmStateCallback(std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 机器人左臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotLeftPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 机器人右臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotRightPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    private:
        // 按键事件码（与 xr_teleop/button_event_encoder.hpp 的 event:: 一致；0 每帧都会收到，不是按键）
        static constexpr int32_t kEventNone = 0;
        static constexpr int32_t kEventFsmForward = 1;   // 右A
        static constexpr int32_t kEventFsmBack = 2;      // 左X
        static constexpr int32_t kEventToggleFollow = 3; // 右摇杆按下

        // FSM 状态 / /fsm_command 命令
        static constexpr int32_t kFsmHome = 1;
        static constexpr int32_t kFsmHold = 2;
        static constexpr int32_t kFsmOcs2 = 3;
        static constexpr int32_t kFsmMoveJ = 4;
        static constexpr int32_t kFsmRestToggle = 100;  // HOME 下切换 HOME ↔ REST（状态不变）

        /**
         * XR左臂pose回调函数
         * @param msg XR pose消息
         */
        void xrLeftCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * XR右臂pose回调函数
         * @param msg XR pose消息
         */
        void xrRightCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * XR头显pose回调函数
         * @param msg XR 头显 pose 消息
         */
        void xrHeadCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * 扳机模拟量回调函数（合并处理左右扳机）
         * @param msg Twist消息，linear.x 表示左扳机拉动比例，angular.x 表示右扳机拉动比例
         */
        void triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        /**
         * 按钮事件回调函数（处理按钮事件数字）
         * @param msg Int32消息：0=无事件, 1=右A, 2=左X, 3=右摇杆按下；其他值忽略
         */
        void processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg);

        /// 右摇杆按下：在 OCS2 中切换 UPDATE（记录基准，双臂跟随）/ STORAGE（停止）
        void toggleFollow();

        /// UPDATE 模式下一侧手臂的目标：基准机器人位姿 + 手柄自记录基准以来的位移和转动，变化明显才发布
        void followArm(const std::string& arm,
                       const Eigen::Vector3d& xrPos, const Eigen::Quaterniond& xrOri,
                       const Eigen::Vector3d& xrBasePos, const Eigen::Quaterniond& xrBaseOri,
                       const Eigen::Quaterniond& xrBaseFrame,
                       const Eigen::Vector3d& robotBasePos, const Eigen::Quaterniond& robotBaseOri,
                       const Eigen::Vector3d& robotCurrentPos, std::optional<Eigen::Vector3d>& lastAcceptedPos,
                       Eigen::Vector3d& prevPos, Eigen::Quaterniond& prevOri);

        /**
         * 从hand_controllers参数中提取左右控制器名称
         * @param hand_controllers 控制器名称列表
         */
        void detectGripperControllers(const std::vector<std::string>& hand_controllers);

        /**
         * 发布夹爪/灵巧手百分比目标到对应的控制器
         * @param controller_name 控制器名称
         * @param percent 百分比目标 (0.0=完全闭合, 1.0=完全张开)
         */
        void publishGripperPercent(const std::string& controller_name, double percent);

        /**
         * 发送FSM状态切换命令
         * @param command FSM命令值 (1=HOME, 2=HOLD, 3=OCS2, 100=切换姿态(REST), 0=重置)
         */
        void sendFsmCommand(int32_t command);

        /**
         * 直接发布目标位姿到left_target/right_target话题（解耦模式，无坐标转换）
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 位置（已在目标坐标系下）
         * @param orientation 方向
         */
        void publishTargetPoseDirect(const std::string& armType,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation);

        /**
         * 将Pose消息转换为Eigen::Matrix4d
         * @param msg Pose消息
         * @return 4x4变换矩阵
         */
        Eigen::Matrix4d poseMsgToMatrix(geometry_msgs::msg::Pose::SharedPtr msg);

        /**
         * 从4x4矩阵提取位置和方向
         * @param matrix 4x4变换矩阵
         * @param position 输出位置
         * @param orientation 输出方向
         */
        void matrixToPosOri(const Eigen::Matrix4d& matrix,
                            Eigen::Vector3d& position,
                            Eigen::Quaterniond& orientation);

        /**
         * 检查pose是否发生显著变化
         * @param currentPos 当前位置
         * @param currentOri 当前方向
         * @param prevPos 之前位置
         * @param prevOri 之前方向
         * @return true如果pose发生显著变化
         */
        bool hasPoseChanged(const Eigen::Vector3d& currentPos,
                            const Eigen::Quaterniond& currentOri,
                            const Eigen::Vector3d& prevPos,
                            const Eigen::Quaterniond& prevOri);

        /**
         * 基于差值计算pose并应用到机器人base pose
         * @param xrCurrentPos 当前XR位置
         * @param xrCurrentOri 当前XR方向
         * @param xrBasePos XR base位置
         * @param xrBaseOri XR base方向
         * @param xrBaseFrame 位移坐标系（见 motionFrame）
         * @param robotBasePos 机器人base位置
         * @param robotBaseOri 机器人base方向
         * @param resultPos 输出计算的位置
         * @param resultOri 输出计算的方向
         */
        void calculatePoseFromDifference(const Eigen::Vector3d& xrCurrentPos,
                                         const Eigen::Quaterniond& xrCurrentOri,
                                         const Eigen::Vector3d& xrBasePos,
                                         const Eigen::Quaterniond& xrBaseOri,
                                         const Eigen::Quaterniond& xrBaseFrame,
                                         const Eigen::Vector3d& robotBasePos,
                                         const Eigen::Quaterniond& robotBaseOri,
                                         Eigen::Vector3d& resultPos,
                                         Eigen::Quaterniond& resultOri);

        /// 记录一侧手臂的 XR / 机器人基准位姿和位移坐标系（进入 UPDATE 时）
        void captureLeftBase();
        void captureRightBase();

        // ROS组件
        rclcpp::Node::SharedPtr node_;
        TargetGenerator* target_generator_;

        // 发布器（用于直接发布到left_target/right_target）
        // Health of the xr -> target_generator -> controller hops: XR poses coming in, targets going out, one line per window.
        void reportHealth();
        void recordTarget(std::uint32_t side, const geometry_msgs::msg::Pose& pose);
        // XR poses received and targets sent, kept in shared memory for the incident recorder (nullptr: not recording).
        std::shared_ptr<robot_log::BlackboxRecorder> blackbox_;
        robot_log::RateStats xr_input_stats_;
        robot_log::RateStats target_stats_;
        double health_window_s_ = 10.0;
        rclcpp::TimerBase::SharedPtr health_timer_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target_;
        // 尺度校准后发送校准目标用（校准代码目前停用）
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped_;
        // FSM命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        std::unique_ptr<robot_controller_core::FSMCommandPublisher> fsm_command_publisher_;

        // 订阅器
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_right_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_head_;
        // 按钮事件订阅器（Int32类型）
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_controller_state_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_fsm_state_;
        // 扳机模拟量订阅器（合并订阅左右扳机，使用 Twist 消息）
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_trigger_values_;
        // 机器人 current_pose 订阅和 FSM 命令订阅在 target_generator_node 中统一处理

        // XR pose参数
        Eigen::Matrix4d left_ee_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_ee_pose_ = Eigen::Matrix4d::Identity();

        // XR位置和方向参数（世界/机器人坐标系下）
        // *_position_raw_ 表示未应用缩放系数的原始XR位姿
        Eigen::Vector3d xr_left_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d xr_right_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d left_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d xr_head_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond xr_head_orientation_ = Eigen::Quaterniond::Identity();

        // 用于变化检测的之前pose（更新模式）
        Eigen::Vector3d prev_calculated_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_calculated_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_right_orientation_ = Eigen::Quaterniond::Identity();

        // 目标守卫（见 target_guard.h）：丢弃离末端太远、或相邻两帧跳变太大的目标
        TargetGuardLimits guard_limits_;
        // 本次跟随里最近一个通过守卫的目标；开始/停止跟随、XR 掉线时清空，下一帧只做距离检查
        std::optional<Eigen::Vector3d> last_accepted_left_;
        std::optional<Eigen::Vector3d> last_accepted_right_;

        // 状态管理
        std::atomic<bool> enabled_;
        std::atomic<bool> is_update_mode_; // true = 更新模式（跟随）, false = 存储模式（停止）
        std::atomic<int32_t> current_fsm_state_; // 当前FSM状态：1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ

        // XR base poses（进入 UPDATE 时存储）
        Eigen::Vector3d xr_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond xr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d xr_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond xr_base_right_orientation_ = Eigen::Quaterniond::Identity();
        // 位移坐标系（记录基准时由 motionFrame() 计算：旧行为 = 手柄基准姿态，对齐模式 = 头显 yaw）
        Eigen::Quaterniond xr_base_left_frame_ = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond xr_base_right_frame_ = Eigen::Quaterniond::Identity();

        // 头显 yaw 对齐（见 setAlignHeadYaw）
        bool align_head_yaw_ = false;

        // 机器人base poses（进入 UPDATE 时存储）
        Eigen::Vector3d left_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_base_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d right_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_base_orientation_ = Eigen::Quaterniond::Identity();

        // 当前机器人poses
        Eigen::Vector3d left_current_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_current_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d right_current_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_current_orientation_ = Eigen::Quaterniond::Identity();

        // Hand controllers mapping（类似ControlInputHandler）
        std::vector<std::string> hand_controllers_;
        // Publishers for hand / gripper percentages (created on demand)
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gripper_percent_publishers_;
        std::map<std::string, double> last_gripper_percent_;
        // 检测到的左右控制器名称
        std::string left_gripper_controller_name_;
        std::string right_gripper_controller_name_;

        double xr_pose_scale_;  // 手柄位姿位置缩放因子（左右共用）

        // 参考link名称（用于将XR头显/手柄关联到机器人某个link），从target_generator.yaml读取，默认"base_link"
        std::string reference_link_;

        // 机器人末端当前使用的参考坐标系（来自 left_current_pose/right_current_pose 的 frame_id）
        std::string ee_frame_id_;
        bool ee_frame_id_initialized_ = false;

        // TF组件（用于在不同link/frame之间转换）
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 常量
        static const std::string XR_NODE_NAME;
        static const double POSITION_THRESHOLD;
        static const double ORIENTATION_THRESHOLD;
    };
} // namespace target_generator
