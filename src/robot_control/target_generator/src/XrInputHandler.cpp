//
// Created for Arms ROS2 Control - XrInputHandler
//

#include "target_generator/XrInputHandler.h"
#include <cstdio>
#include "target_generator/TargetGenerator.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace target_generator
{
    // 静态常量定义

    XrInputHandler::XrInputHandler(
        rclcpp::Node::SharedPtr node,
        TargetGenerator* targetGenerator,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        const std::vector<std::string>& handControllers,
        double xr_pose_scale)
        : node_(std::move(node))
          , target_generator_(targetGenerator)
          , blackbox_(robot_log::BlackboxRecorder::create("target_generator"))
          , pub_left_target_(std::move(pub_left_target))
          , pub_right_target_(std::move(pub_right_target))
          , is_update_mode_(false)
          , current_fsm_state_(kFsmHold)  // 默认HOLD状态
          , hand_controllers_(handControllers)
          , xr_pose_scale_(xr_pose_scale)
    {
        guard_limits_.enabled = node_->declare_parameter("xr_guard.enabled", guard_limits_.enabled);
        guard_limits_.max_distance = node_->declare_parameter("xr_guard.max_target_distance", guard_limits_.max_distance);
        guard_limits_.max_step = node_->declare_parameter("xr_guard.max_target_step", guard_limits_.max_step);
        stale_ns_ = node_->declare_parameter("xr_guard.stale_ms", static_cast<std::int64_t>(stale_ns_ / 1'000'000)) *
                    1'000'000;

        // 检测左右控制器名称
        detectGripperControllers(hand_controllers_);

        health_window_s_ = robot_log::healthWindowSeconds();
        health_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(health_window_s_)),
            [this]() { reportHealth(); });

        // 创建XR订阅器
        auto xrLeftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->xrLeftCallback(msg);
        };
        sub_left_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/left_ee_pose", 10, xrLeftCallback);

        auto xrRightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->xrRightCallback(msg);
        };
        sub_right_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/right_ee_pose", 10, xrRightCallback);

        // 创建按钮事件订阅器（Int32类型）
        auto controllerStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->processButtonEvent(msg);
        };
        sub_controller_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/teleop/controller_state", 10, controllerStateCallback);

        // 创建扳机模拟量订阅器（合并订阅左右扳机，使用 Twist 消息）
        // linear.x 表示左扳机拉动比例，angular.x 表示右扳机拉动比例
        auto triggerValuesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->triggerValuesCallback(msg);
        };
        sub_trigger_values_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/teleop/trigger_values", 10, triggerValuesCallback);

        // 创建 FSM 命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        auto pub_fsm_command = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
        fsm_command_publisher_ = std::make_unique<robot_controller_core::FSMCommandPublisher>(
            node_, pub_fsm_command);

        // 控制器实际所处的状态（transient_local：晚启动也能拿到当前状态）
        sub_fsm_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/fsm_state", rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Int32::SharedPtr msg) { fsmStateCallback(msg); });

        // 头部跟随：头显位姿、机器人头当前位姿、全身控制器的头部任务状态进，head_target 出（队列长度 1，只要最新的）
        pub_head_target_ = node_->create_publisher<geometry_msgs::msg::Pose>("head_target", 1);
        sub_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/head_pose", 10,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) { xrHeadCallback(msg); });
        sub_robot_head_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "head_current_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { robotHeadPoseCallback(msg); });
        sub_whole_body_state_ = node_->create_subscription<robot_control_msgs::msg::WholeBodyCurrentState>(
            "/whole_body_controller/current_state", 10,
            [this](robot_control_msgs::msg::WholeBodyCurrentState::SharedPtr msg) { wholeBodyStateCallback(msg); });

        // 底盘遥控：摇杆轴值进，底盘速度出（和 joystick_teleop / 全身控制器是同一个话题）。默认关：不开就不创建
        // 发布者、订阅和看门狗，不会往底盘速度话题发任何东西，误按左摇杆也不会让手臂停下
        chassis_enabled_ = node_->declare_parameter("xr_base", chassis_enabled_);
        chassis_linear_scale_ = node_->declare_parameter("xr_chassis.linear_scale", chassis_linear_scale_);
        chassis_angular_scale_ = node_->declare_parameter("xr_chassis.angular_scale", chassis_angular_scale_);
        chassis_stale_ns_ = node_->declare_parameter("xr_chassis.stale_ms",
                                                     static_cast<std::int64_t>(chassis_stale_ns_ / 1'000'000)) *
                            1'000'000;
        if (!std::isfinite(chassis_linear_scale_) || chassis_linear_scale_ < 0.0 ||
            !std::isfinite(chassis_angular_scale_) || chassis_angular_scale_ < 0.0 || chassis_stale_ns_ <= 0)
        {
            throw std::invalid_argument(
                "xr_chassis.linear_scale and xr_chassis.angular_scale must be finite and >= 0, xr_chassis.stale_ms > 0");
        }
        if (chassis_enabled_)
        {
            base_velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("base_velocity_target", 10);
            sub_thumbstick_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                "/teleop/thumbstick_axes", 10,
                [this](geometry_msgs::msg::Twist::SharedPtr msg) { thumbstickCallback(msg); });
            chassis_watchdog_timer_ =
                node_->create_wall_timer(std::chrono::milliseconds(50), [this]() { chassisWatchdog(); });
            RCLCPP_WARN(node_->get_logger(), "🕹️ 底盘遥控已启用（xr_base = true）：左摇杆按下开 / 关");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️ 底盘遥控关闭（xr_base = false，默认）");
        }

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ XrInputHandler created");
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ 按键：右A 状态机前进，左X 状态机后退，右摇杆按下 跟随(UPDATE)/停止(STORAGE)（仅 OCS2）；"
                    "扳机按比例开合手/夹爪");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ XR pose scale: %.3f", xr_pose_scale_);

        // 输出检测到的控制器信息
        if (!left_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Left gripper controller: %s", left_gripper_controller_name_.c_str());
        }
        if (!right_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right gripper controller: %s", right_gripper_controller_name_.c_str());
        }
        if (left_gripper_controller_name_.empty() && right_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ No gripper controllers detected - gripper control disabled");
        }
    }

    void XrInputHandler::detectGripperControllers(const std::vector<std::string>& hand_controllers)
    {
        // 清空之前的检测结果
        left_gripper_controller_name_.clear();
        right_gripper_controller_name_.clear();

        // 从hand_controllers参数中提取左右控制器名称
        for (const auto& controller_name : hand_controllers)
        {
            // 转换为小写以便比较
            std::string name_lower = controller_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

            // 检测左控制器
            if (name_lower.find("left") != std::string::npos)
            {
                left_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "🕹️🕶️🕹️ Detected left gripper controller: %s", controller_name.c_str());
            }
            // 检测右控制器
            else if (name_lower.find("right") != std::string::npos)
            {
                right_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "🕹️🕶️🕹️ Detected right gripper controller: %s", controller_name.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "🕹️ Hand controller '%s' has no left/right in name; XR gripper mapping skipped",
                            controller_name.c_str());
            }
        }
    }

    void XrInputHandler::publishGripperPercent(const std::string& controller_name, double percent)
    {
        if (controller_name.empty())
        {
            return;
        }

        percent = std::clamp(percent, 0.0, 1.0);
        auto last_it = last_gripper_percent_.find(controller_name);
        if (last_it != last_gripper_percent_.end() && std::abs(last_it->second - percent) < 0.005)
        {
            return;
        }
        last_gripper_percent_[controller_name] = percent;

        if (gripper_percent_publishers_.find(controller_name) == gripper_percent_publishers_.end())
        {
            std::string topic_name = "/" + controller_name + "/target_percent";
            gripper_percent_publishers_[controller_name] =
                node_->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Created gripper percent publisher: %s", topic_name.c_str());
        }

        auto target_msg = std_msgs::msg::Float64();
        target_msg.data = percent;
        gripper_percent_publishers_[controller_name]->publish(target_msg);
    }

    void XrInputHandler::sendFsmCommand(int32_t command)
    {
        // 使用通用工具类发布FSM命令（自动处理command=100的特殊情况）
        if (fsm_command_publisher_)
        {
            fsm_command_publisher_->publishCommand(command);
        }
    }

    void XrInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, left_current_position_, left_current_orientation_);
        arm_[kLeft].has_current_pose = true;
    }

    void XrInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, right_current_position_, right_current_orientation_);
        arm_[kRight].has_current_pose = true;
    }

    void XrInputHandler::reportHealth()
    {
        const auto input = xr_input_stats_.take(health_window_s_);
        const auto targets = target_stats_.take(health_window_s_);
        char extra[96];
        std::snprintf(extra, sizeof(extra), "input_hz=%.1f input_gap_max_ms=%.1f", input.rate_hz, input.gap_max_ms);
        RCLCPP_INFO(node_->get_logger(), "%s", robot_log::formatRateLine("target_generator", health_window_s_, targets, extra).c_str());
    }

    void XrInputHandler::xrLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        xr_input_stats_.record();
        if (blackbox_)
        {
            blackbox_->record(robot_log::blackbox::kTelXrIn,
                            {msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z},
                            0);
        }
        arm_[kLeft].last_xr_pose_ns = steadyNowNs();

        // 左话题接收的是左手柄数据
        left_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）左手柄位姿
        matrixToPosOri(left_ee_pose_, xr_left_position_raw_, left_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的left_position_
        left_position_ = xr_left_position_raw_;
        left_position_ *= xr_pose_scale_;

        // 存储模式（STORAGE）：只记录手柄位姿，不发目标
        if (is_update_mode_.load() && arm_[kLeft].following)
        {
            followArm("left", left_position_, left_orientation_, xr_base_left_position_, xr_base_left_orientation_,
                      left_base_position_, left_base_orientation_,
                      left_current_position_, last_accepted_left_);
        }
    }

    void XrInputHandler::xrRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        xr_input_stats_.record();
        if (blackbox_)
        {
            blackbox_->record(robot_log::blackbox::kTelXrIn,
                            {msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z},
                            1);
        }
        arm_[kRight].last_xr_pose_ns = steadyNowNs();

        // 右话题接收的是右手柄数据
        right_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）右手柄位姿
        matrixToPosOri(right_ee_pose_, xr_right_position_raw_, right_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的right_position_
        right_position_ = xr_right_position_raw_;
        right_position_ *= xr_pose_scale_;

        if (is_update_mode_.load() && arm_[kRight].following)
        {
            followArm("right", right_position_, right_orientation_, xr_base_right_position_, xr_base_right_orientation_,
                      right_base_position_, right_base_orientation_,
                      right_current_position_, last_accepted_right_);
        }
    }

    void XrInputHandler::followArm(const std::string& arm,
                                   const Eigen::Vector3d& xrPos, const Eigen::Quaterniond& xrOri,
                                   const Eigen::Vector3d& xrBasePos, const Eigen::Quaterniond& xrBaseOri,
                                   const Eigen::Vector3d& robotBasePos, const Eigen::Quaterniond& robotBaseOri,
                                   const Eigen::Vector3d& robotCurrentPos,
                                   std::optional<Eigen::Vector3d>& lastAccepted)
    {
        // 更新模式（UPDATE）：目标 = 进入跟随时的机器人位姿 + 手柄自那时起的位移和转动
        Eigen::Vector3d calculatedPos;
        Eigen::Quaterniond calculatedOri;
        calculatePoseFromDifference(xrPos, xrOri, xrBasePos, xrBaseOri, robotBasePos, robotBaseOri,
                                    calculatedPos, calculatedOri);

        // 守卫：不论来源（头显毛刺、跟踪原点移动、坏帧），一帧目标不能把手臂拽远。
        // 丢弃的帧不更新参考，所以之后回到正常范围的帧仍会被接受。
        const auto verdict = checkTarget(guard_limits_, calculatedPos, calculatedOri, robotCurrentPos, lastAccepted);
        if (verdict != TargetVerdict::Accept)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[TEL-002] %s target dropped (%s): target (%.3f, %.3f, %.3f), end effector (%.3f, %.3f, %.3f), "
                                 "%.3f m from the end effector (limit %.3f), %.3f m from the previous target (limit %.3f); "
                                 "if it keeps happening press the right stick to align again",
                                 arm.c_str(), toString(verdict), calculatedPos.x(), calculatedPos.y(), calculatedPos.z(),
                                 robotCurrentPos.x(), robotCurrentPos.y(), robotCurrentPos.z(),
                                 (calculatedPos - robotCurrentPos).norm(), guard_limits_.max_distance,
                                 lastAccepted ? (calculatedPos - *lastAccepted).norm() : 0.0, guard_limits_.max_step);
            return;
        }
        lastAccepted = calculatedPos;
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                              "🕹️🕶️🕹️ %s XR Base: [%.3f, %.3f, %.3f] Current: [%.3f, %.3f, %.3f] -> Target: [%.3f, %.3f, %.3f]",
                              arm.c_str(), xrBasePos.x(), xrBasePos.y(), xrBasePos.z(), xrPos.x(), xrPos.y(), xrPos.z(),
                              calculatedPos.x(), calculatedPos.y(), calculatedPos.z());
        publishTargetPoseDirect(arm, calculatedPos, calculatedOri);
    }

    void XrInputHandler::recordTarget(std::uint32_t side, const geometry_msgs::msg::Pose& pose)
    {
        if (!blackbox_) { return; }
        blackbox_->record(robot_log::blackbox::kTelTargetOut,
                        {pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z},
                        side);
    }

    void XrInputHandler::publishTargetPoseDirect(const std::string& armType,
                                                const Eigen::Vector3d& position,
                                                const Eigen::Quaterniond& orientation)
    {
        // 转换为geometry_msgs格式
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.w = orientation.w();
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();

        // 直接发布到对应的话题（无坐标转换）
        if (armType == "left" && pub_left_target_)
        {
            pub_left_target_->publish(pose);
            target_stats_.record();
            recordTarget(0, pose);
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "🕹️🕶️🕹️ Published left_target: [%.3f, %.3f, %.3f]",
                        pose.position.x, pose.position.y, pose.position.z);
        }
        else if (armType == "right" && pub_right_target_)
        {
            pub_right_target_->publish(pose);
            target_stats_.record();
            recordTarget(1, pose);
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "🕹️🕶️🕹️ Published right_target: [%.3f, %.3f, %.3f]",
                        pose.position.x, pose.position.y, pose.position.z);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "🕹️🕶️🕹️ Invalid armType or publisher not initialized: %s", armType.c_str());
        }
    }

    Eigen::Matrix4d XrInputHandler::poseMsgToMatrix(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = msg->position.x;
        pose(1, 3) = msg->position.y;
        pose(2, 3) = msg->position.z;

        Eigen::Quaterniond q(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
        Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
        pose.block<3, 3>(0, 0) = rot;

        return pose;
    }


    void XrInputHandler::matrixToPosOri(const Eigen::Matrix4d& matrix,
                                        Eigen::Vector3d& position,
                                        Eigen::Quaterniond& orientation)
    {
        position = matrix.block<3, 1>(0, 3);
        Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
        orientation = Eigen::Quaterniond(rot);
    }

    void XrInputHandler::calculatePoseFromDifference(const Eigen::Vector3d& xrCurrentPos,
                                                     const Eigen::Quaterniond& xrCurrentOri,
                                                     const Eigen::Vector3d& xrBasePos,
                                                     const Eigen::Quaterniond& xrBaseOri,
                                                     const Eigen::Vector3d& robotBasePos,
                                                     const Eigen::Quaterniond& robotBaseOri,
                                                     Eigen::Vector3d& resultPos,
                                                     Eigen::Quaterniond& resultOri)
    {
        // 1. 计算XR位置在世界坐标系下的差值
        Eigen::Vector3d xrPosDiff_world = xrCurrentPos - xrBasePos;

        // 2. 将位置差值转换到进入UPDATE时手柄的姿态坐标系：位置和下面的姿态差值在同一个坐标系里
        Eigen::Vector3d xrPosDiff_local = xrBaseOri.inverse() * xrPosDiff_world;

        // 3. 计算XR姿态差值（相对旋转）
        Eigen::Quaterniond xrOriDiff = xrBaseOri.inverse() * xrCurrentOri;

        // 4. 将局部坐标系的位置差值应用到机器人基准位姿上，保持相对运动一致性
        resultPos = robotBasePos + xrPosDiff_local;

        // 5. 应用姿态差值
        resultOri = xrOriDiff * robotBaseOri;

        // 6. 归一化四元数以避免漂移
        resultOri.normalize();
    }

    void XrInputHandler::triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double left_trigger_pull = std::clamp(msg->linear.x, 0.0, 1.0);
        const double right_trigger_pull = std::clamp(msg->angular.x, 0.0, 1.0);

        // 灵巧手/夹爪控制器的 target_percent: 0.0=闭合, 1.0=张开。
        // XR 扳机语义: 0.0=松开, 1.0=拉满，所以这里反向映射。
        publishGripperPercent(left_gripper_controller_name_, 1.0 - left_trigger_pull);
        publishGripperPercent(right_gripper_controller_name_, 1.0 - right_trigger_pull);
    }

    void XrInputHandler::processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 事件码见 xr_teleop/button_event_encoder.hpp：0=无事件（每帧都会收到），1/2/3 为按键；其他值（旧方案的
        // 事件码）一律忽略
        switch (msg->data)
        {
            case kEventFsmForward: // 右A：FSM状态前进
            {
                // 根据当前FSM状态发送相应的转换命令
                int32_t current_state = current_fsm_state_.load();

                if (current_state == kFsmHold)
                {
                    sendFsmCommand(kFsmOcs2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOLD → OCS2");
                }
                else if (current_state == kFsmHome)
                {
                    sendFsmCommand(kFsmHold);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOME → HOLD");
                }
                else if (current_state == kFsmOcs2)
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（已在OCS2状态，无法继续前进）");
                }
                break;
            }
            case kEventFsmBack: // 左X：FSM状态后退/切换
            {
                int32_t current_state = current_fsm_state_.load();

                if (current_state == kFsmOcs2)
                {
                    sendFsmCommand(kFsmHold);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: OCS2 → HOLD");
                }
                else if (current_state == kFsmMoveJ)
                {
                    // MOVEJ → HOLD（不能直接进入 HOME）
                    sendFsmCommand(kFsmHold);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: MOVEJ → HOLD");
                }
                else if (current_state == kFsmHold)
                {
                    sendFsmCommand(kFsmHome);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: HOLD → HOME");
                }
                else if (current_state == kFsmHome)
                {
                    // HOME状态下，X按钮切换姿态 (HOME ↔ REST)
                    sendFsmCommand(kFsmRestToggle);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: 在HOME状态切换姿态 (HOME ↔ REST)");
                }
                break;
            }
            case kEventToggleFollow: // 右摇杆按下：UPDATE（跟随）↔ STORAGE（停止）
            {
                toggleFollow();
                break;
            }
            case kEventToggleChassisDrive: // 左摇杆按下：底盘遥控 开 ↔ 关
            {
                toggleChassisDrive();
                break;
            }
            case kEventNone:
            default:
                // 无事件或旧方案的事件码
                break;
        }
    }

    void XrInputHandler::toggleFollow()
    {
        // 只在OCS2状态下执行；不在 OCS2 时按了没反应，用 INFO 提示（节流），方便排查
        if (current_fsm_state_.load() != kFsmOcs2)
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "🔘 [右摇杆按钮] 按下 - 功能: 切换跟随 - 操作: 忽略（当前 FSM 不是 OCS2，先按右A进入 OCS2）");
            return;
        }

        // 确保切换到连续发布模式（更稳健，防止用户手动切换回单次模式）
        if (target_generator_ && target_generator_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_generator_->togglePublishMode();
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "   TargetGenerator已切换到CONTINUOUS模式");
        }

        if (!is_update_mode_.load())
        {
            // 切换到更新模式：以当前手柄位姿和机器人位姿为基准，之后手柄的位移和转动驱动机器人。
            // 只有输入可信的手臂才开始跟随：头显连着、手柄位姿是新的、机器人当前末端位姿已收到。
            const auto now_ns = steadyNowNs();
            std::string skipped;
            for (const int side : {kLeft, kRight})
            {
                const std::string why = whyNotFollowing(side, now_ns);
                if (!why.empty())
                {
                    skipped += std::string(skipped.empty() ? "" : "; ") + (side == kLeft ? "left: " : "right: ") + why;
                    continue;
                }
                if (side == kLeft) { captureLeftBase(); last_accepted_left_.reset(); }
                else { captureRightBase(); last_accepted_right_.reset(); }
                arm_[side].following = true;
            }
            if (!arm_[kLeft].following && !arm_[kRight].following)
            {
                RCLCPP_WARN(node_->get_logger(), "[TEL-003] not following, no arm can start (%s)", skipped.c_str());
                return;
            }
            if (!skipped.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 部分手臂不跟随 (%s)", skipped.c_str());
            }
            is_update_mode_.store(true);
            RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换跟随 - 操作: 切换到UPDATE模式（已存储基准位姿，开始跟随）");
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                  "   XR Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                                  xr_base_left_position_.x(), xr_base_left_position_.y(), xr_base_left_position_.z(),
                                  xr_base_right_position_.x(), xr_base_right_position_.y(), xr_base_right_position_.z());
        }
        else
        {
            stopFollowing();
            RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换跟随 - 操作: 切换到STORAGE模式（双臂停止跟随）");
        }
    }

    void XrInputHandler::stopHeadTracking()
    {
        head_following_ = false;
        last_accepted_head_.reset();
    }

    void XrInputHandler::robotHeadPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const Eigen::Vector3d position(p.x, p.y, p.z);
        const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        if (!headPoseUsable(position, orientation))
        {
            // 没有可信的机器人头位姿就不能当基准，也不能拿它来判断目标是不是太远
            has_robot_head_pose_ = false;
            stopHeadTracking();
            return;
        }
        robot_head_position_ = position;
        robot_head_orientation_ = orientation.normalized();
        has_robot_head_pose_ = true;
    }

    void XrInputHandler::wholeBodyStateCallback(robot_control_msgs::msg::WholeBodyCurrentState::SharedPtr msg)
    {
        head_state_.store(msg->head_state);
        if (msg->head_state != robot_control_msgs::msg::WholeBodyCurrentState::HEAD_TRACKING)
        {
            stopHeadTracking();
        }
    }

    std::string XrInputHandler::whyChassisDriveBlocked() const
    {
        if (current_fsm_state_.load() != kFsmOcs2) { return "not in OCS2"; }
        return {};
    }

    std::string XrInputHandler::whyNoChassisDrive(std::int64_t now_ns) const
    {
        std::string why = whyChassisDriveBlocked();
        if (!why.empty()) { return why; }
        if (now_ns - last_thumbstick_ns_.load() > chassis_stale_ns_)
        {
            return "no fresh stick input from the headset";
        }
        return {};
    }

    void XrInputHandler::toggleChassisDrive()
    {
        if (!chassis_enabled_)
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🔘 [左摇杆按钮] 按下 - 忽略（底盘遥控关闭：xr_base = false）");
            return;
        }
        if (chassis_drive_.load())
        {
            stopChassisDrive("the left stick key was pressed again", false);
            return;
        }
        const std::string why = whyNoChassisDrive(steadyNowNs());
        if (!why.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "[TEL-004] chassis drive refused: %s", why.c_str());
            return;
        }
        chassis_drive_.store(true);  // the arms keep following: their targets are relative to the body
        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [左摇杆按钮] 按下 - 功能: 底盘遥控 - 操作: 开启（左摇杆前后 / 左右，右摇杆转向；满杆 %.2f m/s、%.2f rad/s）",
                    chassis_linear_scale_, chassis_angular_scale_);
    }

    void XrInputHandler::stopChassisDrive(const std::string& reason, bool forced)
    {
        if (!chassis_drive_.exchange(false)) { return; }
        publishBaseVelocity(0.0, 0.0, 0.0);
        if (forced)
        {
            RCLCPP_WARN(node_->get_logger(), "[TEL-004] chassis drive stopped, base velocity set to zero: %s",
                        reason.c_str());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "🔘 底盘遥控关闭（%s），底盘速度置零", reason.c_str());
        }
    }

    void XrInputHandler::publishBaseVelocity(double forward, double sideways, double turn)
    {
        if (!base_velocity_publisher_) { return; }  // chassis drive is off (xr_base = false)
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = forward;
        velocity.linear.y = sideways;
        velocity.angular.z = turn;
        base_velocity_publisher_->publish(velocity);
    }

    void XrInputHandler::thumbstickCallback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_thumbstick_ns_.store(steadyNowNs());
        if (!chassis_drive_.load()) { return; }

        const std::string blocked = whyChassisDriveBlocked();
        if (!blocked.empty())
        {
            stopChassisDrive(blocked, true);
            return;
        }

        // linear = 左摇杆 (x 右为正, y 前为负)，angular.x = 右摇杆 x（右为正）。机器人轴：x 前、y 左、绕 z 逆时针为正，
        // 所以三个量都取负。一帧里有任何非有限值就整帧当 0：宁可停，不能带着 NaN 往底盘发
        const double left_x = msg->linear.x, left_y = msg->linear.y, right_x = msg->angular.x;
        if (!std::isfinite(left_x) || !std::isfinite(left_y) || !std::isfinite(right_x))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[TEL-004] stick frame is not finite, base velocity set to zero");
            publishBaseVelocity(0.0, 0.0, 0.0);
            return;
        }
        const auto axis = [](double v) { return std::clamp(v, -1.0, 1.0); };
        publishBaseVelocity(-axis(left_y) * chassis_linear_scale_,
                            -axis(left_x) * chassis_linear_scale_,
                            -axis(right_x) * chassis_angular_scale_);
    }

    void XrInputHandler::chassisWatchdog()
    {
        if (!chassis_drive_.load()) { return; }
        const auto silent_ns = steadyNowNs() - last_thumbstick_ns_.load();
        if (silent_ns > chassis_stale_ns_)
        {
            char reason[96];
            std::snprintf(reason, sizeof(reason), "no stick input for %.0f ms (limit %.0f ms)", silent_ns * 1e-6,
                          chassis_stale_ns_ * 1e-6);
            stopChassisDrive(reason, true);
        }
    }

    void XrInputHandler::xrHeadCallback(geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 头部只在这些条件同时满足时跟随：手臂在跟随（UPDATE，只会在 OCS2 里）、全身控制器的头部任务是
        // HEAD_TRACKING、已收到机器人头当前位姿。任何一项不满足就停止，再满足时重新取基准，不会跳。
        const bool ready = is_update_mode_.load() && current_fsm_state_.load() == kFsmOcs2 &&
                           head_state_.load() == robot_control_msgs::msg::WholeBodyCurrentState::HEAD_TRACKING &&
                           has_robot_head_pose_;
        if (!ready)
        {
            stopHeadTracking();
            return;
        }

        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                                             msg->orientation.z);
        if (!headPoseUsable(position, orientation))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[TEL-002] head target dropped: the headset pose is not finite or has a zero quaternion");
            return;
        }
        followHead(position, orientation.normalized());
    }

    void XrInputHandler::followHead(const Eigen::Vector3d& xrPos, const Eigen::Quaterniond& xrOri)
    {
        if (!head_following_)
        {
            head_base_ = {xrPos, xrOri, robot_head_position_, robot_head_orientation_};
            last_accepted_head_.reset();
            head_following_ = true;
        }

        const HeadTarget target = headTarget(head_base_, xrPos, xrOri);

        // 守卫和手臂的一样：目标离机器人头太远、或和上一个目标差太多就丢弃这一帧，参考不更新
        const auto verdict = checkTarget(guard_limits_, target.position, target.orientation, robot_head_position_,
                                         last_accepted_head_);
        if (verdict != TargetVerdict::Accept)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[TEL-002] head target dropped (%s): target (%.3f, %.3f, %.3f), head (%.3f, %.3f, %.3f), "
                                 "%.3f m from the head (limit %.3f), %.3f m from the previous target (limit %.3f)",
                                 toString(verdict), target.position.x(), target.position.y(), target.position.z(),
                                 robot_head_position_.x(), robot_head_position_.y(), robot_head_position_.z(),
                                 (target.position - robot_head_position_).norm(), guard_limits_.max_distance,
                                 last_accepted_head_ ? (target.position - *last_accepted_head_).norm() : 0.0,
                                 guard_limits_.max_step);
            return;
        }
        last_accepted_head_ = target.position;

        geometry_msgs::msg::Pose pose;
        pose.position.x = target.position.x();
        pose.position.y = target.position.y();
        pose.position.z = target.position.z();
        pose.orientation.w = target.orientation.w();
        pose.orientation.x = target.orientation.x();
        pose.orientation.y = target.orientation.y();
        pose.orientation.z = target.orientation.z();
        pub_head_target_->publish(pose);
    }

    void XrInputHandler::fsmStateCallback(std_msgs::msg::Int32::SharedPtr msg)
    {
        const int32_t state = msg->data;
        if (state != kFsmHome && state != kFsmHold && state != kFsmOcs2 && state != kFsmMoveJ)
        {
            return;  // 0 = 控制器还没有有效状态
        }

        const int32_t old_state = current_fsm_state_.exchange(state);
        // 离开 OCS2 时底盘立刻停：再进 OCS2 要重新按左摇杆
        if (state != kFsmOcs2 && chassis_drive_.load())
        {
            stopChassisDrive("the controller left OCS2", true);
        }
        // 离开 OCS2 时自动切换到存储模式：再次进入 OCS2 后要重新按右摇杆才跟随，不会一进 OCS2 就跳
        if (state != old_state && state != kFsmOcs2 && is_update_mode_.load())
        {
            stopFollowing();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ FSM状态不是OCS2，自动切换到STORAGE模式 (状态=%d)", state);
        }
    }

    std::int64_t XrInputHandler::steadyNowNs()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::string XrInputHandler::whyNotFollowing(int side, std::int64_t now_ns) const
    {
        if (!arm_[side].has_current_pose) { return "no current end effector pose from the robot yet"; }
        if (now_ns - arm_[side].last_xr_pose_ns > stale_ns_)
        {
            return "no fresh controller pose (tracking lost, or the controller is off)";
        }
        return {};
    }

    void XrInputHandler::stopFollowing()
    {
        for (auto& arm : arm_) { arm.following = false; }
        last_accepted_left_.reset();
        last_accepted_right_.reset();
        stopHeadTracking();
        is_update_mode_.store(false);
    }

    void XrInputHandler::captureLeftBase()
    {
        xr_base_left_position_ = left_position_;
        xr_base_left_orientation_ = left_orientation_;
        left_base_position_ = left_current_position_;
        left_base_orientation_ = left_current_orientation_;
    }

    void XrInputHandler::captureRightBase()
    {
        xr_base_right_position_ = right_position_;
        xr_base_right_orientation_ = right_orientation_;
        right_base_position_ = right_current_position_;
        right_base_orientation_ = right_current_orientation_;
    }
} // namespace target_generator
