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
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace target_generator
{
    // 静态常量定义
    const std::string XrInputHandler::XR_NODE_NAME = "/xr_target_node";
    const double XrInputHandler::POSITION_THRESHOLD = 0.01; // 1cm threshold for position changes
    const double XrInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)

    XrInputHandler::XrInputHandler(
        rclcpp::Node::SharedPtr node,
        TargetGenerator* targetGenerator,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        const std::vector<std::string>& handControllers,
        double xr_pose_scale,
        const std::string& reference_link)
        : node_(std::move(node))
          , target_generator_(targetGenerator)
          , flight_(robot_log::FlightRecorder::create("target_generator"))
          , pub_left_target_(std::move(pub_left_target))
          , pub_right_target_(std::move(pub_right_target))
          , enabled_(false)
          , is_update_mode_(false)
          , current_fsm_state_(kFsmHold)  // 默认HOLD状态
          , hand_controllers_(handControllers)
          , xr_pose_scale_(xr_pose_scale)
          , reference_link_(reference_link)
    {
        guard_limits_.enabled = node_->declare_parameter("xr_guard.enabled", guard_limits_.enabled);
        guard_limits_.max_distance = node_->declare_parameter("xr_guard.max_target_distance", guard_limits_.max_distance);
        guard_limits_.max_step = node_->declare_parameter("xr_guard.max_target_step", guard_limits_.max_step);

        // 初始化 TF 组件（用于在 reference_link 与末端 frame 之间进行坐标变换）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

        // 头显位姿订阅器
        auto xrHeadCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->xrHeadCallback(msg);
        };
        sub_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/head_pose", 10, xrHeadCallback);

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

        // 创建双臂目标位姿发布器（用于尺度校准后发送校准目标；校准代码目前停用，见 processButtonEvent）
        pub_dual_target_stamped_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/dual_target/stamped", 1);

        // 创建 FSM 命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        auto pub_fsm_command = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
        fsm_command_publisher_ = std::make_unique<robot_controller_core::FSMCommandPublisher>(
            node_, pub_fsm_command);

        // 控制器实际所处的状态（transient_local：晚启动也能拿到当前状态）
        sub_fsm_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/fsm_state", rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Int32::SharedPtr msg) { fsmStateCallback(msg); });

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ XrInputHandler created (XR control is DISABLED until %s appears)",
                    XR_NODE_NAME.c_str());
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

    bool XrInputHandler::checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName)
    {
        std::vector<std::string> nodeNames = node->get_node_graph_interface()->get_node_names();

        for (const auto& name : nodeNames)
        {
            if (name == targetNodeName)
            {
                return true;
            }
        }
        return false;
    }

    void XrInputHandler::enable()
    {
        enabled_.store(true);
        // 启用XR控制时，自动设置为连续发布模式以获得更好的响应性
        if (target_generator_ && target_generator_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_generator_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ TargetGenerator switched to CONTINUOUS mode for XR control");
        }
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ XR control ENABLED!");
    }

    void XrInputHandler::disable()
    {
        enabled_.store(false);
        // Following must not outlive the headset: the bases captured before the drop-out no longer match where
        // the headset (and its origin) is after it returns, so resuming would send a far-away target.
        // The next toggleFollow() recaptures both bases.
        last_accepted_left_.reset();
        last_accepted_right_.reset();
        if (is_update_mode_.exchange(false))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[TEL-001] XR control disabled while following: following stopped, press the right stick again to resume");
        }
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ XR control DISABLED!");
    }

    void XrInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 初始化末端参考坐标系（frame_id）- 左右手共用一次即可
        if (!ee_frame_id_initialized_ && !msg->header.frame_id.empty())
        {
            ee_frame_id_ = msg->header.frame_id;
            ee_frame_id_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ EE frame_id initialized from left_current_pose: %s",
                        ee_frame_id_.c_str());
        }

        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, left_current_position_, left_current_orientation_);
    }

    void XrInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 如果还未初始化 frame_id，也可以从右手话题初始化（通常左右相同）
        if (!ee_frame_id_initialized_ && !msg->header.frame_id.empty())
        {
            ee_frame_id_ = msg->header.frame_id;
            ee_frame_id_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ EE frame_id initialized from right_current_pose: %s",
                        ee_frame_id_.c_str());
        }

        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, right_current_position_, right_current_orientation_);
    }

    void XrInputHandler::xrHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, xr_head_position_, xr_head_orientation_);
    }

    void XrInputHandler::reportHealth()
    {
        const auto input = xr_input_stats_.take(health_window_s_);
        const auto targets = target_stats_.take(health_window_s_);
        char extra[96];
        std::snprintf(extra, sizeof(extra), "input_hz=%.1f input_gap_max_ms=%.1f enabled=%d", input.rate_hz,
                      input.gap_max_ms, enabled_.load() ? 1 : 0);
        RCLCPP_INFO(node_->get_logger(), "%s", robot_log::formatRateLine("target_generator", health_window_s_, targets, extra).c_str());
    }

    void XrInputHandler::xrLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        xr_input_stats_.record();
        if (flight_)
        {
            flight_->record(robot_log::flight::kTelXrIn,
                            {msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z},
                            0);
        }
        if (checkNodeExists(node_, XR_NODE_NAME) && !enabled_.load())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🕹️🕶️🕹️ xr_target_node found, XR control ENABLED!");
            this->enable();
        }
        else if (!checkNodeExists(node_, XR_NODE_NAME) && enabled_.load())
        {
            this->disable();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[TEL-001] 🕹️🕶️🕹️ xr_target_node not found, XR control DISABLED!");
            return;
        }

        // 左话题接收的是左手柄数据
        left_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）左手柄位姿
        matrixToPosOri(left_ee_pose_, xr_left_position_raw_, left_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的left_position_
        left_position_ = xr_left_position_raw_;
        left_position_ *= xr_pose_scale_;

        // 存储模式（STORAGE）：只记录手柄位姿，不发目标
        if (enabled_.load() && is_update_mode_.load())
        {
            followArm("left", left_position_, left_orientation_, xr_base_left_position_, xr_base_left_orientation_,
                      xr_base_left_frame_, left_base_position_, left_base_orientation_,
                      left_current_position_, last_accepted_left_,
                      prev_calculated_left_position_, prev_calculated_left_orientation_);
        }
    }

    void XrInputHandler::xrRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        xr_input_stats_.record();
        if (flight_)
        {
            flight_->record(robot_log::flight::kTelXrIn,
                            {msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z},
                            1);
        }
        // 右话题接收的是右手柄数据
        right_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）右手柄位姿
        matrixToPosOri(right_ee_pose_, xr_right_position_raw_, right_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的right_position_
        right_position_ = xr_right_position_raw_;
        right_position_ *= xr_pose_scale_;

        if (enabled_.load() && is_update_mode_.load())
        {
            followArm("right", right_position_, right_orientation_, xr_base_right_position_, xr_base_right_orientation_,
                      xr_base_right_frame_, right_base_position_, right_base_orientation_,
                      right_current_position_, last_accepted_right_,
                      prev_calculated_right_position_, prev_calculated_right_orientation_);
        }
    }

    void XrInputHandler::followArm(const std::string& arm,
                                   const Eigen::Vector3d& xrPos, const Eigen::Quaterniond& xrOri,
                                   const Eigen::Vector3d& xrBasePos, const Eigen::Quaterniond& xrBaseOri,
                                   const Eigen::Quaterniond& xrBaseFrame,
                                   const Eigen::Vector3d& robotBasePos, const Eigen::Quaterniond& robotBaseOri,
                                   const Eigen::Vector3d& robotCurrentPos,
                                   std::optional<Eigen::Vector3d>& lastAcceptedPos,
                                   Eigen::Vector3d& prevPos, Eigen::Quaterniond& prevOri)
    {
        // 更新模式（UPDATE）：目标 = 进入跟随时的机器人位姿 + 手柄自那时起的位移和转动
        Eigen::Vector3d calculatedPos;
        Eigen::Quaterniond calculatedOri;
        calculatePoseFromDifference(xrPos, xrOri, xrBasePos, xrBaseOri, xrBaseFrame, robotBasePos, robotBaseOri,
                                    calculatedPos, calculatedOri);

        // 守卫：不论来源（头显毛刺、跟踪原点移动、坏帧），一帧目标不能把手臂拽远。
        // 丢弃的帧不更新参考，所以之后回到正常范围的帧仍会被接受。
        const auto verdict = checkTarget(guard_limits_, calculatedPos, calculatedOri, robotCurrentPos, lastAcceptedPos);
        if (verdict != TargetVerdict::Accept)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[TEL-002] %s target dropped (%s): target (%.3f, %.3f, %.3f), end effector (%.3f, %.3f, %.3f), "
                                 "%.3f m from the end effector (limit %.3f), %.3f m from the previous target (limit %.3f)",
                                 arm.c_str(), toString(verdict), calculatedPos.x(), calculatedPos.y(), calculatedPos.z(),
                                 robotCurrentPos.x(), robotCurrentPos.y(), robotCurrentPos.z(),
                                 (calculatedPos - robotCurrentPos).norm(), guard_limits_.max_distance,
                                 lastAcceptedPos ? (calculatedPos - *lastAcceptedPos).norm() : 0.0, guard_limits_.max_step);
            return;
        }
        lastAcceptedPos = calculatedPos;
        // 只在位姿有明显变化时发布，避免目标话题刷屏
        if (!hasPoseChanged(calculatedPos, calculatedOri, prevPos, prevOri))
        {
            return;
        }
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                              "🕹️🕶️🕹️ %s XR Base: [%.3f, %.3f, %.3f] Current: [%.3f, %.3f, %.3f] -> Target: [%.3f, %.3f, %.3f]",
                              arm.c_str(), xrBasePos.x(), xrBasePos.y(), xrBasePos.z(), xrPos.x(), xrPos.y(), xrPos.z(),
                              calculatedPos.x(), calculatedPos.y(), calculatedPos.z());
        publishTargetPoseDirect(arm, calculatedPos, calculatedOri);
        prevPos = calculatedPos;
        prevOri = calculatedOri;
    }

    void XrInputHandler::recordTarget(std::uint32_t side, const geometry_msgs::msg::Pose& pose)
    {
        if (!flight_) { return; }
        flight_->record(robot_log::flight::kTelTargetOut,
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

    bool XrInputHandler::hasPoseChanged(const Eigen::Vector3d& currentPos,
                                        const Eigen::Quaterniond& currentOri,
                                        const Eigen::Vector3d& prevPos,
                                        const Eigen::Quaterniond& prevOri)
    {
        // 检查位置变化
        double positionDiff = (currentPos - prevPos).norm();
        if (positionDiff > POSITION_THRESHOLD)
        {
            return true;
        }

        // 使用四元数角度差检查方向变化
        double orientationDiff = std::abs(currentOri.angularDistance(prevOri));
        if (orientationDiff > ORIENTATION_THRESHOLD)
        {
            return true;
        }

        return false;
    }

    void XrInputHandler::calculatePoseFromDifference(const Eigen::Vector3d& xrCurrentPos,
                                                     const Eigen::Quaterniond& xrCurrentOri,
                                                     const Eigen::Vector3d& xrBasePos,
                                                     const Eigen::Quaterniond& xrBaseOri,
                                                     const Eigen::Quaterniond& xrBaseFrame,
                                                     const Eigen::Vector3d& robotBasePos,
                                                     const Eigen::Quaterniond& robotBaseOri,
                                                     Eigen::Vector3d& resultPos,
                                                     Eigen::Quaterniond& resultOri)
    {
        // 1. 计算XR位置在世界坐标系下的差值
        Eigen::Vector3d xrPosDiff_world = xrCurrentPos - xrBasePos;

        // 2. 将位置差值转换到位移坐标系（记录基准时由 motionFrame() 得到）：
        //    旧行为 = 进入UPDATE时手柄的姿态；头显 yaw 对齐 = 操作员面朝方向（只含 yaw）
        Eigen::Vector3d xrPosDiff_local = xrBaseFrame.inverse() * xrPosDiff_world;

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
            // 以下为停用的尺度校准（旧方案的握把+扳机组合键），保留待以后需要时恢复；它引用的部分成员已删除。
          //  case 13: // 左握把+左扳机组合键
          //  {
          //      // 取消镜像模式，确保尺度校准基于正常模式（左手→左臂，右手→右臂）
          //      mirror_mode_.store(false);
          //      RCLCPP_INFO(node_->get_logger(), "🔘 [左组合键] 已切换至非镜像模式");
          //      // 先切换到 STORAGE 模式，避免尺度更新期间发送错误目标
          //      is_update_mode_.store(false);
          //      RCLCPP_INFO(node_->get_logger(), "🔘 [左组合键] 自动切换到 STORAGE 模式 - 正在更新映射尺度，请重新进入 UPDATE 模式");

          //      // 计算 XR 头显与 XR 左手柄之间的 Z 轴距离（使用未缩放的原始左手柄位姿）
          //      double head_left_dist = std::abs(xr_head_position_.z() - xr_left_position_raw_.z());
          //      RCLCPP_INFO(node_->get_logger(),
          //                  "🕹️🕶️🕹️ [左组合键][校准调试] frame初始化=%s, ee_frame_id='%s', reference_link='%s'",
          //                  ee_frame_id_initialized_ ? "true" : "false",
          //                  ee_frame_id_.c_str(),
          //                  reference_link_.c_str());
          //      RCLCPP_INFO(node_->get_logger(),
          //                  "🕹️🕶️🕹️ [左组合键][校准调试] XR raw: head=[%.4f, %.4f, %.4f], "
          //                  "left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, %.4f], head_left_z_dist=%.6f",
          //                  xr_head_position_.x(), xr_head_position_.y(), xr_head_position_.z(),
          //                  xr_left_position_raw_.x(), xr_left_position_raw_.y(), xr_left_position_raw_.z(),
          //                  xr_right_position_raw_.x(), xr_right_position_raw_.y(), xr_right_position_raw_.z(),
          //                  head_left_dist);

          //      // 计算机器人左末端与 reference_link_ 之间的 Z 轴距离
          //      double left_ee_dist_from_ref = std::numeric_limits<double>::quiet_NaN();
          //      Eigen::Vector3d p_F_ref = Eigen::Vector3d::Zero();
          //      if (ee_frame_id_initialized_)
          //      {
          //          try
          //          {
          //              // 查询 reference_link_ 在 ee_frame_id_ 下的位置
          //              geometry_msgs::msg::TransformStamped tf_ref_in_F =
          //                  tf_buffer_->lookupTransform(
          //                      ee_frame_id_,      // target_frame: F
          //                      reference_link_,   // source_frame: ref
          //                      tf2::TimePointZero);

          //              p_F_ref = Eigen::Vector3d(
          //                  tf_ref_in_F.transform.translation.x,
          //                  tf_ref_in_F.transform.translation.y,
          //                  tf_ref_in_F.transform.translation.z);
          //              RCLCPP_INFO(node_->get_logger(),
          //                          "🕹️🕶️🕹️ [左组合键][校准调试] TF %s -> %s: translation=[%.4f, %.4f, %.4f], "
          //                          "rotation=[x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
          //                          reference_link_.c_str(), ee_frame_id_.c_str(),
          //                          p_F_ref.x(), p_F_ref.y(), p_F_ref.z(),
          //                          tf_ref_in_F.transform.rotation.x,
          //                          tf_ref_in_F.transform.rotation.y,
          //                          tf_ref_in_F.transform.rotation.z,
          //                          tf_ref_in_F.transform.rotation.w);

          //              // 取左末端 Z 轴距离
          //              left_ee_dist_from_ref = std::abs(left_current_position_.z() - p_F_ref.z());
          //              RCLCPP_INFO(node_->get_logger(),
          //                          "🕹️🕶️🕹️ [左组合键][校准调试] Robot current: left=[%.4f, %.4f, %.4f], "
          //                          "right=[%.4f, %.4f, %.4f], ref_in_%s=[%.4f, %.4f, %.4f], left_z_dist=%.6f",
          //                          left_current_position_.x(), left_current_position_.y(),
          //                          left_current_position_.z(),
          //                          right_current_position_.x(), right_current_position_.y(),
          //                          right_current_position_.z(),
          //                          ee_frame_id_.c_str(),
          //                          p_F_ref.x(), p_F_ref.y(), p_F_ref.z(),
          //                          left_ee_dist_from_ref);
          //          }
          //          catch (const tf2::TransformException& ex)
          //          {
          //              RCLCPP_WARN(node_->get_logger(),
          //                          "Failed to compute Z-dist(robot_left_ee, %s) in frame %s: %s",
          //                          reference_link_.c_str(), ee_frame_id_.c_str(), ex.what());
          //          }
          //      }
          //      else
          //      {
          //          RCLCPP_WARN(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键][校准调试] 无法查询TF：ee_frame_id_尚未从current_pose初始化");
          //      }

          //      // 用机器人Z轴距离除以XR Z轴距离，更新尺度
          //      if (!std::isnan(left_ee_dist_from_ref) && head_left_dist > 1e-6)
          //      {
          //          double unclamped_scale = left_ee_dist_from_ref / head_left_dist;
          //          xr_pose_scale_ = std::clamp(unclamped_scale, 0.75, 1.5);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键][校准调试] scale计算: robot_z_dist=%.6f / "
          //                      "xr_z_dist=%.6f => raw_scale=%.6f, clamped_scale=%.6f",
          //                      left_ee_dist_from_ref, head_left_dist, unclamped_scale, xr_pose_scale_);

          //          // XR左手柄相对头显的偏移 × scale = 左末端相对 reference_link_ 的目标偏移
          //          Eigen::Vector3d left_target_pos = p_F_ref + (xr_left_position_raw_ - xr_head_position_) * xr_pose_scale_;
          //          // XR右手柄相对头显的偏移 × scale = 右末端相对 reference_link_ 的目标偏移
          //          Eigen::Vector3d right_target_pos = p_F_ref + (xr_right_position_raw_ - xr_head_position_) * xr_pose_scale_;
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键][校准调试] XR offset raw: left-head=[%.4f, %.4f, %.4f], "
          //                      "right-head=[%.4f, %.4f, %.4f]",
          //                      (xr_left_position_raw_ - xr_head_position_).x(),
          //                      (xr_left_position_raw_ - xr_head_position_).y(),
          //                      (xr_left_position_raw_ - xr_head_position_).z(),
          //                      (xr_right_position_raw_ - xr_head_position_).x(),
          //                      (xr_right_position_raw_ - xr_head_position_).y(),
          //                      (xr_right_position_raw_ - xr_head_position_).z());

          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键] xr_pose_scale_ 已更新: "
          //                      "robot_z_dist=%.4f, xr_z_dist=%.4f, scale=%.4f",
          //                      left_ee_dist_from_ref, head_left_dist, xr_pose_scale_);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键] 左臂目标: [%.3f, %.3f, %.3f]  右臂目标: [%.3f, %.3f, %.3f]",
          //                      left_target_pos.x(), left_target_pos.y(), left_target_pos.z(),
          //                      right_target_pos.x(), right_target_pos.y(), right_target_pos.z());

          //          // 发布到 /dual_target/stamped
          //          nav_msgs::msg::Path dual_path;
          //          dual_path.header.stamp = node_->get_clock()->now();
          //          dual_path.header.frame_id = ee_frame_id_;
          //          dual_path.poses.resize(2);
          //          // 左臂（poses[0]）：位置用校准值，姿态保持机器人当前末端朝向
          //          dual_path.poses[0].header = dual_path.header;
          //          dual_path.poses[0].pose.position.x = left_target_pos.x();
          //          dual_path.poses[0].pose.position.y = left_target_pos.y();
          //          dual_path.poses[0].pose.position.z = left_target_pos.z();
          //          dual_path.poses[0].pose.orientation.x = left_current_orientation_.x();
          //          dual_path.poses[0].pose.orientation.y = left_current_orientation_.y();
          //          dual_path.poses[0].pose.orientation.z = left_current_orientation_.z();
          //          dual_path.poses[0].pose.orientation.w = left_current_orientation_.w();
          //          // 右臂（poses[1]）：位置用校准值，姿态保持机器人当前末端朝向
          //          dual_path.poses[1].header = dual_path.header;
          //          dual_path.poses[1].pose.position.x = right_target_pos.x();
          //          dual_path.poses[1].pose.position.y = right_target_pos.y();
          //          dual_path.poses[1].pose.position.z = right_target_pos.z();
          //          dual_path.poses[1].pose.orientation.x = right_current_orientation_.x();
          //          dual_path.poses[1].pose.orientation.y = right_current_orientation_.y();
          //          dual_path.poses[1].pose.orientation.z = right_current_orientation_.z();
          //          dual_path.poses[1].pose.orientation.w = right_current_orientation_.w();
          //          pub_dual_target_stamped_->publish(dual_path);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键] 已发布校准目标到 /dual_target/stamped (frame: %s)",
          //                      ee_frame_id_.c_str());
          //      }
          //      else
          //      {
          //          RCLCPP_WARN(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [左组合键] 无法更新 xr_pose_scale_: "
          //                      "robot_z_dist=%s, xr_z_dist=%.4f",
          //                      std::isnan(left_ee_dist_from_ref) ? "NaN" : std::to_string(left_ee_dist_from_ref).c_str(),
          //                      head_left_dist);
          //      }

          //      break;
          //  }
          // case 14: // 右握把+右扳机组合键
          //  {
          //      // 取消镜像模式，确保尺度校准基于正常模式（左手→左臂，右手→右臂）
          //      mirror_mode_.store(false);
          //      RCLCPP_INFO(node_->get_logger(), "🔘 [右组合键] 已切换至非镜像模式");
          //      // 先切换到 STORAGE 模式，避免尺度更新期间发送错误目标
          //      is_update_mode_.store(false);
          //      RCLCPP_INFO(node_->get_logger(), "🔘 [右组合键] 自动切换到 STORAGE 模式 - 正在更新映射尺度，请重新进入 UPDATE 模式");

          //      // 计算 XR 头显与 XR 右手柄之间的 Z 轴距离（使用未缩放的原始右手柄位姿）
          //      double head_right_dist = std::abs(xr_head_position_.z() - xr_right_position_raw_.z());
          //      RCLCPP_INFO(node_->get_logger(),
          //                  "🕹️🕶️🕹️ [右组合键][校准调试] frame初始化=%s, ee_frame_id='%s', reference_link='%s'",
          //                  ee_frame_id_initialized_ ? "true" : "false",
          //                  ee_frame_id_.c_str(),
          //                  reference_link_.c_str());
          //      RCLCPP_INFO(node_->get_logger(),
          //                  "🕹️🕶️🕹️ [右组合键][校准调试] XR raw: head=[%.4f, %.4f, %.4f], "
          //                  "left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, %.4f], head_right_z_dist=%.6f",
          //                  xr_head_position_.x(), xr_head_position_.y(), xr_head_position_.z(),
          //                  xr_left_position_raw_.x(), xr_left_position_raw_.y(), xr_left_position_raw_.z(),
          //                  xr_right_position_raw_.x(), xr_right_position_raw_.y(), xr_right_position_raw_.z(),
          //                  head_right_dist);

          //      // 计算机器人右末端与 reference_link_ 之间的 Z 轴距离
          //      double right_ee_dist_from_ref = std::numeric_limits<double>::quiet_NaN();
          //      Eigen::Vector3d p_F_ref = Eigen::Vector3d::Zero();
          //      if (ee_frame_id_initialized_)
          //      {
          //          try
          //          {
          //              // 查询 reference_link_ 在 ee_frame_id_ 下的位置
          //              geometry_msgs::msg::TransformStamped tf_ref_in_F =
          //                  tf_buffer_->lookupTransform(
          //                      ee_frame_id_,      // target_frame: F
          //                      reference_link_,   // source_frame: ref
          //                      tf2::TimePointZero);

          //              p_F_ref = Eigen::Vector3d(
          //                  tf_ref_in_F.transform.translation.x,
          //                  tf_ref_in_F.transform.translation.y,
          //                  tf_ref_in_F.transform.translation.z);
          //              RCLCPP_INFO(node_->get_logger(),
          //                          "🕹️🕶️🕹️ [右组合键][校准调试] TF %s -> %s: translation=[%.4f, %.4f, %.4f], "
          //                          "rotation=[x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
          //                          reference_link_.c_str(), ee_frame_id_.c_str(),
          //                          p_F_ref.x(), p_F_ref.y(), p_F_ref.z(),
          //                          tf_ref_in_F.transform.rotation.x,
          //                          tf_ref_in_F.transform.rotation.y,
          //                          tf_ref_in_F.transform.rotation.z,
          //                          tf_ref_in_F.transform.rotation.w);

          //              // 取右末端 Z 轴距离
          //              right_ee_dist_from_ref = std::abs(right_current_position_.z() - p_F_ref.z());
          //              RCLCPP_INFO(node_->get_logger(),
          //                          "🕹️🕶️🕹️ [右组合键][校准调试] Robot current: left=[%.4f, %.4f, %.4f], "
          //                          "right=[%.4f, %.4f, %.4f], ref_in_%s=[%.4f, %.4f, %.4f], right_z_dist=%.6f",
          //                          left_current_position_.x(), left_current_position_.y(),
          //                          left_current_position_.z(),
          //                          right_current_position_.x(), right_current_position_.y(),
          //                          right_current_position_.z(),
          //                          ee_frame_id_.c_str(),
          //                          p_F_ref.x(), p_F_ref.y(), p_F_ref.z(),
          //                          right_ee_dist_from_ref);
          //          }
          //          catch (const tf2::TransformException& ex)
          //          {
          //              RCLCPP_WARN(node_->get_logger(),
          //                          "Failed to compute Z-dist(robot_right_ee, %s) in frame %s: %s",
          //                          reference_link_.c_str(), ee_frame_id_.c_str(), ex.what());
          //          }
          //      }
          //      else
          //      {
          //          RCLCPP_WARN(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键][校准调试] 无法查询TF：ee_frame_id_尚未从current_pose初始化");
          //      }

          //      // 用机器人Z轴距离除以XR Z轴距离，更新尺度
          //      if (!std::isnan(right_ee_dist_from_ref) && head_right_dist > 1e-6)
          //      {
          //          double unclamped_scale = right_ee_dist_from_ref / head_right_dist;
          //          xr_pose_scale_ = std::clamp(unclamped_scale, 0.75, 1.5);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键][校准调试] scale计算: robot_z_dist=%.6f / "
          //                      "xr_z_dist=%.6f => raw_scale=%.6f, clamped_scale=%.6f",
          //                      right_ee_dist_from_ref, head_right_dist, unclamped_scale, xr_pose_scale_);

          //          // XR左手柄相对头显的偏移 × scale = 左末端相对 reference_link_ 的目标偏移
          //          Eigen::Vector3d left_target_pos = p_F_ref + (xr_left_position_raw_ - xr_head_position_) * xr_pose_scale_;
          //          // XR右手柄相对头显的偏移 × scale = 右末端相对 reference_link_ 的目标偏移
          //          Eigen::Vector3d right_target_pos = p_F_ref + (xr_right_position_raw_ - xr_head_position_) * xr_pose_scale_;
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键][校准调试] XR offset raw: left-head=[%.4f, %.4f, %.4f], "
          //                      "right-head=[%.4f, %.4f, %.4f]",
          //                      (xr_left_position_raw_ - xr_head_position_).x(),
          //                      (xr_left_position_raw_ - xr_head_position_).y(),
          //                      (xr_left_position_raw_ - xr_head_position_).z(),
          //                      (xr_right_position_raw_ - xr_head_position_).x(),
          //                      (xr_right_position_raw_ - xr_head_position_).y(),
          //                      (xr_right_position_raw_ - xr_head_position_).z());

          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键] xr_pose_scale_ 已更新: "
          //                      "robot_z_dist=%.4f, xr_z_dist=%.4f, scale=%.4f",
          //                      right_ee_dist_from_ref, head_right_dist, xr_pose_scale_);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键] 左臂目标: [%.3f, %.3f, %.3f]  右臂目标: [%.3f, %.3f, %.3f]",
          //                      left_target_pos.x(), left_target_pos.y(), left_target_pos.z(),
          //                      right_target_pos.x(), right_target_pos.y(), right_target_pos.z());

          //          // 发布到 /dual_target/stamped
          //          nav_msgs::msg::Path dual_path;
          //          dual_path.header.stamp = node_->get_clock()->now();
          //          dual_path.header.frame_id = ee_frame_id_;
          //          dual_path.poses.resize(2);
          //          // 左臂（poses[0]）：位置用校准值，姿态保持机器人当前末端朝向
          //          dual_path.poses[0].header = dual_path.header;
          //          dual_path.poses[0].pose.position.x = left_target_pos.x();
          //          dual_path.poses[0].pose.position.y = left_target_pos.y();
          //          dual_path.poses[0].pose.position.z = left_target_pos.z();
          //          dual_path.poses[0].pose.orientation.x = left_current_orientation_.x();
          //          dual_path.poses[0].pose.orientation.y = left_current_orientation_.y();
          //          dual_path.poses[0].pose.orientation.z = left_current_orientation_.z();
          //          dual_path.poses[0].pose.orientation.w = left_current_orientation_.w();
          //          // 右臂（poses[1]）：位置用校准值，姿态保持机器人当前末端朝向
          //          dual_path.poses[1].header = dual_path.header;
          //          dual_path.poses[1].pose.position.x = right_target_pos.x();
          //          dual_path.poses[1].pose.position.y = right_target_pos.y();
          //          dual_path.poses[1].pose.position.z = right_target_pos.z();
          //          dual_path.poses[1].pose.orientation.x = right_current_orientation_.x();
          //          dual_path.poses[1].pose.orientation.y = right_current_orientation_.y();
          //          dual_path.poses[1].pose.orientation.z = right_current_orientation_.z();
          //          dual_path.poses[1].pose.orientation.w = right_current_orientation_.w();
          //          pub_dual_target_stamped_->publish(dual_path);
          //          RCLCPP_INFO(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键] 已发布校准目标到 /dual_target/stamped (frame: %s)",
          //                      ee_frame_id_.c_str());

          //      }
          //      else
          //      {
          //          RCLCPP_WARN(node_->get_logger(),
          //                      "🕹️🕶️🕹️ [右组合键] 无法更新 xr_pose_scale_: "
          //                      "robot_z_dist=%s, xr_z_dist=%.4f",
          //                      std::isnan(right_ee_dist_from_ref) ? "NaN" : std::to_string(right_ee_dist_from_ref).c_str(),
          //                      head_right_dist);
          //      }

          //      break;
          //  }
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
            // 切换到更新模式：以当前手柄位姿和机器人位姿为基准，之后手柄的位移和转动驱动机器人
            captureLeftBase();
            captureRightBase();
            last_accepted_left_.reset();
            last_accepted_right_.reset();
            is_update_mode_.store(true);
            RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换跟随 - 操作: 切换到UPDATE模式（已存储基准位姿，双臂开始跟随）");
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                  "   XR Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                                  xr_base_left_position_.x(), xr_base_left_position_.y(), xr_base_left_position_.z(),
                                  xr_base_right_position_.x(), xr_base_right_position_.y(), xr_base_right_position_.z());
        }
        else
        {
            is_update_mode_.store(false);
            RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换跟随 - 操作: 切换到STORAGE模式（双臂停止跟随）");
        }
    }

    void XrInputHandler::fsmStateCallback(std_msgs::msg::Int32::SharedPtr msg)
    {
        const int32_t state = msg->data;
        if (state != kFsmHome && state != kFsmHold && state != kFsmOcs2 && state != kFsmMoveJ)
        {
            return;  // 0 = 控制器还没有有效状态
        }

        const int32_t old_state = current_fsm_state_.exchange(state);
        // 离开 OCS2 时自动切换到存储模式：再次进入 OCS2 后要重新按右摇杆才跟随，不会一进 OCS2 就跳
        if (state != old_state && state != kFsmOcs2 && is_update_mode_.load())
        {
            is_update_mode_.store(false);
            last_accepted_left_.reset();
            last_accepted_right_.reset();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ FSM状态不是OCS2，自动切换到STORAGE模式 (状态=%d)", state);
        }
    }

    void XrInputHandler::setAlignHeadYaw(bool enable)
    {
        align_head_yaw_ = enable;
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ 头显 yaw 对齐: %s",
                    enable ? "开启（操作员正前方 = 机器人 +X，不受房间原点/手柄倾角影响）"
                           : "关闭（沿用手柄基准姿态）");
    }

    void XrInputHandler::captureLeftBase()
    {
        xr_base_left_position_ = left_position_;
        xr_base_left_orientation_ = left_orientation_;
        xr_base_left_frame_ = motionFrame(align_head_yaw_, xr_head_orientation_, xr_base_left_orientation_);
        left_base_position_ = left_current_position_;
        left_base_orientation_ = left_current_orientation_;
    }

    void XrInputHandler::captureRightBase()
    {
        xr_base_right_position_ = right_position_;
        xr_base_right_orientation_ = right_orientation_;
        xr_base_right_frame_ = motionFrame(align_head_yaw_, xr_head_orientation_, xr_base_right_orientation_);
        right_base_position_ = right_current_position_;
        right_base_orientation_ = right_current_orientation_;
    }
} // namespace target_generator
