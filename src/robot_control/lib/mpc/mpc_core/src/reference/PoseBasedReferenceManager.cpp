//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#include "mpc_core/reference/PoseBasedReferenceManager.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <fstream>
#include <iomanip>
#include <limits>

#include "mpc_core/reference/BimanualCouplingSource.h"

namespace ocs2::mpc_core
{

    PoseBasedReferenceManager::PoseBasedReferenceManager(
        std::string topicPrefix,
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
        OCS2ReferenceTargetContext target_context)
        : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
          topic_prefix_(std::move(topicPrefix)),
          target_context_(std::move(target_context))
    {
        dual_arm_mode_ = target_context_.dual_arm;

        // 获取base frame
        base_frame_ = target_context_.base_frame;

        // 初始化target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
        body_pose_7_xyzw_ = vector_t::Zero(7);
        body_pose_7_xyzw_(6) = 1.0; // identity quaternion (geometry_msgs order)
        head_pose_7_xyzw_ = vector_t::Zero(7);
        head_pose_7_xyzw_(6) = 1.0;

        // 初始化圆弧指针
#ifdef HAS_MOTION_PLANNING
        left_circle_curve_ = std::make_shared<planning::CircularCurver>();
        right_circle_curve_ = std::make_shared<planning::CircularCurver>();
        // 初始化Service状态
        left_service_state_.arm_name = "left";
        left_service_state_.curve = left_circle_curve_;

        if (dual_arm_mode_)
        {
            right_service_state_.arm_name = "right";
            right_service_state_.curve = right_circle_curve_;
        }
#endif
    }

    void PoseBasedReferenceManager::setAcceptingTargets(bool accept)
    {
        accepting_targets_.store(accept, std::memory_order_release);
        if (!accept)
        {
            left_twist_active_ = false;
            right_twist_active_ = false;
        }
    }

    void PoseBasedReferenceManager::setStampedTargetHandlers(
        PoseStampedHandler left,
        PoseStampedHandler right,
        DualStampedHandler dual)
    {
        stamped_left_handler_ = std::move(left);
        stamped_right_handler_ = std::move(right);
        stamped_dual_handler_ = std::move(dual);
    }

    std::shared_ptr<tf2_ros::Buffer> PoseBasedReferenceManager::getTfBuffer() const
    {
        return tf_buffer_;
    }

    void PoseBasedReferenceManager::setupTfBuffer(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

        // Prefer controller-declared tf_topic (WholeBodyMpcTask); else declare defaults.
        std::string tf_topic = "tf";
        std::string tf_static_topic = "tf_static";
        if (!node->has_parameter("tf_topic"))
        {
            node->declare_parameter("tf_topic", tf_topic);
        }
        if (!node->has_parameter("tf_static_topic"))
        {
            node->declare_parameter("tf_static_topic", tf_static_topic);
        }
        tf_topic = node->get_parameter("tf_topic").as_string();
        tf_static_topic = node->get_parameter("tf_static_topic").as_string();

        tf_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic, rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tfMsgCallback(msg, false); });
        tf_static_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_static_topic, rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tfMsgCallback(msg, true); });

        RCLCPP_INFO(logger_,
                    "PoseBasedReferenceManager TF buffer on relative topics '%s' + '%s' "
                    "(multi-robot: resolves under node namespace, not absolute /tf)",
                    tf_topic.c_str(), tf_static_topic.c_str());
    }

    void PoseBasedReferenceManager::tfMsgCallback(
        const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
    {
        if (!tf_buffer_ || !msg)
        {
            return;
        }
        for (const auto& t : msg->transforms)
        {
            try
            {
                tf_buffer_->setTransform(t, "isaac_tf_authority", is_static);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "setTransform failed: %s", ex.what());
            }
        }
    }

    void PoseBasedReferenceManager::subscribe(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
        // 保存node引用用于访问参数服务器
        node_ = node;

        // 保存node的logger用于后续日志输出
        logger_ = node->get_logger();

        // 保存clock用于时间戳
        clock_ = node->get_clock();

        // 初始化参数（从参数服务器获取）
        updateParam();

        setupTfBuffer(node);

        // 左臂pose订阅者（单臂机器人也使用这个）
        auto leftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            leftPoseCallback(msg);
        };
        left_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
            "left_target", 1, leftCallback);

        // 右臂pose订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
            {
                rightPoseCallback(msg);
            };
            right_pose_subscriber_ =
                node->create_subscription<geometry_msgs::msg::Pose>("right_target", 1,
                                                                    rightCallback);
        }

        // 左臂PoseStamped订阅者（单臂机器人也使用这个）
        auto leftStampedCallback =
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            leftPoseStampedCallback(msg);
        };
        left_pose_stamped_subscriber_ =
            node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "left_target/stamped", 1, leftStampedCallback);

        // 左臂速度流（手柄）：latch + dt 积分，立即更新目标
        left_twist_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
            "left_target/twist", 1,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { leftTwistCallback(msg); });

        // 左臂一次相对位移 + moveL（TwistStamped：frame_id 区分基座/末端等）
        left_relative_subscriber_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
            "left_target/relative", 1,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { leftRelativeCallback(msg); });

        // 右臂PoseStamped订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightStampedCallback =
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                rightPoseStampedCallback(msg);
            };
            right_pose_stamped_subscriber_ =
                node->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "right_target/stamped", 1, rightStampedCallback);

            right_twist_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
                "right_target/twist", 1,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) { rightTwistCallback(msg); });

            right_relative_subscriber_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                "right_target/relative", 1,
                [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { rightRelativeCallback(msg); });
        }
        if (target_context_.body_target_enabled)
        {
            body_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
                "body_target", 1,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyPoseCallback(msg); });
            body_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "body_target/stamped", 1,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { bodyPoseStampedCallback(msg); });
        }
        if (target_context_.head_target_enabled)
        {
            head_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
                "head_target", 1,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) { headPoseCallback(msg); });
            head_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "head_target/stamped", 1,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { headPoseStampedCallback(msg); });
        }
        // 删除圆弧订阅者，添加Service
#ifdef HAS_MOTION_PLANNING
        // 左臂圆弧Service
        auto leftServiceCallback =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Request> request,
                   std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Response> response)
        {
            this->handleLeftCircleService(request_header, request, response);
        };

        left_circle_service_ = node->create_service<robot_control_msgs::srv::ExecuteCircle>(
            "execute_left_circle",
            leftServiceCallback);

        // 右臂圆弧Service（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightServiceCallback =
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Request> request,
                       std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Response> response)
            {
                this->handleRightCircleService(request_header, request, response);
            };

            right_circle_service_ = node->create_service<robot_control_msgs::srv::ExecuteCircle>(
                "execute_right_circle",
                rightServiceCallback);
        }
#else
        RCLCPP_WARN(logger_,
                    "motion_planning not available, circle execution services are disabled");
#endif

        // 创建执行定时器
        // execution_timer_ = node->create_wall_timer(
        //     std::chrono::milliseconds(40), // 25Hz，与之前的采样率一致
        //     [this]() { this->executionTimerCallback(); });

        // 双目标PoseStamped订阅者（仅双臂机器人，用于同时更新左右两个目标）
        // Path长度为2，第一个是左臂，第二个是右臂
        if (dual_arm_mode_)
        {
            auto dualTargetStampedCallback =
                [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                this->dualTargetStampedCallback(msg);
            };
            dual_target_stamped_subscriber_ =
                node->create_subscription<nav_msgs::msg::Path>(
                    "dual_target/stamped", 1, dualTargetStampedCallback);
        }

        // Path订阅者（单臂和双臂机器人都支持）
        auto pathCallback = [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
            this->pathCallback(msg);
        };
        path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
            "target_path", 1, pathCallback);

        // ExecutePath服务
        auto executePathCallback =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<robot_control_msgs::srv::ExecutePath::Request> request,
                   std::shared_ptr<robot_control_msgs::srv::ExecutePath::Response> response)
        {
            (void)request_header;
            this->handleExecutePathService(request, response);
        };
        execute_path_service_ = node->create_service<robot_control_msgs::srv::ExecutePath>(
            "execute_path", executePathCallback);

        // 初始化发布器：发布当前目标
        left_target_publisher_ =
            node->create_publisher<geometry_msgs::msg::PoseStamped>(
                "left_current_target", 1);

        if (dual_arm_mode_)
        {
            right_target_publisher_ =
                node->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "right_current_target", 1);
        }

        if (target_context_.body_target_enabled)
        {
            body_target_publisher_ =
                node->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "body_current_target", 1);
            body_relative_subscriber_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                "body_target/relative", 1,
                [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                    bodyRelativeCallback(msg);
                });
        }
        if (target_context_.head_target_enabled)
        {
            head_target_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
                "head_current_target", 1);
        }
    }

    void PoseBasedReferenceManager::publishCurrentTargetsFromCache()
    {
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::setCurrentObservation(
        const SystemObservation& observation)
    {
        clearTwistLatchIfTimedOut();

        double dt = 0.0;
        if (has_observation_time_)
        {
            dt = observation.time - last_observation_time_;
        }
        current_observation_ = observation;
        last_observation_time_ = observation.time;
        has_observation_time_ = true;

        if (dt > 0.0 && dt < 1.0)
        {
            integrateLatchedTwists(dt);
        }
    }

    void PoseBasedReferenceManager::updateParam()
    {
        trajectory_duration_ = std::max(0.0,
            node_->get_parameter("movel_trajectory_duration").as_double());
        moveL_duration_ = std::max(0.0, node_->get_parameter("movel_duration").as_double());
        moveL_sample_interval_ = std::max(1e-3, node_->get_parameter("movel_sample_interval").as_double());
        moveL_max_linear_velocity_ = std::max(1e-6, node_->get_parameter("movel_max_linear_velocity").as_double());
        moveL_max_linear_acceleration_ = std::max(1e-6, node_->get_parameter("movel_max_linear_acceleration").as_double());
        moveL_max_linear_jerk_ = std::max(1e-6, node_->get_parameter("movel_max_linear_jerk").as_double());
        moveL_max_angular_velocity_ = std::max(1e-6, node_->get_parameter("movel_max_angular_velocity").as_double());
        moveL_max_angular_acceleration_ = std::max(1e-6, node_->get_parameter("movel_max_angular_acceleration").as_double());
        moveL_max_angular_jerk_ = std::max(1e-6, node_->get_parameter("movel_max_angular_jerk").as_double());
        moveL_auto_extend_duration_ = node_->get_parameter("movel_auto_extend_duration").as_bool();
    }

    void PoseBasedReferenceManager::resetTargetStateCache()
    {
        // 重置target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
        body_pose_7_xyzw_ = vector_t::Zero(7);
        body_pose_7_xyzw_(6) = 1.0;
        head_pose_7_xyzw_ = vector_t::Zero(7);
        head_pose_7_xyzw_(6) = 1.0;
        left_arm_reference_buffer_ = ArmReferenceBuffer{};
        right_arm_reference_buffer_ = ArmReferenceBuffer{};
        body_reference_buffer_ = ArmReferenceBuffer{};
        head_reference_buffer_ = ArmReferenceBuffer{};

        RCLCPP_INFO(logger_,
                    "Target state cache reset - cleared all cached target states");
    }

    void PoseBasedReferenceManager::setCurrentEndEffectorPoses(
        const vector_t& left_ee_pose, const vector_t& right_ee_pose,
        bool update_target_trajectory)
    {
        const double t = current_observation_.time;
        const bool left_buffer_active = isArmReferenceBufferActive(left_arm_reference_buffer_, t);
        const bool right_buffer_active = isArmReferenceBufferActive(right_arm_reference_buffer_, t);

        if (!left_buffer_active)
        {
            left_target_state_ = left_ee_pose;
        }

        // 如果是双臂模式，设置右臂pose到缓存
        if (dual_arm_mode_)
        {
            if (!right_buffer_active)
            {
                right_target_state_ = right_ee_pose;
            }
            // Optional wheel-humanoid pose blocks require rebuilding the full explicit layout.
            if (update_target_trajectory &&
                isWholeBodyTargetLayout() &&
                !anyReferenceBufferActive(t))
            {
                updateTargetTrajectory();
            }
        }

        // 始终发布当前目标（保证 marker 同步刷新）
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::resetLeftEndEffectorTarget(
        const vector_t& pose, bool update_target_trajectory)
    {
        if (pose.size() != 7)
        {
            RCLCPP_ERROR(logger_, "left EE target must have 7 elements");
            return;
        }
        left_target_state_ = pose;
        resetArmReferenceBuffer(left_arm_reference_buffer_, pose, current_observation_.time);
        if (update_target_trajectory &&
            isWholeBodyTargetLayout())
        {
            updateTargetTrajectory();
        }
        publishCurrentTargets("left");
    }

    void PoseBasedReferenceManager::resetRightEndEffectorTarget(
        const vector_t& pose, bool update_target_trajectory)
    {
        if (pose.size() != 7)
        {
            RCLCPP_ERROR(logger_, "right EE target must have 7 elements");
            return;
        }
        right_target_state_ = pose;
        resetArmReferenceBuffer(right_arm_reference_buffer_, pose, current_observation_.time);
        if (update_target_trajectory &&
            isWholeBodyTargetLayout())
        {
            updateTargetTrajectory();
        }
        publishCurrentTargets("right");
    }

    void PoseBasedReferenceManager::publishCurrentTargets(
        const std::string& target_type)
    {
        const bool publish_left =
            (target_type.empty() || target_type == "both" || target_type == "left");
        const bool publish_right =
            (target_type.empty() || target_type == "both" || target_type == "right");
        const bool publish_body =
            (target_type.empty() || target_type == "both" || target_type == "body");
        const bool publish_head =
            (target_type.empty() || target_type == "both" || target_type == "head");

        // 发布左臂当前目标
        if (publish_left)
        {
            geometry_msgs::msg::PoseStamped left_target_msg;
            left_target_msg.header.stamp = clock_->now();
            left_target_msg.header.frame_id = base_frame_;
            left_target_msg.pose.position.x = left_target_state_(0);
            left_target_msg.pose.position.y = left_target_state_(1);
            left_target_msg.pose.position.z = left_target_state_(2);
            left_target_msg.pose.orientation.x = left_target_state_(3);
            left_target_msg.pose.orientation.y = left_target_state_(4);
            left_target_msg.pose.orientation.z = left_target_state_(5);
            left_target_msg.pose.orientation.w = left_target_state_(6);
            left_target_publisher_->publish(left_target_msg);
        }

        // 发布右臂当前目标（仅双臂模式）
        if (dual_arm_mode_ && publish_right)
        {
            geometry_msgs::msg::PoseStamped right_target_msg;
            right_target_msg.header.stamp = clock_->now();
            right_target_msg.header.frame_id = base_frame_;
            right_target_msg.pose.position.x = right_target_state_(0);
            right_target_msg.pose.position.y = right_target_state_(1);
            right_target_msg.pose.position.z = right_target_state_(2);
            right_target_msg.pose.orientation.x = right_target_state_(3);
            right_target_msg.pose.orientation.y = right_target_state_(4);
            right_target_msg.pose.orientation.z = right_target_state_(5);
            right_target_msg.pose.orientation.w = right_target_state_(6);
            right_target_publisher_->publish(right_target_msg);
        }

        if (target_context_.body_target_enabled && publish_body && body_target_publisher_)
        {
            geometry_msgs::msg::PoseStamped body_target_msg;
            body_target_msg.header.stamp = clock_->now();
            body_target_msg.header.frame_id = base_frame_;
            body_target_msg.pose.position.x = body_pose_7_xyzw_(0);
            body_target_msg.pose.position.y = body_pose_7_xyzw_(1);
            body_target_msg.pose.position.z = body_pose_7_xyzw_(2);
            body_target_msg.pose.orientation.x = body_pose_7_xyzw_(3);
            body_target_msg.pose.orientation.y = body_pose_7_xyzw_(4);
            body_target_msg.pose.orientation.z = body_pose_7_xyzw_(5);
            body_target_msg.pose.orientation.w = body_pose_7_xyzw_(6);
            body_target_publisher_->publish(body_target_msg);
        }

        if (target_context_.head_target_enabled && publish_head && head_target_publisher_)
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = clock_->now();
            msg.header.frame_id = base_frame_;
            msg.pose.position.x = head_pose_7_xyzw_(0);
            msg.pose.position.y = head_pose_7_xyzw_(1);
            msg.pose.position.z = head_pose_7_xyzw_(2);
            msg.pose.orientation.x = head_pose_7_xyzw_(3);
            msg.pose.orientation.y = head_pose_7_xyzw_(4);
            msg.pose.orientation.z = head_pose_7_xyzw_(5);
            msg.pose.orientation.w = head_pose_7_xyzw_(6);
            head_target_publisher_->publish(msg);
        }
    }

    // 新增圆形轨迹相关函数

} // namespace ocs2::mpc_core
