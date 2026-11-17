//
// Created for Arms ROS2 Control - TargetGenerator Node
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "target_generator/TargetGenerator.h"
#include "target_generator/ControlInputHandler.h"
#include "target_generator/XrInputHandler.h"
#include <robot_log/log_mode.hpp>

using namespace target_generator;

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = rclcpp::Node::make_shared("target_generator_node");
    robot_log::applyLogMode();  // debug mode of this run, applied once

    // 获取参数
    bool dual_arm_mode = node->declare_parameter("dual_arm_mode", false);
    std::string control_base_frame = node->declare_parameter("control_base_frame", "world");
    std::string marker_fixed_frame = node->declare_parameter("marker_fixed_frame", "base_link");
    std::string body_controller_name = node->declare_parameter("body_controller_name", "body_joint_controller");
    double linear_scale = node->declare_parameter("linear_scale", 0.005);
    double angular_scale = node->declare_parameter("angular_scale", 0.05);
    double xr_pose_scale = node->declare_parameter("xr_pose_scale", 1.0);
    // XR/头显参考link（通常为机器人头部link），可在各机器人target_generator.yaml中配置
    std::string reference_link = node->declare_parameter("reference_link", "head_link2");

    bool enable_xr = node->declare_parameter("enable_xr", true);

    // 获取 hand_controllers 参数（用于自动检测和验证）
    std::vector<std::string> hand_controllers = node->declare_parameter("hand_controllers", std::vector<std::string>());

    // 头部控制参数在 TargetGenerator::initialize() 中从配置文件读取

    // 自动检测双臂模式（基于 hand_controllers 数量）
    bool auto_detected_dual_arm = hand_controllers.size() > 1;
    
    // 验证配置一致性并记录控制器信息
    if (!hand_controllers.empty())
    {
        // 构建控制器列表字符串
        std::string controllers_list;
        for (size_t i = 0; i < hand_controllers.size(); ++i) {
            if (i > 0) controllers_list += ", ";
            controllers_list += hand_controllers[i];
        }

        if (dual_arm_mode != auto_detected_dual_arm)
        {
            RCLCPP_WARN(node->get_logger(),
                        "⚠️  Dual arm mode mismatch: parameter=%s, auto-detected=%s (from %zu hand controllers)",
                        dual_arm_mode ? "true" : "false",
                        auto_detected_dual_arm ? "true" : "false",
                        hand_controllers.size());
            RCLCPP_WARN(node->get_logger(),
                        "⚠️  Detected hand controllers: %s",
                        controllers_list.c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(),
                        "✅ Hand controllers detected: %zu controller(s) - %s mode",
                        hand_controllers.size(),
                        auto_detected_dual_arm ? "DUAL-ARM" : "SINGLE-ARM");
            RCLCPP_INFO(node->get_logger(), "   Controllers: %s", controllers_list.c_str());
        }
    }

    RCLCPP_INFO(node->get_logger(),
                "Starting TargetGenerator with dual_arm_mode: %s, control_base_frame: %s, marker_fixed_frame: %s, body_controller: %s",
                dual_arm_mode ? "true" : "false",
                control_base_frame.c_str(),
                marker_fixed_frame.c_str(),
                body_controller_name.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Joystick/keyboard scales: linear=%.3f, angular=%.3f (deadzone handled at input source)",
                linear_scale, angular_scale);

    RCLCPP_INFO(node->get_logger(),
            "XR control: enabled=%s",
            enable_xr ? "true" : "false");
    if (enable_xr)
    {
        RCLCPP_INFO(node->get_logger(),
                    "XR pose scale: %.3f (xr_pose_scale in target_generator.yaml)", xr_pose_scale);
        RCLCPP_INFO(node->get_logger(),
                    "XR reference_link: %s (from target_generator.yaml, default=head_link2)",
                    reference_link.c_str());
    }

    try
    {
        // 创建统一的目标位姿发布器（在所有模块之前创建，供 ArmMarker 和 XrInputHandler 共享）
        // 队列长度统一为 1，只关心最新的目标位姿
        auto pub_left_target = node->create_publisher<geometry_msgs::msg::Pose>("left_target", 1);
        auto pub_left_target_stamped = node->create_publisher<geometry_msgs::msg::PoseStamped>("left_target/stamped", 1);
        auto pub_right_target = node->create_publisher<geometry_msgs::msg::Pose>("right_target", 1);
        auto pub_right_target_stamped = node->create_publisher<geometry_msgs::msg::PoseStamped>("right_target/stamped", 1);

        RCLCPP_INFO(node->get_logger(), "✅ Created unified target pose publishers (queue_size=1)");

        auto target_generator = std::make_unique<TargetGenerator>(
            node, dual_arm_mode, control_base_frame, marker_fixed_frame,
            20.0, std::vector<int32_t>{3}, 0.05);

        // 初始化（传入统一的发布器）
        target_generator->initialize(pub_left_target, pub_left_target_stamped,
                                   pub_right_target, pub_right_target_stamped);

        // 创建ControlInputHandler（传入hand_controllers参数）
        auto control_handler = std::make_unique<ControlInputHandler>(
            node, target_generator.get(), linear_scale, angular_scale, hand_controllers);

        // 创建XrInputHandler（如果启用，传入统一的发布器、hand_controllers 及 target_generator.yaml 的 xr_pose_scale）
        std::unique_ptr<XrInputHandler> xr_handler = nullptr;
        if (enable_xr)
        {
            xr_handler = std::make_unique<XrInputHandler>(
                node, target_generator.get(), pub_left_target, pub_right_target, hand_controllers,
                xr_pose_scale, reference_link);
            // 头显 yaw 对齐（默认保持旧行为，见 XrInputHandler.h）
            xr_handler->setAlignHeadYaw(node->declare_parameter("xr_align_head_yaw", false));
        }

        // 创建 control input 订阅器（用于增量控制）
        auto control_subscription = node->create_subscription<robot_control_msgs::msg::Inputs>(
            "/teleop/control_input", 10,
            [control_handler_ptr = control_handler.get()](
            const robot_control_msgs::msg::Inputs::ConstSharedPtr msg)
            {
                // 处理增量控制
                if (control_handler_ptr)
                {
                    control_handler_ptr->processControlInput(msg);
                }
            });

        // 创建 FSM 命令订阅器（用于状态切换和 marker 形态更新）。
        // XrInputHandler 不再看命令，它自己订阅控制器实际发布的 /fsm_state。
        auto fsm_command_subscription = node->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10,
            [target_generator_ptr = target_generator.get()](const std_msgs::msg::Int32::ConstSharedPtr msg)
            {
                if (target_generator_ptr)
                {
                    target_generator_ptr->fsmCommandCallback(msg);
                }
            });

        // 设置 current_pose 回调（用于 XrInputHandler，通过 ArmMarker 的回调机制避免重复订阅）
        // ArmMarker 内部订阅了 current_pose，通过回调机制将原始消息转发给 XrInputHandler
        if (enable_xr && xr_handler)
        {
            // 左臂 current_pose 回调
            target_generator->setCurrentPoseCallback("left",
                [xr_handler_ptr = xr_handler.get()](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
                {
                    if (xr_handler_ptr)
                    {
                        // 转换为 SharedPtr 以匹配 XrInputHandler 的回调签名
                        auto shared_msg = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
                        xr_handler_ptr->robotLeftPoseCallback(shared_msg);
                    }
                });

            // 右臂 current_pose 回调（仅双臂模式）
            if (dual_arm_mode)
            {
                target_generator->setCurrentPoseCallback("right",
                    [xr_handler_ptr = xr_handler.get()](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
                    {
                        if (xr_handler_ptr)
                        {
                            // 转换为 SharedPtr 以匹配 XrInputHandler 的回调签名
                            auto shared_msg = std::const_pointer_cast<geometry_msgs::msg::PoseStamped>(msg);
                            xr_handler_ptr->robotRightPoseCallback(shared_msg);
                        }
                    });
            }
        }

        RCLCPP_INFO(node->get_logger(), "TargetGenerator is ready!");
        RCLCPP_INFO(node->get_logger(), "Use RViz to interact with the markers.");
        RCLCPP_INFO(node->get_logger(), "Control input handler is ready for joystick/keyboard control!");
        if (enable_xr)
        {
            RCLCPP_INFO(node->get_logger(), "XR input handler is ready for XR control!");
        }

        // 保持节点运行
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in TargetGenerator: %s", e.what());
        return 1;
    }

    // 清理
    rclcpp::shutdown();
    return 0;
}
