//
// PoseBasedReferenceManager: circular-arc services (needs motion_planning)
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

    bool PoseBasedReferenceManager::initCircleCurve(
        vector_t start_pose,
        robot_control_msgs::msg::CircleMessage::SharedPtr msg,
        std::shared_ptr<planning::CircularCurver> circle_ptr)
    {
        if (start_pose.size() < 7)
        {
            RCLCPP_WARN(logger_,
                        "No starting point provided,please update current point!");
            return false;
        }
        // 更新起点
        planning::TrajectPoint ps;
        ps.cart_pos(0) = start_pose(0);
        ps.cart_pos(1) = start_pose(1);
        ps.cart_pos(2) = start_pose(2);
        ps.quaternion_point.q.x = start_pose(3);
        ps.quaternion_point.q.y = start_pose(4);
        ps.quaternion_point.q.z = start_pose(5);
        ps.quaternion_point.q.w = start_pose(6);
        // 更新终点，无论是哪种计算方式都给一个终点，终点只有在直接给值且在姿态不是slerp插值的时候才不会用到，这时候可以给任意值
        planning::TrajectPoint pe;
        pe.cart_pos(0) = msg->endpoint.position.x;
        pe.cart_pos(1) = msg->endpoint.position.y;
        pe.cart_pos(2) = msg->endpoint.position.z;
        pe.quaternion_point.q.x = msg->endpoint.orientation.x;
        pe.quaternion_point.q.y = msg->endpoint.orientation.y;
        pe.quaternion_point.q.z = msg->endpoint.orientation.z;
        pe.quaternion_point.q.w = msg->endpoint.orientation.w;
        // 初始化圆弧参数
        if (msg->time_mode)
        {
            // 时间模式
            if (msg->duration < 0.001)
            {
                RCLCPP_WARN(logger_,
                            "The time mode does not have the exercise time inputted. "
                            "Please input the time or change the exercise mode");
                return false;
            }
            planning::TrajectoryParameter time_mode_para(msg->use_slerp_for_orientation,
                                                         msg->duration);

            if (fabs(msg->max_linear_velocity) > min_val &&
                fabs(msg->max_linear_acceleration) > min_val &&
                fabs(msg->max_linear_jerk) > min_val &&
                fabs(msg->max_angular_velocity) > min_val &&
                fabs(msg->max_angular_acceleration) > min_val &&
                fabs(msg->max_angular_jerk) > min_val)
            {
                // 如果用户设置了参数
                time_mode_para.max_linear_vel = msg->max_linear_velocity;
                time_mode_para.max_linear_acc = msg->max_linear_acceleration;
                time_mode_para.max_linear_jerk = msg->max_linear_jerk;
                time_mode_para.max_angular_vel = msg->max_angular_velocity;
                time_mode_para.max_angular_acc = msg->max_angular_acceleration;
                time_mode_para.max_angular_jerk = msg->max_angular_jerk;
            }

            if (!msg->use_three_point_method)
            {
                // 如果是直接输入参数
                time_mode_para.circular_rotation_axis(0) = msg->axis.x;
                time_mode_para.circular_rotation_axis(1) = msg->axis.y;
                time_mode_para.circular_rotation_axis(2) = msg->axis.z;

                time_mode_para.center_of_circle(0) = msg->center.x;
                time_mode_para.center_of_circle(1) = msg->center.y;
                time_mode_para.center_of_circle(2) = msg->center.z;

                time_mode_para.circular_angle = msg->rotate_angle;
                time_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
                planning::TrajectoryInitParameters init_para(ps, pe, time_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
            else
            {
                // 三点计算
                planning::TrajectPoint pm;
                pm.cart_pos(0) = msg->midpoint.position.x;
                pm.cart_pos(1) = msg->midpoint.position.y;
                pm.cart_pos(2) = msg->midpoint.position.z;
                pm.quaternion_point.q.x = msg->midpoint.orientation.x;
                pm.quaternion_point.q.y = msg->midpoint.orientation.y;
                pm.quaternion_point.q.z = msg->midpoint.orientation.z;
                pm.quaternion_point.q.w = msg->midpoint.orientation.w;
                planning::TrajectoryInitParameters init_para(ps, pm, pe, time_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
        }
        else
        {
            // 参数模式
            planning::TrajectoryParameter speed_mode_para(
                msg->max_linear_velocity, msg->max_linear_acceleration,
                msg->max_linear_jerk, msg->max_angular_velocity,
                msg->max_angular_acceleration, msg->max_angular_jerk,
                msg->use_slerp_for_orientation);
            if (!msg->use_three_point_method)
            {
                // 如果是直接输入参数
                speed_mode_para.circular_rotation_axis(0) = msg->axis.x;
                speed_mode_para.circular_rotation_axis(1) = msg->axis.y;
                speed_mode_para.circular_rotation_axis(2) = msg->axis.z;

                speed_mode_para.center_of_circle(0) = msg->center.x;
                speed_mode_para.center_of_circle(1) = msg->center.y;
                speed_mode_para.center_of_circle(2) = msg->center.z;

                speed_mode_para.circular_angle = msg->rotate_angle;
                speed_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
                planning::TrajectoryInitParameters init_para(ps, pe, speed_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
            else
            {
                // 三点计算
                planning::TrajectPoint pm;
                pm.cart_pos(0) = msg->midpoint.position.x;
                pm.cart_pos(1) = msg->midpoint.position.y;
                pm.cart_pos(2) = msg->midpoint.position.z;
                pm.quaternion_point.q.x = msg->midpoint.orientation.x;
                pm.quaternion_point.q.y = msg->midpoint.orientation.y;
                pm.quaternion_point.q.z = msg->midpoint.orientation.z;
                pm.quaternion_point.q.w = msg->midpoint.orientation.w;
                planning::TrajectoryInitParameters init_para(ps, pm, pe, speed_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
        }

        return true;
    };

    // 使用已有的变换转换位姿
    static geometry_msgs::msg::Pose
    transformPose(const geometry_msgs::msg::Pose& pose,
                  const geometry_msgs::msg::TransformStamped& transform)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header.frame_id = transform.header.frame_id;
        pose_stamped.header.stamp = transform.header.stamp;

        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(pose_stamped, transformed, transform);

        return transformed.pose;
    };

    // 转换向量（只旋转不平移）
    static geometry_msgs::msg::Vector3
    transformVector3(const geometry_msgs::msg::Vector3& vector,
                     const geometry_msgs::msg::TransformStamped& transform)
    {
        geometry_msgs::msg::Vector3Stamped vector_in, vector_out;
        vector_in.vector = vector;
        vector_in.header.frame_id = transform.header.frame_id;
        vector_in.header.stamp = transform.header.stamp;

        tf2::doTransform(vector_in, vector_out, transform);
        return vector_out.vector;
    };

    void PoseBasedReferenceManager::transCircleMessageToBaseFrame(const
                                                                  robot_control_msgs::msg::CircleMessage& msg,
                                                                  robot_control_msgs::msg::CircleMessage::SharedPtr
                                                                  base_msg)
    {
        // 把输入的圆弧参数转换成base_frame
        if (msg.frame_id == base_frame_)
        {
            *base_msg = msg;
            return;
        }
        // 如果不是极坐标系，需要对其中的参数进行转换
        try
        {
            // 复制所有的参数
            *base_msg = msg;

            // 获取变换
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, msg.frame_id,
                                            tf2::TimePointZero);
            // 根据不同的方法转换不同的数据
            if (msg.use_three_point_method)
            {
                // 转换三点法中的两个点
                base_msg->midpoint = transformPose(msg.midpoint, transform);
                base_msg->endpoint = transformPose(msg.endpoint, transform);
            }
            else
            {
                // 转换参数法中的圆心
                geometry_msgs::msg::Pose center_pose;
                center_pose.position = msg.center;
                center_pose.orientation = geometry_msgs::msg::Quaternion();
                auto transformed_center = transformPose(center_pose, transform);
                base_msg->center = transformed_center.position;

                // 转换旋转轴（只旋转不平移）
                base_msg->axis = transformVector3(msg.axis, transform);
            }
            // 更新坐标系ID
            base_msg->frame_id = base_frame_;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_, "无法将pose从 %s 转换到 %s: %s", msg.frame_id.c_str(),
                        base_frame_.c_str(), ex.what());
        }
    };

    void PoseBasedReferenceManager::handleLeftCircleService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Request> request,
        std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Response> response)
    {
        (void)request_header; // 不使用

        if (!armExternalCommandAllowed(true))
        {
            response->success = false;
            response->message = "left EE command ignored while the arm is disabled or enable has not released";
            warnBlockedArmExternalCommand(true);
            return;
        }

        RCLCPP_INFO(logger_, "Received left arm arc execution request");
        // 获取起始状态
        const vector_t start_state = left_target_state_;
        // 验证请求
        std::string error_message;
        if (!validateCircleRequest(start_state, request, error_message))
        {
            response->success = false;
            response->message = "Request verification failed: " + error_message;
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }

        // 转换坐标系
        robot_control_msgs::msg::CircleMessage::SharedPtr base_frame_msg =
            std::make_shared<robot_control_msgs::msg::CircleMessage>();
        transCircleMessageToBaseFrame(request->circle_params, base_frame_msg);

        // 开始执行
        startServiceExecution(left_service_state_, start_state, *base_frame_msg, "left");
        sendPlannedTrajectoryToOCS2(left_service_state_.time_trajectory, left_service_state_.state_trajectory, "left");

        response->success = true;
        response->message = "The left arm starts to execute a circular arc trajectory, with an estimated time of: " +
            std::to_string(left_service_state_.total_duration) + "s";
        response->estimated_duration = left_service_state_.total_duration;

        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    void PoseBasedReferenceManager::handleRightCircleService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Request> request,
        std::shared_ptr<robot_control_msgs::srv::ExecuteCircle::Response> response)
    {
        (void)request_header; // 不使用

        if (!dual_arm_mode_)
        {
            response->success = false;
            response->message = "The robot is not in a dual-arm mode and does not support right arm arc";
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }
        if (!armExternalCommandAllowed(false))
        {
            response->success = false;
            response->message = "right EE command ignored while the arm is disabled or enable has not released";
            warnBlockedArmExternalCommand(false);
            return;
        }

        RCLCPP_INFO(logger_, "Received right arm arc execution request");
        // 获取起始状态
        const vector_t start_state = right_target_state_;
        // 验证请求
        std::string error_message;
        if (!validateCircleRequest(start_state, request, error_message))
        {
            response->success = false;
            response->message = "Request verification failed: " + error_message;
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }

        // 转换坐标系
        robot_control_msgs::msg::CircleMessage::SharedPtr base_frame_msg =
            std::make_shared<robot_control_msgs::msg::CircleMessage>();
        transCircleMessageToBaseFrame(request->circle_params, base_frame_msg);

        // 开始执行
        startServiceExecution(right_service_state_, start_state, *base_frame_msg, "right");
        sendPlannedTrajectoryToOCS2(right_service_state_.time_trajectory, right_service_state_.state_trajectory,
                                    "right");
        response->success = true;
        response->message = "The right arm starts to execute a circular arc trajectory, with an estimated time of: " +
            std::to_string(right_service_state_.total_duration) + "s";
        response->estimated_duration = right_service_state_.total_duration;

        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    bool PoseBasedReferenceManager::validateCircleRequest(vector_t start_pose,
                                                          const
                                                          robot_control_msgs::srv::ExecuteCircle::Request::SharedPtr
                                                          request,
                                                          std::string& error_message)
    {
        const auto& msg = request->circle_params;
        //检查起点的位姿
        if (start_pose.size() < 7)
        {
            error_message =
                "The starting point of the arc has not been entered. Please update the starting point of the arc！";
            return false;
        }
        bool all_start_pose_is_zero = true;
        for (int i = 0; i < 7; i++)
        {
            if (fabs(start_pose(i)) > min_val)
            {
                all_start_pose_is_zero = false;
            }
            if (std::isnan(start_pose(i)))
            {
                error_message = "Please check the starting point！";
                return false;
            }
        }
        if (all_start_pose_is_zero)
        {
            error_message = "Please check the starting point！";
            return false;
        }

        // 检查时间模式下的时间参数
        if (msg.time_mode && msg.duration < 0.001)
        {
            error_message = "In time mode, a valid time parameter (duration > 0.001) must be provided";
            return false;
        }

        // 检查速度模式下的速度参数
        if (!msg.time_mode)
        {
            if (fabs(msg.max_linear_velocity) < min_val)
            {
                error_message = "In speed mode, the maximum linear speed must be provided";
                return false;
            }
            if (fabs(msg.max_angular_velocity) < min_val)
            {
                error_message =
                    "When using attitude interpolation in velocity mode, the maximum angular velocity must be provided";
                return false;
            }
            if (fabs(msg.max_linear_acceleration) < min_val)
            {
                error_message = "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_angular_acceleration) < min_val)
            {
                error_message =
                    "When using pose interpolation in velocity mode, the maximum angular acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_linear_jerk) < min_val)
            {
                error_message = "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_angular_jerk) < min_val)
            {
                error_message =
                    "When using attitude interpolation in velocity mode, the maximum angular jerk must be provided";
                return false;
            }
        }

        if (msg.use_three_point_method)
        {
            //检查输入的中间点和终点是不是有问题
            if (fabs(msg.midpoint.position.x) < min_val && fabs(msg.midpoint.position.y) < min_val && fabs(
                    msg.midpoint.position.z) < min_val && fabs(msg.midpoint.orientation.x) < min_val &&
                fabs(msg.midpoint.orientation.y) < min_val && fabs(msg.midpoint.orientation.z) < min_val && fabs(
                    msg.midpoint.orientation.w) < min_val)
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(msg.midpoint.position.x) || std::isnan(msg.midpoint.position.y) || std::isnan(
                msg.midpoint.position.z) || std::isnan(msg.midpoint.orientation.x) || std::isnan(
                msg.midpoint.orientation.y) || std::isnan(msg.midpoint.orientation.z) || std::isnan(
                msg.midpoint.orientation.w))
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (fabs(msg.endpoint.position.x) < min_val && fabs(msg.endpoint.position.y) < min_val && fabs(
                    msg.endpoint.position.z) < min_val && fabs(msg.endpoint.orientation.x) < min_val &&
                fabs(msg.endpoint.orientation.y) < min_val && fabs(msg.endpoint.orientation.z) < min_val && fabs(
                    msg.endpoint.orientation.w) < min_val)
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(msg.endpoint.position.x) || std::isnan(msg.endpoint.position.y) || std::isnan(
                msg.endpoint.position.z) || std::isnan(msg.endpoint.orientation.x) || std::isnan(
                msg.endpoint.orientation.y) || std::isnan(msg.endpoint.orientation.z) || std::isnan(
                msg.endpoint.orientation.w))
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }

            if (fabs(msg.midpoint.position.x - start_pose(0)) < min_val && fabs(msg.midpoint.position.y - start_pose(1))
                < min_val && fabs(msg.midpoint.position.z - start_pose(2)) < min_val && fabs(
                    msg.midpoint.orientation.x - start_pose(3)) < min_val && fabs(
                    msg.midpoint.orientation.y - start_pose(4)) < min_val && fabs(
                    msg.midpoint.orientation.z - start_pose(5)) < min_val && fabs(
                    msg.midpoint.orientation.w - start_pose(6)) < min_val)
            {
                error_message = "The starting point and the middle point of the arc are equal！";
                return false;
            }

            if (fabs(msg.endpoint.position.x - start_pose(0)) < min_val && fabs(msg.endpoint.position.y - start_pose(1))
                < min_val && fabs(msg.endpoint.position.z - start_pose(2)) < min_val && fabs(
                    msg.endpoint.orientation.x - start_pose(3)) < min_val && fabs(
                    msg.endpoint.orientation.y - start_pose(4)) < min_val && fabs(
                    msg.endpoint.orientation.z - start_pose(5)) < min_val && fabs(
                    msg.endpoint.orientation.w - start_pose(6)) < min_val)
            {
                error_message = "The starting point and the end point of the arc are equal！";
                return false;
            }
        }
        else
        {
            if (fabs(msg.axis.x) < min_val && fabs(msg.axis.y) < min_val && fabs(msg.axis.z) < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }

            if (msg.rotate_angle < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }
        }

        return true;
    }

    void PoseBasedReferenceManager::startServiceExecution(
        ServiceExecutionState& state,
        const vector_t& start_pose,
        const robot_control_msgs::msg::CircleMessage& msg,
        const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(state.mutex);

        // 初始化圆弧曲线
        if (!initCircleCurve(start_pose,
                             std::make_shared<robot_control_msgs::msg::CircleMessage>(msg),
                             state.curve))
        {
            RCLCPP_ERROR(logger_, "Initialization failure of the circular arc curve of the %s arm", arm_name.c_str());
            return;
        }

        // 获取总时间
        state.total_duration = state.curve->getTotalTime();

        // 预计算轨迹点
        constexpr double kSampleInterval = 0.04; // 25Hz
        const size_t kNumSamples = std::max(
            static_cast<size_t>(std::ceil(state.total_duration / kSampleInterval)) + 1,
            static_cast<size_t>(2)
        );

        // 清空之前的轨迹
        state.time_trajectory.clear();
        state.state_trajectory.clear();
        state.time_trajectory.reserve(kNumSamples);
        state.state_trajectory.reserve(kNumSamples);

        const double dt = state.total_duration / static_cast<double>(kNumSamples - 1);

        // 预计算所有轨迹点
        planning::TrajectPoint point;
        //保存计算的点
        // std::ofstream file("/home/lina/lina/data/cal_movec_data.csv");
        // file << "t" << "," << "x" << "," << "y" << "," << "z" << ","
        //     << "qx" << "," << "qy" << "," << "qz" << "," << "qw" << std::endl;
        for (size_t i = 0; i < kNumSamples; ++i)
        {
            const double t = static_cast<double>(i) * dt;
            point = state.curve->calculatePointAtTInRealTime(t);

            vector_t state_vec = vector_t::Zero(7);
            state_vec(0) = point.cart_pos(0);
            state_vec(1) = point.cart_pos(1);
            state_vec(2) = point.cart_pos(2);
            state_vec(3) = point.quaternion_point.q.x;
            state_vec(4) = point.quaternion_point.q.y;
            state_vec(5) = point.quaternion_point.q.z;
            state_vec(6) = point.quaternion_point.q.w;
            // file << std::fixed << std::setprecision(12) << t << "," << state_vec(0) << "," << state_vec(1) << "," <<
            //     state_vec(2) << "," << state_vec(3) << "," << state_vec(4) << "," << state_vec(5) << "," << state_vec(6)
            //     << std::endl;
            state.time_trajectory.push_back(t);
            state.state_trajectory.push_back(state_vec);
        }
        // file.close();
        // std::cout << "数据已保存" << std::endl;

        // 设置执行状态
        state.start_state = start_pose;
        state.start_time = std::chrono::steady_clock::now();
        state.current_point_index = 0;
        // state.is_executing = true;
    }

    void PoseBasedReferenceManager::sendPlannedTrajectoryToOCS2(const scalar_array_t& time_traj,
                                                                const vector_array_t& pose_trajectory,
                                                                const std::string& arm_name)
    {
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;
        size_t time_size = time_traj.size();
        size_t traj_size = pose_trajectory.size();
        if (traj_size != time_size)
        {
            RCLCPP_WARN(logger_, "The number of input times matches the number of trajectories");
            return;
        }
        // 起始时间
        const double t0 = current_observation_.time;
        scalar_array_t time_trajectory;
        time_trajectory.reserve(time_size);
        vector_array_t state_trajectory;
        state_trajectory.reserve(time_size);
        if (dual_arm_mode_)
        {
            if (arm_name == "left")
            {
                for (size_t i = 0; i < time_size; i++)
                {
                    vector_t current_state =
                        assembleDualArmReferenceState(pose_trajectory[i], previous_right_target_state);
                    time_trajectory.push_back(t0 + time_traj[i]);
                    state_trajectory.push_back(current_state);
                }
            }
            else if (arm_name == "right")
            {
                for (size_t i = 0; i < time_size; i++)
                {
                    vector_t current_state =
                        assembleDualArmReferenceState(previous_left_target_state, pose_trajectory[i]);
                    time_trajectory.push_back(t0 + time_traj[i]);
                    state_trajectory.push_back(current_state);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < time_size; i++)
            {
                vector_t current_state = vector_t::Zero(7);
                current_state = pose_trajectory[i]; // 保持右臂不变
                time_trajectory.push_back(t0 + time_traj[i]);
                state_trajectory.push_back(current_state);
            }
        }

        vector_array_t input_trajectory(
            time_size,
            vector_t::Zero(target_context_.input_dim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));

        if (arm_name == "left")
        {
            left_target_state_ = pose_trajectory.back();
            // 发布当前目标（只发布左臂）
            publishCurrentTargets("left");
        }
        else if (arm_name == "right")
        {
            right_target_state_ = pose_trajectory.back();
            // 发布当前目标（只发布左臂）
            publishCurrentTargets("right");
        }
    }

} // namespace ocs2::mpc_core
