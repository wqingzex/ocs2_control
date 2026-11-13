//
// PoseBasedReferenceManager: dual-arm stamped target, nav_msgs/Path and execute-path service
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

    void PoseBasedReferenceManager::dualTargetStampedCallback(
        const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            if (stamped_dual_handler_)
            {
                stamped_dual_handler_(*msg);
            }
            return;
        }
        updateParam();

        if (!dual_arm_mode_)
        {
            RCLCPP_WARN(
                logger_,
                "Dual target stamped callback called but dual arm mode is not enabled");
            return;
        }
        if (!armExternalCommandAllowed(true) || !armExternalCommandAllowed(false))
        {
            if (!armExternalCommandAllowed(true))
            {
                warnBlockedArmExternalCommand(true);
            }
            if (!armExternalCommandAllowed(false))
            {
                warnBlockedArmExternalCommand(false);
            }
            return;
        }

        // 支持两种格式：
        //   2 poses: [left, right]
        //   3 poses: [left, right, body]  -> 三条轨迹统一插值规划
        if (msg->poses.size() != 2 &&
            !(target_context_.body_target_enabled && msg->poses.size() == 3))
        {
            RCLCPP_WARN(logger_, "Dual target path must contain 2 or 3 poses ([left,right] or [left,right,body]), got %zu",
                        msg->poses.size());
            return;
        }

        // 保存上一帧缓存（用于插值起点）
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;
        const vector_t previous_body_target_state = body_pose_7_xyzw_;

        // 处理左臂（第一个pose）
        vector_t left_target_state;
        if (!parsePoseStampedToState(msg->poses[0], "左臂", left_target_state))
        {
            return;
        }
        markLeftCouplingCommand();

        // 处理右臂（第二个pose）
        vector_t right_target_state;
        if (!parsePoseStampedToState(msg->poses[1], "右臂", right_target_state))
        {
            return;
        }

        // 在双臂耦合下，单次发布若左右臂目标与已捕获耦合关系冲突，
        // 将右臂目标匹配（投影）到由左臂目标定义的耦合流形上。
        if (isWholeBodyTargetLayout())
        {
            auto* sw = dynamic_cast<const BimanualCouplingSource*>(referenceManagerPtr_.get());
            if (sw != nullptr)
            {
                const scalar_t t = current_observation_.time;
                if (sw->isBimanualCoupled(t) && sw->hasCapturedCoupling())
                {
                    const auto [relPos, relQuatRaw] = sw->getCouplingParameters(t);
                    Eigen::Quaternion<scalar_t> relQuat = relQuatRaw;
                    if (relQuat.norm() > static_cast<scalar_t>(1e-9))
                    {
                        relQuat.normalize();

                        const Eigen::Matrix<scalar_t, 3, 1> leftPos = left_target_state.head<3>();
                        Eigen::Quaternion<scalar_t> leftQuat(
                            left_target_state(6), left_target_state(3), left_target_state(4), left_target_state(5));
                        if (leftQuat.norm() < static_cast<scalar_t>(1e-9))
                        {
                            leftQuat = Eigen::Quaternion<scalar_t>::Identity();
                        }
                        else
                        {
                            leftQuat.normalize();
                        }

                        vector_t matched_right = right_target_state;
                        const Eigen::Matrix<scalar_t, 3, 1> rightPosMatched = leftPos + leftQuat * relPos;
                        const Eigen::Quaternion<scalar_t> rightQuatMatched = (leftQuat * relQuat).normalized();
                        matched_right.head<3>() = rightPosMatched;
                        matched_right(3) = rightQuatMatched.x();
                        matched_right(4) = rightQuatMatched.y();
                        matched_right(5) = rightQuatMatched.z();
                        matched_right(6) = rightQuatMatched.w();

                        const scalar_t posErr = (right_target_state.head<3>() - rightPosMatched).norm();
                        Eigen::Quaternion<scalar_t> rightQuatGiven(
                            right_target_state(6), right_target_state(3), right_target_state(4), right_target_state(5));
                        if (rightQuatGiven.norm() < static_cast<scalar_t>(1e-9))
                        {
                            rightQuatGiven = Eigen::Quaternion<scalar_t>::Identity();
                        }
                        else
                        {
                            rightQuatGiven.normalize();
                        }
                        const scalar_t quatDot = std::abs(rightQuatGiven.dot(rightQuatMatched));
                        const scalar_t oriErr = static_cast<scalar_t>(
                            2.0 * std::acos(std::clamp(static_cast<double>(quatDot), 0.0, 1.0)));

                        constexpr scalar_t kPosConflictEps = static_cast<scalar_t>(1e-6);
                        constexpr scalar_t kOriConflictEps = static_cast<scalar_t>(1e-6);
                        if (posErr > kPosConflictEps || oriErr > kOriConflictEps)
                        {
                            right_target_state = matched_right;
                            RCLCPP_WARN_THROTTLE(
                                logger_, *clock_, 2000,
                                "dual_target/stamped conflicts with coupled constraint; right target is matched to coupling manifold");
                        }
                    }
                }
            }
        }

        // 更新缓存到新目标
        left_target_state_ = left_target_state;
        right_target_state_ = right_target_state;

        if (msg->poses.size() == 3)
        {
            vector_t body_target_state;
            if (!parsePoseStampedToState(msg->poses[2], "腰部", body_target_state))
            {
                return;
            }
            body_pose_7_xyzw_ = body_target_state;

            // 三条轨迹统一插值规划（左/右/腰）
            updateTrajectoryWithBody(previous_left_target_state, previous_right_target_state, previous_body_target_state);
        }
        else
        {
            // 兼容旧格式：仅双臂
            updateTrajectory(previous_left_target_state, previous_right_target_state);
        }

        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::pathCallback(
        const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        updateParam();

        if (msg->poses.empty())
        {
            RCLCPP_WARN(logger_, "Received empty path");
            return;
        }

        const size_t total_points = msg->poses.size();

        // 转换路径点到状态向量
        auto poseToState =
            [](const geometry_msgs::msg::PoseStamped& pose_stamped) -> vector_t
        {
            vector_t state = vector_t::Zero(7);
            state(0) = pose_stamped.pose.position.x;
            state(1) = pose_stamped.pose.position.y;
            state(2) = pose_stamped.pose.position.z;
            state(3) = pose_stamped.pose.orientation.x;
            state(4) = pose_stamped.pose.orientation.y;
            state(5) = pose_stamped.pose.orientation.z;
            state(6) = pose_stamped.pose.orientation.w;
            return state;
        };

        // 处理路径点：默认所有点都在base frame下，不做TF转换
        std::vector<vector_t> left_arm_waypoints;
        std::vector<vector_t> right_arm_waypoints;

        for (size_t i = 0; i < total_points; ++i)
        {
            const auto& pose_stamped = msg->poses[i];
            // 直接转换，假设所有点都在base frame下
            vector_t state = poseToState(pose_stamped);

            // 根据模式分配路径点
            if (dual_arm_mode_)
            {
                // 双臂模式：将路径点分成两半，前半段给左臂，后半段给右臂
                const size_t left_points = (total_points + 1) / 2; // 前半段（向上取整）
                if (i < left_points)
                {
                    left_arm_waypoints.push_back(state);
                }
                else
                {
                    right_arm_waypoints.push_back(state);
                }
            }
            else
            {
                // 单臂模式：所有路径点都用于左臂（单臂机器人使用左臂主题）
                left_arm_waypoints.push_back(state);
            }
        }

        // 检查是否有有效的路径点
        if (left_arm_waypoints.empty() && right_arm_waypoints.empty())
        {
            RCLCPP_WARN(logger_, "没有有效的路径点可以处理");
            return;
        }
        if ((!left_arm_waypoints.empty() && !armExternalCommandAllowed(true)) ||
            (!right_arm_waypoints.empty() && !armExternalCommandAllowed(false)))
        {
            if (!left_arm_waypoints.empty() && !armExternalCommandAllowed(true))
            {
                warnBlockedArmExternalCommand(true);
            }
            if (!right_arm_waypoints.empty() && !armExternalCommandAllowed(false))
            {
                warnBlockedArmExternalCommand(false);
            }
            return;
        }

        // 如果某个臂没有路径点，使用当前缓存的状态（仅双臂模式）
        if (dual_arm_mode_)
        {
            if (left_arm_waypoints.empty())
            {
                left_arm_waypoints.push_back(left_target_state_);
            }
            if (right_arm_waypoints.empty())
            {
                right_arm_waypoints.push_back(right_target_state_);
            }
        }
        else
        {
            // 单臂模式：如果左臂没有路径点，使用当前缓存的状态
            if (left_arm_waypoints.empty())
            {
                left_arm_waypoints.push_back(left_target_state_);
            }
        }

        runInterpolatedPathTrajectory(left_arm_waypoints, right_arm_waypoints, trajectory_duration_);
    }

    void PoseBasedReferenceManager::runInterpolatedPathTrajectory(
        const std::vector<vector_t>& left_arm_waypoints,
        const std::vector<vector_t>& right_arm_waypoints,
        double trajectory_duration_sec)
    {
        const double start_time = current_observation_.time;
        if (!left_arm_waypoints.empty())
        {
            setArmReferenceBufferFromWaypoints(left_arm_reference_buffer_, left_target_state_,
                                               left_arm_waypoints, start_time, trajectory_duration_sec);
            left_target_state_ = left_arm_waypoints.back();
        }
        if (dual_arm_mode_ && !right_arm_waypoints.empty())
        {
            setArmReferenceBufferFromWaypoints(right_arm_reference_buffer_, right_target_state_,
                                               right_arm_waypoints, start_time, trajectory_duration_sec);
            right_target_state_ = right_arm_waypoints.back();
        }
        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, trajectory_duration_sec);

        RCLCPP_INFO(logger_, "MoveL S-curve path: left=%zu, right=%zu, duration=%.3fs",
                    left_arm_waypoints.size(), right_arm_waypoints.size(),
                    std::max(left_arm_reference_buffer_.duration, right_arm_reference_buffer_.duration));
    }

    void PoseBasedReferenceManager::handleExecutePathService(
        const std::shared_ptr<robot_control_msgs::srv::ExecutePath::Request> request,
        std::shared_ptr<robot_control_msgs::srv::ExecutePath::Response> response)
    {
        updateParam();

        double duration = request->trajectory_duration;
        if (duration <= 0.0) duration = trajectory_duration_;

        std::vector<vector_t> left_arm_waypoints;
        std::vector<vector_t> right_arm_waypoints;

        for (const auto& ps : request->left_arm_path.poses)
        {
            vector_t state;
            if (!parsePoseStampedToState(ps, "左臂路径点", state))
            {
                response->success = false;
                response->message = "failed to transform left_arm_path waypoint to base frame";
                response->estimated_duration = 0.0;
                return;
            }
            left_arm_waypoints.push_back(state);
        }

        if (dual_arm_mode_)
        {
            for (const auto& ps : request->right_arm_path.poses)
            {
                vector_t state;
                if (!parsePoseStampedToState(ps, "右臂路径点", state))
                {
                    response->success = false;
                    response->message = "failed to transform right_arm_path waypoint to base frame";
                    response->estimated_duration = 0.0;
                    return;
                }
                right_arm_waypoints.push_back(state);
            }
        }

        if (dual_arm_mode_ && request->left_arm_path.poses.empty() && request->right_arm_path.poses.empty())
        {
            response->success = false;
            response->message = "both left_arm_path and right_arm_path are empty";
            response->estimated_duration = 0.0;
            return;
        }
        if ((!left_arm_waypoints.empty() && !armExternalCommandAllowed(true)) ||
            (!right_arm_waypoints.empty() && !armExternalCommandAllowed(false)))
        {
            response->success = false;
            response->message = "EE path ignored while the arm is disabled or enable has not released";
            response->estimated_duration = 0.0;
            if (!left_arm_waypoints.empty() && !armExternalCommandAllowed(true))
            {
                warnBlockedArmExternalCommand(true);
            }
            if (!right_arm_waypoints.empty() && !armExternalCommandAllowed(false))
            {
                warnBlockedArmExternalCommand(false);
            }
            return;
        }

        const double start_time = current_observation_.time;
        double end_time = start_time + duration;

        if (!left_arm_waypoints.empty())
        {
            setArmReferenceBufferFromWaypoints(
                left_arm_reference_buffer_, left_target_state_, left_arm_waypoints, start_time, duration);
            left_target_state_ = left_arm_waypoints.back();
            end_time = std::max(end_time, left_arm_reference_buffer_.startTime + left_arm_reference_buffer_.duration);
        }

        if (dual_arm_mode_ && !right_arm_waypoints.empty())
        {
            setArmReferenceBufferFromWaypoints(
                right_arm_reference_buffer_, right_target_state_, right_arm_waypoints, start_time, duration);
            right_target_state_ = right_arm_waypoints.back();
            end_time = std::max(end_time, right_arm_reference_buffer_.startTime + right_arm_reference_buffer_.duration);
        }

        const auto extendEndTimeFromActiveBuffer = [&end_time, start_time](const ArmReferenceBuffer& buffer)
        {
            if (!buffer.path.empty() && buffer.duration > 0.0)
            {
                const double buffer_end_time = buffer.startTime + buffer.duration;
                if (buffer_end_time > start_time)
                {
                    end_time = std::max(end_time, buffer_end_time);
                }
            }
        };
        extendEndTimeFromActiveBuffer(left_arm_reference_buffer_);
        if (dual_arm_mode_)
        {
            extendEndTimeFromActiveBuffer(right_arm_reference_buffer_);
        }
        if (target_context_.body_target_enabled)
        {
            extendEndTimeFromActiveBuffer(body_reference_buffer_);
        }
        if (target_context_.head_target_enabled)
        {
            extendEndTimeFromActiveBuffer(head_reference_buffer_);
        }

        if (!dual_arm_mode_ && left_arm_waypoints.empty())
        {
            response->success = false;
            response->message = "left_arm_path is empty";
            response->estimated_duration = 0.0;
            return;
        }

        rebuildTargetTrajectoriesFromArmReferenceBuffers(start_time, end_time);

        response->success = true;
        response->message = "trajectory stitched";
        response->estimated_duration = end_time - start_time;
    }

} // namespace ocs2::mpc_core
