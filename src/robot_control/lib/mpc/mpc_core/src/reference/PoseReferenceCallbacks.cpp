//
// PoseBasedReferenceManager: pose / twist / relative / stamped-pose topic callbacks and absolute targets
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

    void PoseBasedReferenceManager::logTargetJump(bool is_left, const vector_t& goal7) const
    {
        constexpr double kJumpLogDistance = 0.1;  // m
        const vector_t& tracked = is_left ? left_target_state_ : right_target_state_;
        if (!clock_ || tracked.size() < 3 || goal7.size() < 3)
        {
            return;
        }
        const double jump = (goal7.head<3>() - tracked.head<3>()).norm();
        if (jump <= kJumpLogDistance)
        {
            return;
        }
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "[CTL-014] %s target jumped %.3f m in one message: tracked (%.3f, %.3f, %.3f) -> new (%.3f, %.3f, %.3f)",
            is_left ? "left" : "right", jump, tracked(0), tracked(1), tracked(2), goal7(0), goal7(1), goal7(2));
    }

    void PoseBasedReferenceManager::leftPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        logTargetJump(true, target_state);
        applyLeftAbsoluteTarget(target_state, false);
    }

    void PoseBasedReferenceManager::rightPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        logTargetJump(false, target_state);
        applyRightAbsoluteTarget(target_state, false);
    }

    void PoseBasedReferenceManager::bodyPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        applyBodyAbsoluteTarget(target_state, false);
    }

    void PoseBasedReferenceManager::headPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        vector_t target_state(7);
        target_state << msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
        applyHeadAbsoluteTarget(target_state, false);
    }

    void PoseBasedReferenceManager::setArmExternalCommandPredicate(
        std::function<bool(bool is_left)> predicate)
    {
        arm_external_command_predicate_ = std::move(predicate);
    }

    bool PoseBasedReferenceManager::armExternalCommandAllowed(bool is_left) const
    {
        if (!arm_external_command_predicate_)
        {
            return true;
        }
        return arm_external_command_predicate_(is_left);
    }

    void PoseBasedReferenceManager::warnBlockedArmExternalCommand(bool is_left) const
    {
        if (!clock_)
        {
            return;
        }
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "%s EE command ignored while the arm is disabled or enable has not released",
            is_left ? "left" : "right");
    }

    void PoseBasedReferenceManager::applyLeftAbsoluteTarget(const vector_t& goal7, bool interpolate)
    {
        if (!armExternalCommandAllowed(true))
        {
            warnBlockedArmExternalCommand(true);
            return;
        }
        markLeftCouplingCommand();

        if (!interpolate)
        {
            // Continuous / marker Pose preempts this arm's moveL only.
            // If the opposite arm still has an active buffer, rebuild so its trajectory is not collapsed.
            resetArmReferenceBuffer(left_arm_reference_buffer_, goal7, current_observation_.time);
            left_target_state_ = goal7;
            (void)syncWholeBodyCoupledOppositeArmIfNeeded(true);
            const double t = current_observation_.time;
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
            publishCurrentTargets();
            return;
        }

        updateParam();

        const double start_time = current_observation_.time;
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;

        setArmReferenceBufferFromWaypoints(
            left_arm_reference_buffer_, previous_left_target_state, {goal7},
            start_time, moveL_duration_);
        left_target_state_ = goal7;

        const bool coupled = syncWholeBodyCoupledOppositeArmIfNeeded(true);
        if (coupled && dual_arm_mode_)
        {
            const vector_t coupled_right_target = right_target_state_;
            setArmReferenceBufferFromWaypoints(
                right_arm_reference_buffer_, previous_right_target_state, {coupled_right_target},
                start_time, moveL_duration_);
        }

        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, moveL_duration_);
        publishCurrentTargets(coupled ? "" : "left");
    }

    void PoseBasedReferenceManager::applyRightAbsoluteTarget(const vector_t& goal7, bool interpolate)
    {
        if (!armExternalCommandAllowed(false))
        {
            warnBlockedArmExternalCommand(false);
            return;
        }
        if (isWholeBodyBimanualCoupled() && isCoupledLeftLeaderFresh())
        {
            // Left still owns coupling (XR both-hands heartbeat, or recent left teleop).
            // Do not inverse-sync or push a second trajectory from right_target.
            return;
        }

        if (!interpolate)
        {
            resetArmReferenceBuffer(right_arm_reference_buffer_, goal7, current_observation_.time);
            right_target_state_ = goal7;
            (void)syncWholeBodyCoupledOppositeArmIfNeeded(false);
            const double t = current_observation_.time;
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
            publishCurrentTargets();
            return;
        }

        updateParam();

        const double start_time = current_observation_.time;
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;

        setArmReferenceBufferFromWaypoints(
            right_arm_reference_buffer_, previous_right_target_state, {goal7},
            start_time, moveL_duration_);
        right_target_state_ = goal7;

        const bool coupled = syncWholeBodyCoupledOppositeArmIfNeeded(false);
        if (coupled && dual_arm_mode_)
        {
            const vector_t coupled_left_target = left_target_state_;
            setArmReferenceBufferFromWaypoints(
                left_arm_reference_buffer_, previous_left_target_state, {coupled_left_target},
                start_time, moveL_duration_);
        }

        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, moveL_duration_);
        publishCurrentTargets(coupled ? "" : "right");
    }

    void PoseBasedReferenceManager::applyBodyAbsoluteTarget(const vector_t& goal7, bool interpolate)
    {
        if (goal7.size() < 7)
        {
            return;
        }

        if (!interpolate)
        {
            // Immediate body update: preempt body moveL only; preserve arm buffers.
            body_pose_7_xyzw_ = goal7.head<7>();
            const double t = current_observation_.time;
            resetArmReferenceBuffer(body_reference_buffer_, body_pose_7_xyzw_, t);
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
            publishCurrentTargets("body");
            return;
        }

        updateParam();
        const vector_t previous_body_target_state = body_pose_7_xyzw_;
        body_pose_7_xyzw_ = goal7.head<7>();
        updateBodyTrajectory(previous_body_target_state);
        publishCurrentTargets("body");
    }

    void PoseBasedReferenceManager::applyHeadAbsoluteTarget(const vector_t& goal7, bool interpolate)
    {
        if (goal7.size() < 7 || !target_context_.head_target_enabled)
        {
            return;
        }
        if (!interpolate)
        {
            head_pose_7_xyzw_ = goal7.head<7>();
            const double t = current_observation_.time;
            resetArmReferenceBuffer(head_reference_buffer_, head_pose_7_xyzw_, t);
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
            publishCurrentTargets("head");
            return;
        }

        updateParam();
        const vector_t previous = head_pose_7_xyzw_;
        head_pose_7_xyzw_ = goal7.head<7>();
        updateHeadTrajectory(previous);
        publishCurrentTargets("head");
    }

    vector_t PoseBasedReferenceManager::composeTwistDelta(
        const vector_t& base7, const geometry_msgs::msg::Twist& delta)
    {
        return integrateTwistVelocity(base7, delta, 1.0);
    }

    vector_t PoseBasedReferenceManager::integrateTwistVelocity(
        const vector_t& base7, const geometry_msgs::msg::Twist& velocity, double dt)
    {
        vector_t out = base7;
        out(0) += velocity.linear.x * dt;
        out(1) += velocity.linear.y * dt;
        out(2) += velocity.linear.z * dt;

        const double d_roll = velocity.angular.x * dt;
        const double d_pitch = velocity.angular.y * dt;
        const double d_yaw = velocity.angular.z * dt;
        if (std::abs(d_roll) > 1e-12 || std::abs(d_pitch) > 1e-12 || std::abs(d_yaw) > 1e-12)
        {
            Eigen::Quaterniond q(
                base7(6), base7(3), base7(4), base7(5));
            if (q.norm() < 1e-9)
            {
                q = Eigen::Quaterniond::Identity();
            }
            else
            {
                q.normalize();
            }

            const Eigen::AngleAxisd roll_angle(d_roll, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd pitch_angle(d_pitch, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd yaw_angle(d_yaw, Eigen::Vector3d::UnitZ());
            const Eigen::Quaterniond dq = yaw_angle * pitch_angle * roll_angle;
            q = (dq * q).normalized();
            out(3) = q.x();
            out(4) = q.y();
            out(5) = q.z();
            out(6) = q.w();
        }
        return out;
    }

    void PoseBasedReferenceManager::clearTwistLatchIfTimedOut()
    {
        const auto now = std::chrono::steady_clock::now();
        if (left_twist_active_)
        {
            const double age = std::chrono::duration<double>(now - left_twist_stamp_).count();
            if (age > kTwistTimeoutSec_)
            {
                left_twist_active_ = false;
                left_latched_twist_ = geometry_msgs::msg::Twist{};
            }
        }
        if (right_twist_active_)
        {
            const double age = std::chrono::duration<double>(now - right_twist_stamp_).count();
            if (age > kTwistTimeoutSec_)
            {
                right_twist_active_ = false;
                right_latched_twist_ = geometry_msgs::msg::Twist{};
            }
        }
    }

    void PoseBasedReferenceManager::integrateLatchedTwists(double dt)
    {
        const double t = current_observation_.time;
        bool updated = false;
        if (left_twist_active_ && !armExternalCommandAllowed(true))
        {
            left_twist_active_ = false;
            left_latched_twist_ = geometry_msgs::msg::Twist{};
        }
        else if (left_twist_active_)
        {
            // Preempt this arm's moveL; opposite arm preserved via rebuild below.
            markLeftCouplingCommand();
            resetArmReferenceBuffer(left_arm_reference_buffer_, left_target_state_, t);
            left_target_state_ = integrateTwistVelocity(left_target_state_, left_latched_twist_, dt);
            (void)syncWholeBodyCoupledOppositeArmIfNeeded(true);
            updated = true;
        }
        if (dual_arm_mode_ && right_twist_active_ && !armExternalCommandAllowed(false))
        {
            right_twist_active_ = false;
            right_latched_twist_ = geometry_msgs::msg::Twist{};
        }
        const bool allow_right_twist =
            dual_arm_mode_ && right_twist_active_ &&
            !(isWholeBodyBimanualCoupled() && isCoupledLeftLeaderFresh());
        if (allow_right_twist)
        {
            resetArmReferenceBuffer(right_arm_reference_buffer_, right_target_state_, t);
            right_target_state_ = integrateTwistVelocity(right_target_state_, right_latched_twist_, dt);
            (void)syncWholeBodyCoupledOppositeArmIfNeeded(false);
            updated = true;
        }
        if (updated)
        {
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
            publishCurrentTargets();
        }
    }

    void PoseBasedReferenceManager::leftTwistCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        if (!armExternalCommandAllowed(true))
        {
            left_twist_active_ = false;
            left_latched_twist_ = geometry_msgs::msg::Twist{};
            warnBlockedArmExternalCommand(true);
            return;
        }
        left_latched_twist_ = *msg;
        left_twist_stamp_ = std::chrono::steady_clock::now();
        const bool is_zero =
            std::abs(msg->linear.x) < 1e-9 && std::abs(msg->linear.y) < 1e-9 &&
            std::abs(msg->linear.z) < 1e-9 && std::abs(msg->angular.x) < 1e-9 &&
            std::abs(msg->angular.y) < 1e-9 && std::abs(msg->angular.z) < 1e-9;
        left_twist_active_ = !is_zero;
        if (left_twist_active_)
        {
            markLeftCouplingCommand();
        }
    }

    void PoseBasedReferenceManager::rightTwistCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        if (!armExternalCommandAllowed(false))
        {
            right_twist_active_ = false;
            right_latched_twist_ = geometry_msgs::msg::Twist{};
            warnBlockedArmExternalCommand(false);
            return;
        }
        right_latched_twist_ = *msg;
        right_twist_stamp_ = std::chrono::steady_clock::now();
        const bool is_zero =
            std::abs(msg->linear.x) < 1e-9 && std::abs(msg->linear.y) < 1e-9 &&
            std::abs(msg->linear.z) < 1e-9 && std::abs(msg->angular.x) < 1e-9 &&
            std::abs(msg->angular.y) < 1e-9 && std::abs(msg->angular.z) < 1e-9;
        right_twist_active_ = !is_zero;
    }

    bool PoseBasedReferenceManager::twistStampedToBaseTwist(
        const geometry_msgs::msg::TwistStamped& msg,
        geometry_msgs::msg::Twist& out_twist_base) const
    {
        out_twist_base = msg.twist;
        if (msg.header.frame_id.empty() || msg.header.frame_id == base_frame_)
        {
            return true;
        }
        if (!tf_buffer_)
        {
            RCLCPP_WARN(logger_, "Cannot transform relative twist from %s: TF buffer unavailable",
                        msg.header.frame_id.c_str());
            return false;
        }
        try
        {
            const geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, msg.header.frame_id, tf2::TimePointZero);

            geometry_msgs::msg::Vector3Stamped lin_in, lin_out, ang_in, ang_out;
            lin_in.header = msg.header;
            lin_in.vector = msg.twist.linear;
            ang_in.header = msg.header;
            ang_in.vector = msg.twist.angular;
            tf2::doTransform(lin_in, lin_out, transform);
            tf2::doTransform(ang_in, ang_out, transform);
            out_twist_base.linear = lin_out.vector;
            out_twist_base.angular = ang_out.vector;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_, "Cannot transform relative twist from %s to %s: %s",
                        msg.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    void PoseBasedReferenceManager::leftRelativeCallback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        geometry_msgs::msg::Twist twist_base;
        if (!twistStampedToBaseTwist(*msg, twist_base))
        {
            return;
        }
        const vector_t goal = composeTwistDelta(left_target_state_, twist_base);
        applyLeftAbsoluteTarget(goal, true);
    }

    void PoseBasedReferenceManager::rightRelativeCallback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        geometry_msgs::msg::Twist twist_base;
        if (!twistStampedToBaseTwist(*msg, twist_base))
        {
            return;
        }
        const vector_t goal = composeTwistDelta(right_target_state_, twist_base);
        applyRightAbsoluteTarget(goal, true);
    }

    void PoseBasedReferenceManager::bodyRelativeCallback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        geometry_msgs::msg::Twist twist_base;
        if (!twistStampedToBaseTwist(*msg, twist_base))
        {
            return;
        }
        const vector_t goal = composeTwistDelta(body_pose_7_xyzw_, twist_base);
        applyBodyAbsoluteTarget(goal, true);
    }

    void PoseBasedReferenceManager::leftPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        applyLeftAbsoluteTarget(target_state, true);
    }

    void PoseBasedReferenceManager::rightPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        applyRightAbsoluteTarget(target_state, true);
    }

    void PoseBasedReferenceManager::bodyPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        applyBodyAbsoluteTarget(target_state, true);
    }

    void PoseBasedReferenceManager::headPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        vector_t target_state(7);
        target_state << msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
        applyHeadAbsoluteTarget(target_state, true);
    }

    void PoseBasedReferenceManager::processPoseStamped(
        const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
        std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback)
    {
        // 如果源frame和目标frame相同，直接使用
        if (msg->header.frame_id == base_frame_)
        {
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = msg->pose.position;
            pose_msg->orientation = msg->pose.orientation;
            callback(pose_msg);
            return;
        }

        // 使用TF转换到base frame（使用最新变换）
        try
        {
            geometry_msgs::msg::PoseStamped transformed_pose;

            // 使用最新可用变换
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
                                            tf2::TimePointZero);

            // 使用doTransform进行转换
            tf2::doTransform(*msg, transformed_pose, transform);

            // 调用指定的回调方法
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = transformed_pose.pose.position;
            pose_msg->orientation = transformed_pose.pose.orientation;
            callback(pose_msg);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_, "无法将pose从 %s 转换到 %s: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    bool PoseBasedReferenceManager::parsePoseStampedToState(
        const geometry_msgs::msg::PoseStamped& pose_stamped,
        const char* tag,
        vector_t& out_state) const
    {
        auto poseToState = [](const geometry_msgs::msg::Pose& pose) -> vector_t
        {
            vector_t state = vector_t::Zero(7);
            state(0) = pose.position.x;
            state(1) = pose.position.y;
            state(2) = pose.position.z;
            state(3) = pose.orientation.x;
            state(4) = pose.orientation.y;
            state(5) = pose.orientation.z;
            state(6) = pose.orientation.w;
            return state;
        };

        if (pose_stamped.header.frame_id == base_frame_)
        {
            out_state = poseToState(pose_stamped.pose);
            return true;
        }

        try
        {
            geometry_msgs::msg::PoseStamped transformed_pose;
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, pose_stamped.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(pose_stamped, transformed_pose, transform);
            out_state = poseToState(transformed_pose.pose);
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_, "无法将%s pose从 %s 转换到 %s: %s",
                        tag, pose_stamped.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    void PoseBasedReferenceManager::leftPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (shouldAcceptExternalTargets())
        {
            processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
            {
                leftPoseStampedPoseCallback(pose_msg);
            });
            return;
        }
        if (stamped_left_handler_)
        {
            stamped_left_handler_(*msg);
        }
    }

    void PoseBasedReferenceManager::rightPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (shouldAcceptExternalTargets())
        {
            processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
            {
                rightPoseStampedPoseCallback(pose_msg);
            });
            return;
        }
        if (stamped_right_handler_)
        {
            stamped_right_handler_(*msg);
        }
    }

    void PoseBasedReferenceManager::bodyPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            bodyPoseStampedPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::headPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!shouldAcceptExternalTargets())
        {
            return;
        }
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            headPoseStampedPoseCallback(pose_msg);
        });
    }

} // namespace ocs2::mpc_core
