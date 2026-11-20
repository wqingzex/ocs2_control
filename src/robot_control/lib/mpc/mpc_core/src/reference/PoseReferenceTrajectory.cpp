//
// PoseBasedReferenceManager: target-state assembly, pose interpolation, arm reference buffers, trajectory updates
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

    namespace
    {
        struct SCurveProfile
        {
            double jerk{1.0};
            double jerk_time{0.0};
            double constant_accel_time{0.0};
            double cruise_time{0.0};
            double minimum_duration{0.0};
        };

        SCurveProfile makeUnitSCurve(double max_velocity, double max_acceleration, double max_jerk)
        {
            constexpr double kEps = 1e-9;
            const double v = std::max(max_velocity, kEps);
            const double a = std::max(max_acceleration, kEps);
            const double j = std::max(max_jerk, kEps);

            SCurveProfile profile;
            profile.jerk = j;

            // Fastest acceleration ramp that respects both acceleration and velocity.
            if (v >= a * a / j)
            {
                profile.jerk_time = a / j;
                profile.constant_accel_time = v / a - profile.jerk_time;
            }
            else
            {
                profile.jerk_time = std::sqrt(v / j);
            }

            const auto accelDistance = [j](double tj, double ta)
            {
                return j * tj * tj * tj + 1.5 * j * tj * tj * ta +
                       0.5 * j * tj * ta * ta;
            };
            double accel_decel_distance =
                2.0 * accelDistance(profile.jerk_time, profile.constant_accel_time);

            if (accel_decel_distance < 1.0 - kEps)
            {
                const double peak_velocity =
                    j * profile.jerk_time * (profile.jerk_time + profile.constant_accel_time);
                profile.cruise_time = (1.0 - accel_decel_distance) / peak_velocity;
            }
            else
            {
                // The move is too short to reach the requested peak velocity.
                const double triangular_tj = std::cbrt(1.0 / (2.0 * j));
                if (triangular_tj <= a / j + kEps && j * triangular_tj * triangular_tj <= v + kEps)
                {
                    profile.jerk_time = triangular_tj;
                    profile.constant_accel_time = 0.0;
                }
                else
                {
                    profile.jerk_time = a / j;
                    // Solve 2*x_acc(tj, ta)=1 for the non-negative constant-acceleration time.
                    const double tj = profile.jerk_time;
                    const double qa = j * tj;
                    const double qb = 3.0 * j * tj * tj;
                    const double qc = 2.0 * j * tj * tj * tj - 1.0;
                    profile.constant_accel_time =
                        std::max(0.0, (-qb + std::sqrt(std::max(0.0, qb * qb - 4.0 * qa * qc))) /
                                          (2.0 * qa));
                }
                profile.cruise_time = 0.0;
            }

            profile.minimum_duration =
                4.0 * profile.jerk_time + 2.0 * profile.constant_accel_time + profile.cruise_time;
            return profile;
        }

        double sampleUnitSCurve(const SCurveProfile& profile, double elapsed, double actual_duration)
        {
            if (actual_duration <= 0.0 || profile.minimum_duration <= 0.0) return 1.0;
            if (elapsed <= 0.0) return 0.0;
            if (elapsed >= actual_duration) return 1.0;
            const double base_time = std::clamp(elapsed / actual_duration, 0.0, 1.0) * profile.minimum_duration;
            const double tj = profile.jerk_time;
            const double ta = profile.constant_accel_time;
            const double tv = profile.cruise_time;
            const double phase_durations[7] = {tj, ta, tj, tv, tj, ta, tj};
            const double phase_jerks[7] = {profile.jerk, 0.0, -profile.jerk, 0.0,
                                           -profile.jerk, 0.0, profile.jerk};

            double position = 0.0;
            double velocity = 0.0;
            double acceleration = 0.0;
            double remaining = base_time;
            for (size_t phase = 0; phase < 7 && remaining > 0.0; ++phase)
            {
                const double dt = std::min(remaining, phase_durations[phase]);
                const double jerk = phase_jerks[phase];
                position += velocity * dt + 0.5 * acceleration * dt * dt +
                            jerk * dt * dt * dt / 6.0;
                velocity += acceleration * dt + 0.5 * jerk * dt * dt;
                acceleration += jerk * dt;
                remaining -= dt;
            }
            return std::clamp(position, 0.0, 1.0);
        }

        double quaternionDistance(const vector_t& start, const vector_t& goal)
        {
            Eigen::Quaterniond q0(start(6), start(3), start(4), start(5));
            Eigen::Quaterniond q1(goal(6), goal(3), goal(4), goal(5));
            if (q0.norm() < 1e-9) q0 = Eigen::Quaterniond::Identity(); else q0.normalize();
            if (q1.norm() < 1e-9) q1 = q0; else q1.normalize();
            return 2.0 * std::acos(std::clamp(std::abs(q0.dot(q1)), 0.0, 1.0));
        }
    } // namespace

    int PoseBasedReferenceManager::effectiveTargetStateDim() const
    {
        if (isWholeBodyTargetLayout())
        {
            return target_context_.wholeBodyTargetStateDim();
        }
        return dual_arm_mode_ ? 14 : 7;
    }

    vector_t PoseBasedReferenceManager::identityBodyPose7() const
    {
        vector_t b = vector_t::Zero(7);
        b(6) = 1.0;
        return b;
    }

    vector_t PoseBasedReferenceManager::bodySegmentForAssembly() const
    {
        if (!target_context_.body_pose_from_current_state &&
            target_context_.body_target_enabled)
        {
            return identityBodyPose7();
        }
        return body_pose_7_xyzw_;
    }

    bool PoseBasedReferenceManager::isWholeBodyTargetLayout() const
    {
        return dual_arm_mode_ &&
            (target_context_.body_target_enabled || target_context_.head_target_enabled);
    }

    vector_t PoseBasedReferenceManager::assembleDualArmReferenceState(const vector_t& left7, const vector_t& right7) const
    {
        if (isWholeBodyTargetLayout())
        {
            return assembleWholeBodyTargetState(
                left7, right7, bodySegmentForAssembly(), head_pose_7_xyzw_,
                target_context_.body_target_enabled, target_context_.head_target_enabled);
        }
        vector_t s = vector_t::Zero(14);
        s.segment(0, 7) = left7;
        s.segment(7, 7) = right7;
        return s;
    }

    vector_t PoseBasedReferenceManager::assembleWholeBodyTargetState(const vector_t& left_pose7_xyzw,
                                                                         const vector_t& right_pose7_xyzw,
                                                                         const vector_t& body_pose7_xyzw,
                                                                         const vector_t& head_pose7_xyzw,
                                                                         bool body_target_enabled,
                                                                         bool head_target_enabled)
    {
        const int dim = OCS2ReferenceTargetContext::kDualArmTargetDim +
            (body_target_enabled ? OCS2ReferenceTargetContext::kPoseTargetDim : 0) +
            (head_target_enabled ? OCS2ReferenceTargetContext::kPoseTargetDim : 0);
        vector_t s = vector_t::Zero(dim);
        s.segment(0, 7) = left_pose7_xyzw.head<7>();
        s.segment(7, 7) = right_pose7_xyzw.head<7>();
        int offset = OCS2ReferenceTargetContext::kDualArmTargetDim;
        if (body_target_enabled) {
            s.segment(offset, 7) = body_pose7_xyzw.head<7>();
            offset += OCS2ReferenceTargetContext::kPoseTargetDim;
        }
        if (head_target_enabled) {
            s.segment(offset, 7) = head_pose7_xyzw.head<7>();
        }
        return s;
    }

    vector_t PoseBasedReferenceManager::interpolatePose7(const vector_t& start,
                                                       const vector_t& goal,
                                                       double alpha)
    {
        const double a = std::clamp(alpha, 0.0, 1.0);
        vector_t out = vector_t::Zero(7);
        out.segment<3>(0) = (1.0 - a) * start.segment<3>(0) + a * goal.segment<3>(0);

        Eigen::Quaterniond q0(start(6), start(3), start(4), start(5));
        Eigen::Quaterniond q1(goal(6), goal(3), goal(4), goal(5));
        if (q0.norm() < 1e-9)
        {
            q0 = Eigen::Quaterniond::Identity();
        }
        else
        {
            q0.normalize();
        }
        if (q1.norm() < 1e-9)
        {
            q1 = q0;
        }
        else
        {
            q1.normalize();
        }
        if (q0.dot(q1) < 0.0)
        {
            q1.coeffs() *= -1.0;
        }

        Eigen::Quaterniond q = q0.slerp(a, q1);
        q.normalize();
        out(3) = q.x();
        out(4) = q.y();
        out(5) = q.z();
        out(6) = q.w();
        return out;
    }

    // 按归一化进度 alpha∈[0,1] 在 7 维 pose 路径上均匀分段采样；段内用 interpolatePose7 做位置线性插值与姿态 slerp。
    vector_t PoseBasedReferenceManager::samplePose7Path(const std::vector<vector_t>& path,
                                                        double alpha)
    {
        if (path.empty())
        {
            vector_t identity = vector_t::Zero(7);
            identity(6) = 1.0;
            return identity;
        }
        if (path.size() == 1)
        {
            return path.front();
        }
        const double global_alpha = std::clamp(alpha, 0.0, 1.0);
        const size_t num_segments = path.size() - 1;
        const double segment_size = 1.0 / static_cast<double>(num_segments);
        size_t segment_idx = static_cast<size_t>(global_alpha / segment_size);
        segment_idx = std::min(segment_idx, num_segments - 1);
        const double segment_start = static_cast<double>(segment_idx) * segment_size;
        const double local_alpha = std::clamp((global_alpha - segment_start) / segment_size, 0.0, 1.0);
        return interpolatePose7(path[segment_idx], path[segment_idx + 1], local_alpha);
    }

    double PoseBasedReferenceManager::minimumPoseDuration(const vector_t& start, const vector_t& goal) const
    {
        constexpr double kEps = 1e-9;
        const double distance = (goal.head<3>() - start.head<3>()).norm();
        const double angle = quaternionDistance(start, goal);

        double path_velocity = std::numeric_limits<double>::infinity();
        double path_acceleration = std::numeric_limits<double>::infinity();
        double path_jerk = std::numeric_limits<double>::infinity();
        if (distance > kEps)
        {
            path_velocity = std::min(path_velocity, moveL_max_linear_velocity_ / distance);
            path_acceleration = std::min(path_acceleration, moveL_max_linear_acceleration_ / distance);
            path_jerk = std::min(path_jerk, moveL_max_linear_jerk_ / distance);
        }
        if (angle > kEps)
        {
            path_velocity = std::min(path_velocity, moveL_max_angular_velocity_ / angle);
            path_acceleration = std::min(path_acceleration, moveL_max_angular_acceleration_ / angle);
            path_jerk = std::min(path_jerk, moveL_max_angular_jerk_ / angle);
        }
        if (!std::isfinite(path_velocity)) return 0.0;
        return makeUnitSCurve(path_velocity, path_acceleration, path_jerk).minimum_duration;
    }

    double PoseBasedReferenceManager::samplePoseProgress(const vector_t& start, const vector_t& goal,
                                                         double elapsed, double duration) const
    {
        constexpr double kEps = 1e-9;
        const double distance = (goal.head<3>() - start.head<3>()).norm();
        const double angle = quaternionDistance(start, goal);
        if (distance <= kEps && angle <= kEps) return 1.0;

        double path_velocity = std::numeric_limits<double>::infinity();
        double path_acceleration = std::numeric_limits<double>::infinity();
        double path_jerk = std::numeric_limits<double>::infinity();
        if (distance > kEps)
        {
            path_velocity = std::min(path_velocity, moveL_max_linear_velocity_ / distance);
            path_acceleration = std::min(path_acceleration, moveL_max_linear_acceleration_ / distance);
            path_jerk = std::min(path_jerk, moveL_max_linear_jerk_ / distance);
        }
        if (angle > kEps)
        {
            path_velocity = std::min(path_velocity, moveL_max_angular_velocity_ / angle);
            path_acceleration = std::min(path_acceleration, moveL_max_angular_acceleration_ / angle);
            path_jerk = std::min(path_jerk, moveL_max_angular_jerk_ / angle);
        }
        return sampleUnitSCurve(makeUnitSCurve(path_velocity, path_acceleration, path_jerk), elapsed, duration);
    }

    void PoseBasedReferenceManager::resetArmReferenceBuffer(ArmReferenceBuffer& buffer,
                                                            const vector_t& pose,
                                                            double time)
    {
        buffer.startTime = time;
        buffer.duration = 0.0;
        buffer.path = {pose};
        buffer.segmentDurations.clear();
    }

    void PoseBasedReferenceManager::setArmReferenceBufferFromWaypoints(
        ArmReferenceBuffer& buffer,
        const vector_t& fallback_start_pose,
        const std::vector<vector_t>& waypoints,
        double start_time,
        double duration)
    {
        const bool has_active_old_path = isArmReferenceBufferActive(buffer, start_time);
        const vector_t start_pose =
            has_active_old_path ? sampleArmReferenceBuffer(buffer, fallback_start_pose, start_time) : fallback_start_pose;

        buffer.startTime = start_time;
        buffer.path.clear();
        buffer.path.reserve(waypoints.size() + 1);
        buffer.path.push_back(start_pose);
        buffer.path.insert(buffer.path.end(), waypoints.begin(), waypoints.end());

        buffer.segmentDurations.clear();
        buffer.segmentDurations.reserve(waypoints.size());
        double minimum_total = 0.0;
        for (size_t i = 0; i + 1 < buffer.path.size(); ++i)
        {
            const double segment_minimum = minimumPoseDuration(buffer.path[i], buffer.path[i + 1]);
            buffer.segmentDurations.push_back(segment_minimum);
            minimum_total += segment_minimum;
        }

        const double requested = std::max(duration, 0.0);
        const double actual_total = moveL_auto_extend_duration_ ? std::max(requested, minimum_total) : requested;
        if (minimum_total > 1e-9)
        {
            const double scale = actual_total / minimum_total;
            for (double& segment_duration : buffer.segmentDurations) segment_duration *= scale;
        }
        else if (!buffer.segmentDurations.empty())
        {
            const double equal_duration = actual_total / static_cast<double>(buffer.segmentDurations.size());
            std::fill(buffer.segmentDurations.begin(), buffer.segmentDurations.end(), equal_duration);
        }
        buffer.duration = actual_total;
        if (moveL_auto_extend_duration_ && actual_total > requested + 1e-6)
        {
            RCLCPP_INFO(logger_, "MoveL duration extended from %.3fs to %.3fs to satisfy limits",
                        requested, actual_total);
        }
    }

    vector_t PoseBasedReferenceManager::sampleArmReferenceBuffer(const ArmReferenceBuffer& buffer,
                                                                 const vector_t& fallback_pose,
                                                                 double time) const
    {
        if (buffer.path.empty())
        {
            return fallback_pose;
        }
        if (buffer.duration <= 0.0)
        {
            return fallback_pose;
        }
        const double end_time = buffer.startTime + buffer.duration;
        if (time >= end_time)
        {
            return fallback_pose;
        }
        double elapsed = std::max(0.0, time - buffer.startTime);
        for (size_t i = 0; i < buffer.segmentDurations.size(); ++i)
        {
            const double segment_duration = buffer.segmentDurations[i];
            if (elapsed <= segment_duration || i + 1 == buffer.segmentDurations.size())
            {
                const double progress = samplePoseProgress(
                    buffer.path[i], buffer.path[i + 1], elapsed, segment_duration);
                return interpolatePose7(buffer.path[i], buffer.path[i + 1], progress);
            }
            elapsed -= segment_duration;
        }
        return buffer.path.back();
    }

    bool PoseBasedReferenceManager::isArmReferenceBufferActive(const ArmReferenceBuffer& buffer,
                                                               double time) const
    {
        return !buffer.path.empty() && buffer.duration > 0.0 && time < buffer.startTime + buffer.duration;
    }

    bool PoseBasedReferenceManager::anyReferenceBufferActive(double time) const
    {
        if (isArmReferenceBufferActive(left_arm_reference_buffer_, time))
        {
            return true;
        }
        if (dual_arm_mode_ && isArmReferenceBufferActive(right_arm_reference_buffer_, time))
        {
            return true;
        }
        if (target_context_.body_target_enabled &&
            isArmReferenceBufferActive(body_reference_buffer_, time))
        {
            return true;
        }
        if (target_context_.head_target_enabled &&
            isArmReferenceBufferActive(head_reference_buffer_, time))
        {
            return true;
        }
        return false;
    }

    void PoseBasedReferenceManager::rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(
        double start_time, double minimum_duration)
    {
        double end_time = start_time + std::max(minimum_duration, 0.0);
        const auto extend_end_time = [&end_time, start_time](const ArmReferenceBuffer& buffer)
        {
            const double buffer_end_time = buffer.startTime + buffer.duration;
            if (!buffer.path.empty() && buffer.duration > 0.0 && buffer_end_time > start_time)
            {
                end_time = std::max(end_time, buffer_end_time);
            }
        };
        extend_end_time(left_arm_reference_buffer_);
        if (dual_arm_mode_)
        {
            extend_end_time(right_arm_reference_buffer_);
        }
        if (target_context_.body_target_enabled)
        {
            extend_end_time(body_reference_buffer_);
        }
        if (target_context_.head_target_enabled)
        {
            extend_end_time(head_reference_buffer_);
        }
        rebuildTargetTrajectoriesFromArmReferenceBuffers(start_time, end_time);
    }

    void PoseBasedReferenceManager::rebuildTargetTrajectoriesFromArmReferenceBuffers(double start_time,
                                                                                     double end_time)
    {
        const double actual_end_time = std::max(end_time, start_time + moveL_sample_interval_);
        // 按约 40ms 间隔估算离散点数，至少 2 个点（起点与终点）以便计算 dt。
        const size_t num_samples = std::max(
            static_cast<size_t>(std::ceil((actual_end_time - start_time) / moveL_sample_interval_)) + 1,
            static_cast<size_t>(2));
        const double dt = (actual_end_time - start_time) / static_cast<double>(num_samples - 1);

        scalar_array_t time_trajectory;
        vector_array_t state_trajectory;
        time_trajectory.reserve(num_samples);
        state_trajectory.reserve(num_samples);

        const bool whole_body = isWholeBodyTargetLayout();

        for (size_t i = 0; i < num_samples; ++i)
        {
            const double t = start_time + static_cast<double>(i) * dt;
            const vector_t left_pose = sampleArmReferenceBuffer(left_arm_reference_buffer_, left_target_state_, t);
            const vector_t right_pose = dual_arm_mode_
                ? sampleArmReferenceBuffer(right_arm_reference_buffer_, right_target_state_, t)
                : vector_t::Zero(7);

            vector_t state;
            if (whole_body && dual_arm_mode_)
            {
                const vector_t body_pose =
                    sampleArmReferenceBuffer(body_reference_buffer_, bodySegmentForAssembly(), t);
                const vector_t head_pose =
                    sampleArmReferenceBuffer(head_reference_buffer_, head_pose_7_xyzw_, t);
                state = assembleWholeBodyTargetState(
                    left_pose, right_pose, body_pose, head_pose,
                    target_context_.body_target_enabled, target_context_.head_target_enabled);
            }
            else if (dual_arm_mode_)
            {
                state = assembleDualArmReferenceState(left_pose, right_pose);
            }
            else
            {
                state = left_pose;
            }

            time_trajectory.push_back(t);
            state_trajectory.push_back(state);
        }

        vector_array_t input_trajectory(num_samples, vector_t::Zero(target_context_.input_dim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory, input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::setBodyPoseReference(const vector_t& body_pose_xyzw_7)
    {
        if (body_pose_xyzw_7.size() >= 7)
        {
            body_pose_7_xyzw_ = body_pose_xyzw_7.head<7>();
        }
    }

    void PoseBasedReferenceManager::setBodyPoseTargetOnly(const vector_t& body_pose_xyzw_7, bool update_target_trajectory)
    {
        if (body_pose_xyzw_7.size() < 7)
        {
            return;
        }

        const vector_t previous_body_target_state = body_pose_7_xyzw_;
        body_pose_7_xyzw_ = body_pose_xyzw_7.head<7>();

        if (update_target_trajectory)
        {
            updateBodyTrajectory(previous_body_target_state);
        }
    }

    void PoseBasedReferenceManager::setHeadPoseReference(const vector_t& head_pose_xyzw_7)
    {
        if (head_pose_xyzw_7.size() >= 7)
        {
            head_pose_7_xyzw_ = head_pose_xyzw_7.head<7>();
        }
    }

    void PoseBasedReferenceManager::setHeadPoseTargetOnly(const vector_t& head_pose_xyzw_7,
                                                           bool update_target_trajectory)
    {
        if (head_pose_xyzw_7.size() < 7 || !target_context_.head_target_enabled)
        {
            return;
        }
        head_pose_7_xyzw_ = head_pose_xyzw_7.head<7>();
        const double t = current_observation_.time;
        resetArmReferenceBuffer(head_reference_buffer_, head_pose_7_xyzw_, t);
        if (update_target_trajectory)
        {
            if (anyReferenceBufferActive(t))
            {
                rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(t, 0.0);
            }
            else
            {
                updateTargetTrajectory();
            }
        }
    }

    void PoseBasedReferenceManager::updateTargetTrajectory()
    {
        vector_t combined_target_state;
        if (dual_arm_mode_)
        {
            combined_target_state = assembleDualArmReferenceState(left_target_state_, right_target_state_);
        }
        else
        {
            combined_target_state = left_target_state_;
        }

        // 使用当前系统时间作为目标时间
        double target_time = current_observation_.time;

        // 创建TargetTrajectories - 只使用一个时间点
        scalar_array_t time_trajectory = {target_time};
        vector_array_t state_trajectory = {combined_target_state};
        vector_array_t input_trajectory(
            1, vector_t::Zero(target_context_.input_dim));

        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);

        // 设置到ReferenceManager
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::updateTrajectory(
        const vector_t& previous_left_target_state,
        const vector_t& previous_right_target_state)
    {
        // dual_target/stamped 等双臂 moveL 必须写入 per-arm buffer。
        // 否则后续单臂 continuous（裸 left/right_target）会因 buffer 空闲走
        // updateTargetTrajectory()，把另一臂尚未结束的插值轨迹压成终点而加速。
        const double start_time = current_observation_.time;
        setArmReferenceBufferFromWaypoints(
            left_arm_reference_buffer_, previous_left_target_state, {left_target_state_},
            start_time, moveL_duration_);
        if (dual_arm_mode_)
        {
            setArmReferenceBufferFromWaypoints(
                right_arm_reference_buffer_, previous_right_target_state, {right_target_state_},
                start_time, moveL_duration_);
        }
        else
        {
            resetArmReferenceBuffer(right_arm_reference_buffer_, right_target_state_, start_time);
        }
        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, moveL_duration_);
    }

    void PoseBasedReferenceManager::updateTrajectoryWithBody(
        const vector_t& previous_left_target_state,
        const vector_t& previous_right_target_state,
        const vector_t& previous_body_target_state)
    {
        if (!target_context_.body_target_enabled)
        {
            updateTrajectory(previous_left_target_state, previous_right_target_state);
            return;
        }

        double actual_duration = moveL_duration_;
        if (moveL_auto_extend_duration_)
        {
            actual_duration = std::max(actual_duration, minimumPoseDuration(previous_left_target_state, left_target_state_));
            actual_duration = std::max(actual_duration, minimumPoseDuration(previous_right_target_state, right_target_state_));
            actual_duration = std::max(actual_duration, minimumPoseDuration(previous_body_target_state, body_pose_7_xyzw_));
        }

        const double start_time = current_observation_.time;
        setArmReferenceBufferFromWaypoints(
            left_arm_reference_buffer_, previous_left_target_state, {left_target_state_},
            start_time, actual_duration);
        setArmReferenceBufferFromWaypoints(
            right_arm_reference_buffer_, previous_right_target_state, {right_target_state_},
            start_time, actual_duration);
        setArmReferenceBufferFromWaypoints(
            body_reference_buffer_, previous_body_target_state, {body_pose_7_xyzw_},
            start_time, actual_duration);
        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, actual_duration);
    }

    void PoseBasedReferenceManager::updateBodyTrajectory(const vector_t& previous_body_target_state)
    {
        if (!target_context_.body_target_enabled)
        {
            updateTargetTrajectory();
            return;
        }

        const double start_time = current_observation_.time;
        setArmReferenceBufferFromWaypoints(
            body_reference_buffer_, previous_body_target_state, {body_pose_7_xyzw_},
            start_time, moveL_duration_);
        // Preserve any active left/right moveL by rebuilding from all buffers.
        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, moveL_duration_);
    }

    void PoseBasedReferenceManager::updateHeadTrajectory(const vector_t& previous_head_target_state)
    {
        if (!target_context_.head_target_enabled)
        {
            updateTargetTrajectory();
            return;
        }
        const double start_time = current_observation_.time;
        setArmReferenceBufferFromWaypoints(
            head_reference_buffer_, previous_head_target_state, {head_pose_7_xyzw_},
            start_time, moveL_duration_);
        rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(start_time, moveL_duration_);
    }

} // namespace ocs2::mpc_core
