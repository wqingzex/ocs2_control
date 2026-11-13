#include "motion_planning/path_planner/movel.h"
#include "motion_planning/common/logger.h"
#include "motion_planning/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.h"
#include <rclcpp/logging.hpp>

namespace planning
{
    moveL::moveL() : velocity_curve(std::make_unique<DoublesWithZeroBoundaryOfVelAndAcc>())
    {
    }

    moveL::~moveL() = default;

    bool moveL::init(const TrajectoryInitParameters& traj_para)
    {
        start_pos = traj_para.start_point.cart_pos;
        delta_pos = traj_para.end_point.cart_pos - traj_para.start_point.cart_pos;
        linear_distance = delta_pos.norm();
        q0 = traj_para.start_point.quaternion_point;
        q1 = traj_para.end_point.quaternion_point.q;
        q0.q.normalized();
        q1.normalized();
        q0.q.checkTargetPointWithShortCurve(q1);
        double quaternion_angle = acos(q0.q.dot(q1));
        period = traj_para.period;
        run_completed = false;
        if (linear_distance < min_val && quaternion_angle < min_val)
        {
            linear_distance = 0.0;
            RCLCPP_WARN_STREAM(planning::logger(), "moveL warning: the linear distance is too small!");
            return false;
        }
        double max_linear_vel = fabs(traj_para.parameter.max_linear_vel);
        double max_linear_acc = fabs(traj_para.parameter.max_linear_acc);
        double max_linear_jerk = fabs(traj_para.parameter.max_linear_jerk);
        double max_quaternion_angular_vel =
            fabs(traj_para.parameter.max_angular_vel * 0.5);
        double max_quaternion_angular_acc =
            fabs(traj_para.parameter.max_angular_acc * 0.5);
        double max_quaternion_angular_jerk =
            fabs(traj_para.parameter.max_angular_jerk * 0.5);
        if (max_linear_vel < min_val || max_linear_acc < min_val ||
            max_linear_jerk < min_val || max_quaternion_angular_vel < min_val ||
            max_quaternion_angular_acc < min_val ||
            max_quaternion_angular_jerk < min_val)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "moveL error: the max linear or angular vel, acc, jerk is "
                "too small!");
            return false;
        }

        // 求归一化速度参数
        double norm_linear_vel = fabs(max_linear_vel / linear_distance);
        double norm_linear_acc = fabs(max_linear_acc / linear_distance);
        double norm_linear_jerk = fabs(max_linear_jerk / linear_distance);
        double norm_quaternion_vel =
            fabs(max_quaternion_angular_vel / quaternion_angle);
        double norm_quaternion_acc =
            fabs(max_quaternion_angular_acc / quaternion_angle);
        double norm_quaternion_jerk =
            fabs(max_quaternion_angular_jerk / quaternion_angle);
        double norm_vel, norm_acc, norm_jerk;
        if (linear_distance < min_val)
        {
            /// 只进行旋转
            if (quaternion_angle < min_val)
            {
                RCLCPP_WARN_STREAM(planning::logger(), "moveL warning: the linear distance and rotation angle are "
                    "too small!");
                return false;
            }
            norm_vel = norm_quaternion_vel;
            norm_acc = norm_quaternion_acc;
            norm_jerk = norm_quaternion_jerk;
        }
        else
        {
            if (quaternion_angle < min_val)
            {
                /// 只进行平移
                norm_vel = norm_linear_vel;
                norm_acc = norm_linear_acc;
                norm_jerk = norm_linear_jerk;
            }
            else /// 平移和旋转同时进行
            {
                norm_vel = std::min(norm_linear_vel, norm_quaternion_vel);
                norm_acc = std::min(norm_linear_acc, norm_quaternion_acc);
                norm_jerk = std::min(norm_linear_jerk, norm_quaternion_jerk);
            }
        }

        if (velocity_curve->init(0.0, 1.0, norm_vel, norm_acc, norm_jerk,
                                 traj_para.period))
        {
            curve_total_time = velocity_curve->getTotalTime();
            curve_start_time = 0.0;
            curve_stop_time = curve_total_time;
            if (traj_para.parameter.time_mode &&
                traj_para.parameter.total_time > period)
            {
                total_time = traj_para.parameter.total_time;
                time_scaling_factor = curve_total_time / total_time;
            }
            else
            {
                total_time = curve_total_time;
                time_scaling_factor = 1.0;
            }
            return true;
        }
        else
        {
            return false;
        }

        return true;
    }

    TrajectPoint moveL::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        res.time = t;
        NormalizationPoint norm_point = velocity_curve->calculatePointAtT(t);
        // 计算直线位置
        res.cart_pos = start_pos + norm_point.pos * delta_pos;
        res.cart_vel = norm_point.vel * delta_pos;
        res.cart_acc = norm_point.acc * delta_pos;
        res.cart_jerk = norm_point.jerk * delta_pos;
        // 计算四元数插值
        res.quaternion_point = q0.slerp(q1, norm_point.pos, norm_point.vel,
                                        norm_point.acc, norm_point.jerk);

        convertVelAccUsingTimeScalingfactor(res);
        if (t - curve_total_time >= -1.0e-9)
        {
            run_completed = true;
        }

        return res;
    }

    void moveL::convertVelAccUsingTimeScalingfactor(TrajectPoint& point)
    {
        point.cart_jerk *= pow(time_scaling_factor, 3.0);
        point.cart_acc *= pow(time_scaling_factor, 2.0);
        point.cart_vel *= time_scaling_factor;

        point.quaternion_point.angle_jerk_in_space_system *=
            pow(time_scaling_factor, 3.0);
        point.quaternion_point.angle_acc_in_space_system *=
            pow(time_scaling_factor, 2.0);
        point.quaternion_point.angle_vel_in_space_system *= time_scaling_factor;
        point.quaternion_point.angle_jerk_in_body_system *=
            pow(time_scaling_factor, 3.0);
        point.quaternion_point.angle_acc_in_body_system *=
            pow(time_scaling_factor, 2.0);
        point.quaternion_point.angle_vel_in_body_system *= time_scaling_factor;
    }
} // namespace planning
