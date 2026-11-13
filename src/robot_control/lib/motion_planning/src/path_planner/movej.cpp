#include "motion_planning/path_planner/movej.h"
#include "motion_planning/common/logger.h"
#include "motion_planning/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.h"
#include <rclcpp/logging.hpp>

namespace planning
{
    moveJ::moveJ() : velocity_curve(std::make_unique<DoublesWithZeroBoundaryOfVelAndAcc>())
    {
    }

    moveJ::~moveJ() = default;

    bool moveJ::init(const TrajectoryInitParameters& traj_para)
    {
        const double epislon = 1.0e-10;
        start_pos = traj_para.start_point.joint_pos;
        delta_pos = traj_para.end_point.joint_pos - start_pos;
        period = traj_para.period;
        run_completed = false;
        size_t nr_of_joints = start_pos.getJointSize();
        for (size_t i = 0; i < nr_of_joints; i++)
        {
            if (fabs(traj_para.start_point.joint_vel(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! starting velocity of joint " << i
                    << "is not zero");
                return false;
            }

            if (fabs(traj_para.start_point.joint_acc(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! starting acceleration of joint "
                    << i << " is not zero");
                return false;
            }

            if (fabs(traj_para.start_point.joint_jerk(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! starting jerk of joint " << i
                    << "is not zero");
                return false;
            }

            if (fabs(traj_para.end_point.joint_vel(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! termination velocity of joint "
                    << i << " is not zero");
                return false;
            }

            if (fabs(traj_para.end_point.joint_acc(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! termination acceleration of joint "
                    << i << " is not zero");
                return false;
            }

            if (fabs(traj_para.end_point.joint_jerk(i)) > epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! termination jerk of joint " << i
                    << "is not zero");
                return false;
            }
        }

        double norm_vel = 1.0e10, norm_acc = 1.0e10, norm_jerk = 1.0e10;
        double tmp_vel, tmp_acc, tmp_jerk;
        bool delta_pos_is_zero = true;
        for (size_t i = 0; i < nr_of_joints; i++)
        {
            if (fabs(delta_pos(i)) < epislon)
            {
                continue;
            }
            delta_pos_is_zero = false;
            if (fabs(traj_para.parameter.max_joint_velocity(i)) < epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! max velocity of joint " << i
                    << "can't be zero");
                return false;
            }

            if (fabs(traj_para.parameter.max_joint_acceleration(i)) < epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! max acceleration of joint " << i
                    << "can't be zero");
                return false;
            }

            if (fabs(traj_para.parameter.max_joint_jerk(i)) < epislon)
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error! max jerk of joint " << i
                    << "can't be zero");
                return false;
            }

            tmp_vel = fabs(traj_para.parameter.max_joint_velocity(i) / delta_pos(i));

            tmp_acc = fabs(traj_para.parameter.max_joint_acceleration(i) / delta_pos(i));

            tmp_jerk = fabs(traj_para.parameter.max_joint_jerk(i) / delta_pos(i));

            if (tmp_vel < norm_vel)
            {
                norm_vel = tmp_vel;
            }
            if (tmp_acc < norm_acc)
            {
                norm_acc = tmp_acc;
            }
            if (tmp_jerk < norm_jerk)
            {
                norm_jerk = tmp_jerk;
            }
        }
        if (delta_pos_is_zero)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error!The starting and ending joints are "
                "the same point.");
            return false;
        }
        if (velocity_curve->init(0.0, 1.0, norm_vel, norm_acc, norm_jerk,
                                traj_para.period))
        {
            curve_total_time = velocity_curve->getTotalTime();
            curve_start_time = 0.0;
            curve_stop_time = curve_total_time;
            setBlendMessage(traj_para.start_point, traj_para.end_point);
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
    };

    void moveJ::setBlendMessage(const TrajectPoint& start_point,
                                const TrajectPoint& end_point)
    {
        const double epislon = 1.0e-10;
        blend_message.blend_start_time = 0.0;
        blend_message.blend_end_time = curve_total_time;
        blend_message.blend_point_met = false;
        blend_message.is_blend_curve = false;
        double start_blend_ratio = start_point.blend_tolerance_ratio_of_movej;
        double end_blend_ratio = end_point.blend_tolerance_ratio_of_movej;
        if (start_blend_ratio > epislon)
        {
            blend_message.start_point_setting_blend_point = true;
        }
        else
        {
            blend_message.start_point_setting_blend_point = false;
        }

        if (end_blend_ratio > epislon)
        {
            blend_message.end_point_setting_blend_point = true;
        }
        else
        {
            blend_message.end_point_setting_blend_point = false;
        }

        if ((start_blend_ratio + end_blend_ratio - 1.0) > -epislon)
        {
            start_blend_ratio =
                start_blend_ratio / (start_blend_ratio + end_blend_ratio);
            end_blend_ratio = 1.0 - start_blend_ratio;
            blend_message.blend_point_met = true;
        }

        blend_message.blend_start_time =
            velocity_curve->calculateTimeOfPos(start_blend_ratio);
        if (blend_message.blend_point_met)
        {
            blend_message.blend_end_time = blend_message.blend_start_time;
        }
        else
        {
            blend_message.blend_end_time =
                velocity_curve->calculateTimeOfPos(1.0 - end_blend_ratio);
        }
    };

    TrajectPoint moveJ::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        res.time = t;
        NormalizationPoint norm_point = velocity_curve->calculatePointAtT(t);
        res.joint_jerk = delta_pos * norm_point.jerk;
        res.joint_acc = delta_pos * norm_point.acc;
        res.joint_vel = delta_pos * norm_point.vel;
        res.joint_pos = delta_pos * norm_point.pos + start_pos;
        convertVelAccJerkUsingTimeScalingfactor(res);
        if (t-curve_total_time>=-1.0e-9)
        {
            run_completed = true;
        }
        return res;
    }

    void moveJ::convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point)
    {
        point.joint_jerk = point.joint_jerk * pow(time_scaling_factor, 3.0);
        point.joint_acc = point.joint_acc * pow(time_scaling_factor, 2.0);
        point.joint_vel = point.joint_vel * time_scaling_factor;
    };
} // namespace planning
