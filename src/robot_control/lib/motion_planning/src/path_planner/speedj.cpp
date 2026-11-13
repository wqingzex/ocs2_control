#include "motion_planning/path_planner/speedj.h"
#include "motion_planning/common/logger.h"
#include <rclcpp/logging.hpp>

#include <cmath>

#include "motion_planning/velocity_planner/plan_velocity_using_doubles.h"

namespace planning
{
    SpeedJ::SpeedJ() : velocity_curve(std::make_unique<PlanVelocityUsingDoubles>())
    {
    }

    SpeedJ::~SpeedJ() = default;

    bool SpeedJ::init(const TrajectoryInitParameters& traj_para)
    {
        const double epislon = 1.0e-10;
        start_pos = traj_para.start_point.joint_pos;
        start_vel = traj_para.start_point.joint_vel;
        delta_vel = traj_para.end_point.joint_vel - traj_para.start_point.joint_vel;
        period = traj_para.period;
        run_completed = false;
        size_t nr_of_joints = start_pos.getJointSize();
        for (size_t i = 0; i < nr_of_joints; i++)
        {
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

        double norm_acc = 1.0e10, norm_jerk = 1.0e10;
        double tmp_acc, tmp_jerk;
        bool delta_vel_is_zero = true;
        for (size_t i = 0; i < nr_of_joints; i++)
        {
            if (fabs(delta_vel(i)) < epislon)
            {
                continue;
            }
            delta_vel_is_zero = false;

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

            tmp_acc = fabs(traj_para.parameter.max_joint_acceleration(i) / delta_vel(i));

            tmp_jerk = fabs(traj_para.parameter.max_joint_jerk(i) / delta_vel(i));

            if (tmp_acc < norm_acc)
            {
                norm_acc = tmp_acc;
            }
            if (tmp_jerk < norm_jerk)
            {
                norm_jerk = tmp_jerk;
            }
        }
        if (delta_vel_is_zero)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Initial condition error!The starting and ending joints vel "
                "are equal");
            return false;
        }
        if (velocity_curve->init(0.0, 0.0, 1.0, norm_acc, norm_jerk, traj_para.period,
                                 traj_para.parameter.total_time))
        {
            curve_total_time = velocity_curve->getTotalTime();
            curve_start_time = 0.0;
            curve_stop_time = curve_total_time;
            total_time = curve_total_time;
            time_scaling_factor = 1.0;
            return true;
        }
        else
        {
            return false;
        }
    }
    double SpeedJ::calculateDecDistanceOfV(double delta_v,double max_acc,double max_jerk)
    {
        PlanVelocityUsingDoubles tmp_vel;
        tmp_vel.init(0,0,fabs(delta_v),max_acc,max_jerk);
        return tmp_vel.getAccDistance();
    }


    TrajectPoint SpeedJ::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        res.time = t;
        if (t > curve_total_time)
        {
            t = curve_total_time;
        }

        NormalizationPoint norm_point = velocity_curve->calculatePointAtT(t);
        res.joint_jerk = delta_vel * norm_point.jerk;
        res.joint_acc = delta_vel * norm_point.acc;
        res.joint_vel = delta_vel * norm_point.vel + start_vel;

        res.joint_pos = delta_vel * norm_point.pos + start_vel * t + start_pos;
        convertVelAccJerkUsingTimeScalingfactor(res);
        if (t - curve_total_time >= -1.0e-9)
        {
            run_completed = true;
        }
        return res;
    }

    void SpeedJ::convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point)
    {
        point.joint_jerk = point.joint_jerk * pow(time_scaling_factor, 3.0);
        point.joint_acc = point.joint_acc * pow(time_scaling_factor, 2.0);
        point.joint_vel = point.joint_vel * time_scaling_factor;
    };
} // namespace planning
