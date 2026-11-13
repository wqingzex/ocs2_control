#pragma once
#include "motion_planning/common/trajectory_init_parameters.h"

namespace planning
{
    class Planner
    {
    public:
        Planner()
        {
        };
        virtual bool init(const TrajectoryInitParameters& traj_para) = 0;

        virtual TrajectPoint calculatePointAtTInRealTime(double t)
        {
            checkIfRemainingTimeIsLessThanOnePeriod(t, &less_than_one_period,
                                                    &remaining_time);
            if (less_than_one_period && (blend_message.end_point_setting_blend_point || blend_message.is_blend_curve))
            {
                run_completed = true;
            }
            return calculatePointAtTInCurve(
                (t - real_start_time) * time_scaling_factor + curve_start_time);
        };
        double getTotalTime() { return total_time; };
        double getCurveTime() { return curve_total_time; };
        bool isMotionOver() { return run_completed; };
        BlendMessage get_blend_message() { return blend_message; };

        void setRealStartTime(double _real_start_time)
        {
            real_start_time = _real_start_time;
        }

        void setCurveStartTime(double _start_time)
        {
            curve_start_time = _start_time;
            curve_current_time = curve_start_time;
        };

        void setCurveStopTime(double _stop_time) { curve_stop_time = _stop_time; }

        void updateTotalTime()
        {
            total_time = (curve_stop_time - curve_start_time) / time_scaling_factor;
        }

        virtual TrajectPoint run()
        {
            curve_current_time += period * time_scaling_factor;
            current_real_time =
                (curve_current_time - curve_start_time) / time_scaling_factor +
                real_start_time;
            checkIfRemainingTimeIsLessThanOnePeriod(
                current_real_time, &less_than_one_period, &remaining_time);
            if (less_than_one_period && blend_message.end_point_setting_blend_point)
            {
                run_completed = true;
            }

            return calculatePointAtTInCurve(curve_current_time);
        }

        double getRemainTime() { return remaining_time; }

        void checkIfRemainingTimeIsLessThanOnePeriod(double real_time,
                                                     bool* is_less_than_one_period,
                                                     double* remain_time)
        {
            *remain_time = total_time - (real_time - real_start_time);
            if (*remain_time < period)
            {
                *is_less_than_one_period = true;
            }
            else
            {
                *is_less_than_one_period = false;
            }
        };
        virtual TrajectPoint calculatePointAtTInCurve(double t) = 0;
        ~Planner() = default;

    public:
        double real_start_time =
            0.0; // 设置这个轨迹开始运行时的时间，这个时间主要是用于多段轨迹运行时的开始时间
        double total_time = 0.0;
        double current_real_time = 0.0;
        double time_scaling_factor = 1.0; // 整体时间缩放因子
        double period = 0.001;
        bool run_completed = false;

    protected:
        BlendMessage blend_message;
        double curve_start_time = 0.0;
        double curve_stop_time = 0.0; // 主要是为了有转接的轨迹需要提前停止的情况
        double curve_current_time = 0.0;
        double curve_total_time = 0.0;
        double remaining_time = 0.0; // 剩余时间
        bool less_than_one_period = false;
    };
} // namespace planning
