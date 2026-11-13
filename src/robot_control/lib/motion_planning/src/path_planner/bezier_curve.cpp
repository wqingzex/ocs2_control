#include "motion_planning/path_planner/bezier_curve.h"
#include "motion_planning/common/logger.h"
#include <vector>
#include <rclcpp/logging.hpp>

namespace planning
{
    bool BezierCurve::init(const TrajectoryInitParameters& traj_para)
    {
        double minval = 1.0e-9;
        double maxtime_for_controllable_position;
        start_point = traj_para.start_point;
        end_point = traj_para.end_point;

        // step1：计算合理的时间，这里的时间先按照我之前的优化进行，后续有其他方式和会进行修改
        //  这里先考虑有中间点的转接模式
        if (traj_para.has_middle_point)
        {
            middle_point = traj_para.middle_point;
            blend_message.is_blend_curve = true;
            // 先计算当前边界条件下的最大时间
            bool vel_of_both_sides_is_not_zero = false;
            if (!calculateMaxTimeOf5thBezierCurveWithPosConstraintInJointSpace(
                &maxtime_for_controllable_position,
                &vel_of_both_sides_is_not_zero))
            {
                return false;
            }

            totaltime_of_bezier = maxtime_for_controllable_position;
            if (traj_para.parameter
                         .time_mode)
            {
                // 如果给了一个原来的参考时间，这里可以作一些参考
                double tmptime;
                if (vel_of_both_sides_is_not_zero)
                {
                    tmptime = traj_para.parameter.total_time * 0.5;
                }
                else
                {
                    tmptime = traj_para.parameter.total_time;
                }

                if (tmptime < maxtime_for_controllable_position)
                {
                    totaltime_of_bezier = tmptime;
                }
            }
        }
        else
        {
            return false;
        }
        if (totaltime_of_bezier < minval)
        {
            return false;
        }
        // 圆整一下贝塞尔的时间
        int timetick = ceil(totaltime_of_bezier / traj_para.period);
        total_time = (double)timetick * period;
        totaltime_of_bezier = total_time;
        curve_total_time = total_time;
        curve_start_time = 0.0;
        curve_stop_time = curve_total_time;
        time_scaling_factor = 1.0;
        run_completed = false;

        // 计算贝塞尔曲线的控制点
        calculateControlPointOf5thBezierCurveInJointSpace(
            traj_para.start_point.joint_pos, traj_para.start_point.joint_vel,
            traj_para.start_point.joint_acc, traj_para.end_point.joint_pos,
            traj_para.end_point.joint_vel, traj_para.end_point.joint_acc);
        calculateVelControlPointOf5thBezierCurveInJointSpace();
        calculateAccControlPointOf5thBezierCurveInJointSpace();
        calculateJerkControlPointOf5thBezierCurveInJointSpace();
        return true;
    }

    TrajectPoint BezierCurve::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        // 因为现在是只有关节运动，后续如果有笛卡尔的运动，这里会做区分
        res = calculatePointAtTof5thBezierCurveInJointSpace(t);
        convertVelAccJerkUsingTimeScalingfactor(res);
        return res;
    };

    bool BezierCurve::
    calcualeNormBoundaryConditionsWithMiddlePointInJointSpaceOf5thBezier(
        double* norm_v0, double* norm_a0, double* r0, double* norm_v5,
        double* norm_a5, double* r1)
    {
        JointVector s_m = middle_point.joint_pos - start_point.joint_pos;
        JointVector m_e = end_point.joint_pos - middle_point.joint_pos;
        *r0 = s_m.norm();
        *r1 = m_e.norm();
        if (!start_point.joint_vel.theProjectionOntoOtherVector(s_m, norm_v0) ||
            !start_point.joint_acc.theProjectionOntoOtherVector(s_m, norm_a0) ||
            !end_point.joint_vel.theProjectionOntoOtherVector(m_e, norm_v5) ||
            !end_point.joint_acc.theProjectionOntoOtherVector(m_e, norm_a5))
        {
            return false;
        }
        return true;
    };

    bool BezierCurve::calculateMaxTimeOf5thBezierCurveStartConstraintInNormSpace(
        double v0, double a0, double r0, double* maxtime)
    {
        double minval = 1.0e-9;
        if (v0 < minval)
        {
            if (a0 > 0)
            {
                *maxtime = sqrt(20.0 * r0 / a0);
                return true;
            }
            return false;
        }
        double tmptime;
        if (a0 < 0)
        {
            *maxtime = -4.0 * v0 / a0;
            if (4.0 * v0 * v0 + 5.0 * a0 * r0 > 0)
            {
                tmptime = -(4.0 * v0 - sqrt(16.0 * v0 * v0 + 20.0 * a0 * r0)) / a0;
                if (tmptime < *maxtime)
                {
                    *maxtime = tmptime;
                }
            }
        }
        else if (a0 == 0)
        {
            *maxtime = 2.5 * r0 / v0;
        }
        else
        {
            // a0>0
            *maxtime = (-4.0 * v0 + sqrt(16.0 * v0 * v0 + 20.0 * a0 * r0)) / a0;
        }
        return true;
    };

    bool BezierCurve::calculateMaxTimeOf5thBezierCurveEndConstraintInNormSpace(
        double v5, double a5, double r1, double* maxtime)
    {
        double minval = 1.0e-9;
        if (v5 < minval)
        {
            if (a5 < 0)
            {
                *maxtime = sqrt(20.0 * r1 / -a5);
                return true;
            }
            return false;
        }
        double tmptime;
        if (a5 > 0)
        {
            *maxtime = 4.0 * v5 / a5;
            if (4.0 * v5 * v5 - 5.0 * a5 * r1 > 0)
            {
                tmptime = (4.0 * v5 - sqrt(16.0 * v5 * v5 - 20.0 * a5 * r1)) / a5;
                if (tmptime < *maxtime)
                {
                    *maxtime = tmptime;
                }
            }
        }
        else if (a5 == 0)
        {
            *maxtime = 2.5 * r1 / v5;
        }
        else
        {
            // a5<0
            *maxtime = (4.0 * v5 - sqrt(16.0 * v5 * v5 - 20.0 * a5 * r1)) / a5;
        }
        return true;
    }

    static double signPara(double x)
    {
        double kmath_epison = 1.0e-9;
        if (x > kmath_epison)
        {
            return 1.0;
        }
        else if (x < -kmath_epison)
        {
            return -1.0;
        }
        else
        {
            return 0.0;
        }
    }

    bool BezierCurve::calculateMaxTimeOf5thBezierCurveWithPosConstraintInJointSpace(
        double* max_time, bool* vel_of_both_sides_is_not_zero)
    {
        bool calculate_max_time_separately = true;
        if (calculate_max_time_separately)
        {
            // 各个关节分别计算最大时间，现在使用这种方式计算，之前使用投影计算不知道为什么计算的时间要很短
            JointVector s_m = middle_point.joint_pos - start_point.joint_pos;
            JointVector m_e = end_point.joint_pos - middle_point.joint_pos;
            std::vector<double> calculate_max_times;
            size_t nr_of_joints = start_point.joint_pos.getJointSize();
            double tmp_time;
            double direction = 1.0;
            bool all_start_vel_is_zero = true;
            bool all_end_vel_is_zero = true;
            // 开始一侧的保证位置的最大时间计算
            for (size_t i = 0; i < nr_of_joints; i++)
            {
                direction = signPara(s_m(i));
                if (calculateMaxTimeOf5thBezierCurveStartConstraintInNormSpace(
                    start_point.joint_vel(i) * direction,
                    start_point.joint_acc(i) * direction, s_m(i) * direction,
                    &tmp_time))
                {
                    calculate_max_times.push_back(tmp_time);
                    all_start_vel_is_zero = false;
                }
            }

            for (size_t i = 0; i < nr_of_joints; i++)
            {
                direction = signPara(m_e(i));
                if (calculateMaxTimeOf5thBezierCurveEndConstraintInNormSpace(
                    end_point.joint_vel(i) * direction,
                    end_point.joint_acc(i) * direction, m_e(i) * direction,
                    &tmp_time))
                {
                    calculate_max_times.push_back(tmp_time);
                    all_end_vel_is_zero = false;
                }
            }

            if (!all_start_vel_is_zero && !all_end_vel_is_zero)
            {
                *vel_of_both_sides_is_not_zero = true;
            }
            else
            {
                *vel_of_both_sides_is_not_zero = false;
            }
            if (calculate_max_times.empty())
            {
                return false;
            }
            else
            {
                auto it = std::min_element(calculate_max_times.begin(),
                                           calculate_max_times.end());
                *max_time = *it;
                return true;
            }
        }
        else
        {
            // 先计算当前边界条件下的转接情况
            double norm_v0, norm_a0, r0, norm_v5, norm_a5, r5, tmptime_s, tmptime_e;
            if (!calcualeNormBoundaryConditionsWithMiddlePointInJointSpaceOf5thBezier(
                &norm_v0, &norm_a0, &r0, &norm_v5, &norm_a5, &r5))
            {
                RCLCPP_ERROR_STREAM(planning::logger(), "normlization error!");
                return false;
            }

            bool start_condition =
                calculateMaxTimeOf5thBezierCurveStartConstraintInNormSpace(
                    norm_v0, norm_a0, r0, &tmptime_s);
            bool end_condition =
                calculateMaxTimeOf5thBezierCurveEndConstraintInNormSpace(
                    norm_v5, norm_a5, r5, &tmptime_e);
            *vel_of_both_sides_is_not_zero = false;
            if (start_condition)
            {
                *max_time = tmptime_s;
                if (end_condition)
                {
                    *vel_of_both_sides_is_not_zero = true;
                    if (tmptime_e < tmptime_s)
                    {
                        *max_time = tmptime_e;
                    }
                }
            }
            else
            {
                if (end_condition)
                {
                    *max_time = tmptime_e;
                }
                else
                {
                    return false;
                }
            }
            return true;
        }
    };

    void BezierCurve::calculateControlPointOf5thBezierCurveInJointSpace(
        const JointVector& p0, const JointVector& v0, const JointVector& a0,
        const JointVector& p5, const JointVector& v5, const JointVector& a5)
    {
        JointVector norm_v0 = v0 * totaltime_of_bezier;
        JointVector norm_a0 = a0 * totaltime_of_bezier * totaltime_of_bezier;
        JointVector norm_v5 = v5 * totaltime_of_bezier;
        JointVector norm_a5 = a5 * totaltime_of_bezier * totaltime_of_bezier;
        JointVector p1 = p0 + norm_v0 / 5.0;
        JointVector p4 = p5 - norm_v5 / 5.0;
        JointVector p2 = p1 * 2.0 - p0 + norm_a0 / 20.0;
        JointVector p3 = p4 * 2.0 - p5 + norm_a5 / 20.0;
        control_point_of_fifth_order_bezier_curve_in_joint_space[0] = p0;
        control_point_of_fifth_order_bezier_curve_in_joint_space[1] = p1;
        control_point_of_fifth_order_bezier_curve_in_joint_space[2] = p2;
        control_point_of_fifth_order_bezier_curve_in_joint_space[3] = p3;
        control_point_of_fifth_order_bezier_curve_in_joint_space[4] = p4;
        control_point_of_fifth_order_bezier_curve_in_joint_space[5] = p5;
    }

    void BezierCurve::calculateVelControlPointOf5thBezierCurveInJointSpace()
    {
        for (size_t i = 0; i < 5; i++)
        {
            vel_control_point_of_fifth_order_bezier_curve_in_joint_space[i] =
                (control_point_of_fifth_order_bezier_curve_in_joint_space[i + 1] -
                    control_point_of_fifth_order_bezier_curve_in_joint_space[i]) *
                5.0;
        }
    };

    void BezierCurve::calculateAccControlPointOf5thBezierCurveInJointSpace()
    {
        for (size_t i = 0; i < 4; i++)
        {
            acc_control_point_of_fifth_order_bezier_curve_in_joint_space[i] =
                (vel_control_point_of_fifth_order_bezier_curve_in_joint_space[i + 1] -
                    vel_control_point_of_fifth_order_bezier_curve_in_joint_space[i]) *
                4.0;
        }
    };

    void BezierCurve::calculateJerkControlPointOf5thBezierCurveInJointSpace()
    {
        for (size_t i = 0; i < 3; i++)
        {
            jerk_control_point_of_fifth_order_bezier_curve_in_joint_space[i] =
                (acc_control_point_of_fifth_order_bezier_curve_in_joint_space[i + 1] -
                    acc_control_point_of_fifth_order_bezier_curve_in_joint_space[i]) *
                3.0;
        }
    };

    TrajectPoint
    BezierCurve::calculatePointAtTof5thBezierCurveInJointSpace(double t)
    {
        TrajectPoint res;
        res.time = t;
        res.joint_pos = calculatePosAtTof5thBezierCurveInJointSpace(t);
        res.joint_vel = calculateVelAtTof5thBezierCurveInJointSpace(t);
        res.joint_acc = calculateAccAtTof5thBezierCurveInJointSpace(t);
        res.joint_jerk = calculateJerkAtTof5thBezierCurveInJointSpace(t);
        if ((t - totaltime_of_bezier) > -1.0e-6)
        {
            run_completed = true;
        }

        return res;
    }

    JointVector BezierCurve::calculatePosAtTof5thBezierCurveInJointSpace(double t)
    {
        size_t n = control_point_of_fifth_order_bezier_curve_in_joint_space[0]
            .getJointSize();
        JointVector res(n);
        double tau = t / totaltime_of_bezier;
        double c[6];
        c[0] = pow(1.0 - tau, 5.0);
        c[1] = 5.0 * tau * pow(1.0 - tau, 4.0);
        c[2] = 10.0 * pow(tau, 2.0) * pow(1.0 - tau, 3.0);
        c[3] = 10.0 * pow(tau, 3.0) * pow(1.0 - tau, 2.0);
        c[4] = 5.0 * pow(tau, 4.0) * (1.0 - tau);
        c[5] = pow(tau, 5.0);
        for (size_t i = 0; i < 6; i++)
        {
            res = res +
                control_point_of_fifth_order_bezier_curve_in_joint_space[i] * c[i];
        }
        return res;
    }

    JointVector BezierCurve::calculateVelAtTof5thBezierCurveInJointSpace(double t)
    {
        size_t n = control_point_of_fifth_order_bezier_curve_in_joint_space[0]
            .getJointSize();
        JointVector res(n);
        double tau = t / totaltime_of_bezier;
        double c[6];
        c[0] = -5.0 * pow(1.0 - tau, 4.0);
        c[1] = 5.0 * (1.0 - 5.0 * tau) * pow(1.0 - tau, 3.0);
        c[2] = 10.0 * tau * (2.0 - 5.0 * tau) * pow(1.0 - tau, 2.0);
        c[3] = 10.0 * pow(tau, 2.0) * (3.0 - 5.0 * tau) * (1.0 - tau);
        c[4] = 5.0 * pow(tau, 3.0) * (4.0 - 5.0 * tau);
        c[5] = 5.0 * pow(tau, 4.0);
        for (size_t i = 0; i < 6; i++)
        {
            res = res +
                control_point_of_fifth_order_bezier_curve_in_joint_space[i] * c[i];
        }
        res = res / totaltime_of_bezier;
        return res;
    }

    JointVector BezierCurve::calculateAccAtTof5thBezierCurveInJointSpace(double t)
    {
        size_t n = control_point_of_fifth_order_bezier_curve_in_joint_space[0]
            .getJointSize();
        JointVector res(n);
        double tau = t / totaltime_of_bezier;
        double c[6];
        c[0] = 20.0 * pow(1.0 - tau, 3.0);
        c[1] = -20.0 * (2.0 - 5.0 * tau) * pow(1.0 - tau, 2.0);
        c[2] = 20.0 - 180.0 * tau + 360.0 * pow(tau, 2.0) - 200.0 * pow(tau, 3.0);
        c[3] = 60.0 * tau - 240.0 * pow(tau, 2.0) + 200.0 * pow(tau, 3.0);
        c[4] = 60.0 * pow(tau, 2.0) - 100.0 * pow(tau, 3.0);
        c[5] = 20.0 * pow(tau, 3.0);
        for (size_t i = 0; i < 6; i++)
        {
            res = res +
                control_point_of_fifth_order_bezier_curve_in_joint_space[i] * c[i];
        }
        res = res / pow(totaltime_of_bezier, 2.0);
        return res;
    }

    JointVector
    BezierCurve::calculateJerkAtTof5thBezierCurveInJointSpace(double t)
    {
        size_t n = control_point_of_fifth_order_bezier_curve_in_joint_space[0]
            .getJointSize();
        JointVector res(n);
        double tau = t / totaltime_of_bezier;
        double c[6];
        c[0] = -60.0 * pow(1.0 - tau, 2.0);
        c[1] = 180.0 - 480.0 * tau + 300.0 * pow(tau, 2.0);
        c[2] = -180.0 + 720.0 * tau - 600.0 * pow(tau, 2.0);
        c[3] = 60.0 - 480.0 * tau + 600.0 * pow(tau, 2.0);
        c[4] = 120.0 * tau - 300.0 * pow(tau, 2.0);
        c[5] = 60.0 * pow(tau, 2.0);
        for (size_t i = 0; i < 6; i++)
        {
            res = res +
                control_point_of_fifth_order_bezier_curve_in_joint_space[i] * c[i];
        }
        res = res / pow(totaltime_of_bezier, 3.0);
        return res;
    }

    void BezierCurve::convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point)
    {
        point.joint_jerk = point.joint_jerk * pow(time_scaling_factor, 3.0);
        point.joint_acc = point.joint_acc * pow(time_scaling_factor, 2.0);
        point.joint_vel = point.joint_vel * time_scaling_factor;
    };
} // namespace planning
