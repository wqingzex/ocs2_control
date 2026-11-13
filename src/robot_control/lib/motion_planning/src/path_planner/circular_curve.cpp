#include "motion_planning/path_planner/circular_curve.h"
#include "motion_planning/common/logger.h"
#include "motion_planning/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.h"
#include <rclcpp/logging.hpp>

namespace planning
{

     CircularCurver::CircularCurver(): velocity_curve(std::make_unique<DoublesWithZeroBoundaryOfVelAndAcc>())
    {
    }
    CircularCurver::~CircularCurver()=default;
    bool CircularCurver::init(const TrajectoryInitParameters& traj_para)
    {
        period = traj_para.period;
        run_completed = false;
        // 先规划圆形参数:
        if (traj_para.parameter
                     .no_need_to_calculate_circle_parameters_with_three_point)
        {
            if (!setCircularParameterWithInputPara(traj_para))
            {
                return false;
            }
        }
        else
        {
            if (!calculatCircularParameterWithThreePoint(traj_para))
            {
                return false;
            }
        }

        if (circular_angle < 0.001)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "circular angle is too small!");
            return false;
        }
        double max_linear_vel = fabs(traj_para.parameter.max_linear_vel);
        double max_linear_acc = fabs(traj_para.parameter.max_linear_acc);
        double max_linear_jerk = fabs(traj_para.parameter.max_linear_jerk);
        double max_angular_vel = fabs(traj_para.parameter.max_angular_vel);
        double max_angular_acc = fabs(traj_para.parameter.max_angular_acc);
        double max_angular_jerk = fabs(traj_para.parameter.max_angular_jerk);
        if (max_linear_vel < min_val || max_linear_acc < min_val ||
            max_linear_jerk < min_val || max_angular_vel < min_val ||
            max_angular_acc < min_val || max_angular_jerk < min_val)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "moveL error: the max linear or angular vel, acc, jerk is "
                "too small!");
            return false;
        }

        q0 = traj_para.start_point.quaternion_point;
        q1 = traj_para.end_point.quaternion_point.q;
        q0.q.normalized();
        q1.normalized();
        q0.q.checkTargetPointWithShortCurve(q1);
        double quaternion_angle = acos(q0.q.dot(q1));
        use_slerp_interpolation_for_pose =
            traj_para.parameter.use_slerp_interpolation_for_pose;
        // 求归一化速度参数
        double circular_max_angulr_vel =
            max_linear_vel / radius_of_circle;
        double circular_max_angular_acc =
            max_linear_acc / radius_of_circle;
        double circular_max_angular_jerk =
            max_linear_jerk / radius_of_circle;
        double norm_circ_vel = fabs(circular_max_angulr_vel / circular_angle);
        double norm_circ_acc = fabs(circular_max_angular_acc / circular_angle);
        double norm_circ_jerk = fabs(circular_max_angular_jerk / circular_angle);
        double norm_quaternion_vel, norm_quaternion_acc, norm_quaternion_jerk;
        if (use_slerp_interpolation_for_pose)
        {
            norm_quaternion_vel =
                fabs(0.5 * max_angular_vel / quaternion_angle);
            norm_quaternion_acc =
                fabs(0.5 * max_angular_acc / quaternion_angle);
            norm_quaternion_jerk =
                fabs(0.5 * max_angular_jerk / quaternion_angle);
        }
        else
        {
            norm_quaternion_vel =
                fabs(max_angular_vel / circular_angle);
            norm_quaternion_acc =
                fabs(max_angular_acc / circular_angle);
            norm_quaternion_jerk =
                fabs(max_angular_jerk / circular_angle);
        }
        double norm_vel = 1.0e10, norm_acc = 1.0e10, norm_jerk = 1.0e10;
        if (use_slerp_interpolation_for_pose&& quaternion_angle<min_val)
        {
            norm_vel = norm_circ_vel;
            norm_acc = norm_circ_acc;
            norm_jerk = norm_circ_jerk;
        }
        else
        {
            norm_vel = std::min(norm_circ_vel, norm_quaternion_vel);
            norm_acc = std::min(norm_circ_acc, norm_quaternion_acc);
            norm_jerk = std::min(norm_circ_jerk, norm_quaternion_jerk);
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
    }

    TrajectPoint CircularCurver::calculatePointAtTInCurve(double t)
    {
        TrajectPoint res;
        res.time = t;
        NormalizationPoint norm_point = velocity_curve->calculatePointAtT(t);
        // 计算在圆形平面中的位置、速度、加速度、加加速度
        Eigen::Vector3d pos_in_circle_plane, vel_in_circle_plane, acc_in_circle_plane,
                        jerk_in_circle_plane;
        double theta = circular_angle * norm_point.pos;
        double dth = circular_angle * norm_point.vel;
        double ddth = circular_angle * norm_point.acc;
        double dddth = circular_angle * norm_point.jerk;
        pos_in_circle_plane(0) = radius_of_circle * cos(theta);
        pos_in_circle_plane(1) = radius_of_circle * sin(theta);
        pos_in_circle_plane(2) = 0.0;

        vel_in_circle_plane(0) = -radius_of_circle * sin(theta) * dth;
        vel_in_circle_plane(1) = radius_of_circle * cos(theta) * dth;
        vel_in_circle_plane(2) = 0.0;

        acc_in_circle_plane(0) = -radius_of_circle * cos(theta) * dth * dth -
            radius_of_circle * sin(theta) * ddth;
        acc_in_circle_plane(1) = -radius_of_circle * sin(theta) * dth * dth +
            radius_of_circle * cos(theta) * ddth;
        acc_in_circle_plane(2) = 0.0;

        jerk_in_circle_plane(0) = radius_of_circle * sin(theta) * dth * dth * dth -
            3.0 * radius_of_circle * cos(theta) * dth * ddth -
            radius_of_circle * sin(theta) * dddth;
        jerk_in_circle_plane(1) = -radius_of_circle * cos(theta) * dth * dth * dth -
            3.0 * radius_of_circle * sin(theta) * dth * ddth +
            radius_of_circle * cos(theta) * dddth;
        jerk_in_circle_plane(2) = 0.0;
        // 转换到真实空间
        res.cart_pos = circular_plane_system * pos_in_circle_plane + center_of_circle;
        res.cart_vel = circular_plane_system * vel_in_circle_plane;
        res.cart_acc = circular_plane_system * acc_in_circle_plane;
        res.cart_jerk = circular_plane_system * jerk_in_circle_plane;
        // 计算四元数
        if (use_slerp_interpolation_for_pose)
        {
            res.quaternion_point = q0.slerp(q1, norm_point.pos, norm_point.vel,
                                            norm_point.acc, norm_point.jerk);
        }
        else
        {
            res.quaternion_point = q0.rotateAroundAxisByTheta(
                circular_rotation_axis, theta, dth, ddth, dddth, false);
        }

        convertVelAccJerkUsingTimeScalingfactor(res);
        if (t - curve_total_time >= -1.0e-9)
        {
            run_completed = true;
        }
        return res;
    }

    void CircularCurver::convertVelAccJerkUsingTimeScalingfactor(
        TrajectPoint& point)
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
    };

    bool calculate_center_of_circle(const Eigen::Vector3d& ps,
                                    const Eigen::Vector3d pm,
                                    const Eigen::Vector3d pe,
                                    Eigen::Vector3d& center_of_circ,
                                    Eigen::Vector3d& z_axis, const double min_val)
    {
        Eigen::Vector3d ps_pm = pm - ps;
        Eigen::Vector3d pm_pe = pe - pm;
        z_axis = ps_pm.cross(pm_pe);
        if (z_axis.norm() < min_val)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Three points colinear cannot form a circle curve!");
            return false;
        }

        z_axis.normalize();
        // 以ps为其其起点，ps-pm为x轴建立平面
        Eigen::Vector3d x_axis = ps_pm.normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();
        Eigen::Matrix3d R;
        R << x_axis(0), y_axis(0), z_axis(0), x_axis(1), y_axis(1), z_axis(1),
            x_axis(2), y_axis(2), z_axis(2);
        Eigen::Matrix3d Rinv = R.transpose();
        Eigen::Vector3d ps_in_circ = Rinv * (ps - ps);
        Eigen::Vector3d pm_in_circ = Rinv * (pm - ps);
        Eigen::Vector3d pe_in_circ = Rinv * (pe - ps);
        // 求在二维平面的圆心，中垂想线的交点
        double x1 = ps_in_circ(0), y1 = ps_in_circ(1);
        double x2 = pm_in_circ(0), y2 = pm_in_circ(1);
        double x3 = pe_in_circ(0), y3 = pe_in_circ(1);
        Eigen::Matrix2d A;
        A << 2.0 * (x2 - x1), 2.0 * (y2 - y1), 2.0 * (x3 - x2), 2.0 * (y3 - y2);
        Eigen::Vector2d b, center_in_circ_plane;
        b << x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1,
            x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2;
        center_in_circ_plane = A.inverse() * b;
        Eigen::Vector3d tmp;
        tmp << center_in_circ_plane(0), center_in_circ_plane(1), 0.0;
        center_of_circ = R * tmp + ps;
        return true;
    }

    bool CircularCurver::calculatCircularParameterWithThreePoint(
        const TrajectoryInitParameters& traj_para)
    {
        Eigen::Vector3d ps = traj_para.start_point.cart_pos;
        Eigen::Vector3d pm = traj_para.middle_point.cart_pos;
        Eigen::Vector3d pe = traj_para.end_point.cart_pos;
        Eigen::Vector3d z_axis;
        // 求圆弧圆心
        if (!calculate_center_of_circle(ps, pm, pe, center_of_circle, z_axis,
                                        min_val))
        {
            return false;
        }

        circular_rotation_axis = z_axis;

        // 转换坐标系
        Eigen::Vector3d x_axis = ps - center_of_circle;
        radius_of_circle = x_axis.norm();
        x_axis.normalize();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        circular_plane_system << x_axis(0), y_axis(0), z_axis(0), x_axis(1),
            y_axis(1), z_axis(1), x_axis(2), y_axis(2), z_axis(2);
        Eigen::Vector3d pe_in_circle_plane_system =
            circular_plane_system.inverse() * (pe - center_of_circle);
        if (traj_para.parameter.circular_angle > min_val)
        {
            circular_angle = traj_para.parameter.circular_angle;
        }
        else
        {
            circular_angle =
                atan2(pe_in_circle_plane_system(1), pe_in_circle_plane_system(0));
        }
        if (circular_angle < -min_val)
        {
            circular_angle += M_PI * 2.0;
        }

        return true;
    };

    bool CircularCurver::setCircularParameterWithInputPara(
        const TrajectoryInitParameters& traj_para)
    {
        Eigen::Vector3d z_axis = traj_para.parameter.circular_rotation_axis;
        Eigen::Vector3d ps = traj_para.start_point.cart_pos;
        if (z_axis.norm() < min_val)
        {
            return false;
        }
        z_axis.normalize();
        circular_rotation_axis = z_axis;
        center_of_circle = traj_para.parameter.center_of_circle;
        // 转换坐标系
        Eigen::Vector3d x_axis = ps - center_of_circle;
        radius_of_circle = x_axis.norm();
        x_axis.normalize();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        circular_plane_system << x_axis(0), y_axis(0), z_axis(0), x_axis(1),
            y_axis(1), z_axis(1), x_axis(2), y_axis(2), z_axis(2);
        circular_angle = traj_para.parameter.circular_angle;
        if (circular_angle < -min_val)
        {
            RCLCPP_ERROR_STREAM(planning::logger(), "Negative circular rotation angles are not allowed.Circular "
                "rotation must be peformed counterclockwise around the axis "
                "of rotation!");
            return false;
        }

        return true;
    };
} // namespace planning
