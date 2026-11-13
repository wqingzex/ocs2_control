#include "motion_planning/common/trajectory_init_parameters.h"
#include <iostream>

namespace planning
{
    TrajectPoint::TrajectPoint(size_t n)
        : joint_pos(n), joint_vel(n), joint_acc(n), joint_jerk(n)
    {
    };

    TrajectPoint::TrajectPoint(const TrajectPoint& arg)
        : joint_pos(arg.joint_pos), joint_vel(arg.joint_vel),
          joint_acc(arg.joint_acc), joint_jerk(arg.joint_jerk), time(arg.time),
          blend_tolerance_ratio_of_movej(arg.blend_tolerance_ratio_of_movej),
          cart_pos(arg.cart_pos), cart_vel(arg.cart_vel), cart_acc(arg.cart_acc),
          cart_jerk(arg.cart_jerk), quaternion_point(arg.quaternion_point),
          is_joint_point(arg.is_joint_point)
    {
    };

    TrajectPoint::TrajectPoint(const JointVector& _joint_pos,
                               const JointVector& _joint_vel,
                               const JointVector& _joint_acc,
                               const JointVector& _joint_jerk,
                               double _blend_tolerance_ratio_of_movej)
        : joint_pos(_joint_pos), joint_vel(_joint_vel), joint_acc(_joint_acc),
          joint_jerk(_joint_jerk),
          blend_tolerance_ratio_of_movej(_blend_tolerance_ratio_of_movej)
    {
        is_joint_point = true;
    };

    TrajectPoint::TrajectPoint(const JointVector& _joint_pos,
                               const JointVector& _joint_vel,
                               const JointVector& _joint_acc,
                               double _blend_tolerance_ratio_of_movej)
        : joint_pos(_joint_pos), joint_vel(_joint_vel), joint_acc(_joint_acc),
          joint_jerk(joint_pos.getJointSize()),
          blend_tolerance_ratio_of_movej(_blend_tolerance_ratio_of_movej)
    {
        is_joint_point = true;
    };

    TrajectPoint::TrajectPoint(const JointVector& _joint_pos,
                               const JointVector& _joint_vel,
                               double _blend_tolerance_ratio_of_movej)
        : joint_pos(_joint_pos), joint_vel(_joint_vel),
          joint_acc(joint_pos.getJointSize()), joint_jerk(joint_pos.getJointSize()),
          blend_tolerance_ratio_of_movej(_blend_tolerance_ratio_of_movej)
    {
        is_joint_point = true;
    };

    TrajectPoint::TrajectPoint(const JointVector& _joint_pos,
                               double _blend_tolerance_ratio_of_movej)
        : joint_pos(_joint_pos), joint_vel(joint_pos.getJointSize()),
          joint_acc(joint_pos.getJointSize()), joint_jerk(joint_pos.getJointSize()),
          blend_tolerance_ratio_of_movej(_blend_tolerance_ratio_of_movej)
    {
        is_joint_point = true;
    };

    TrajectPoint::TrajectPoint(const Eigen::Vector3d& _cart_pos,
                               const QuaternionPoint& _quaternion_point,
                               const Eigen::Vector3d& _cart_vel,
                               const Eigen::Vector3d& _cart_acc,
                               const Eigen::Vector3d& _cart_jerk)
    {
        cart_pos=_cart_pos;
        cart_vel=_cart_vel;
        cart_acc=_cart_acc;
        cart_jerk=_cart_jerk;
        quaternion_point=_quaternion_point;
        is_joint_point = false;
    };

    TrajectPoint& TrajectPoint::operator=(const TrajectPoint& arg)
    {
        joint_pos = arg.joint_pos;
        joint_vel = arg.joint_vel;
        joint_acc = arg.joint_acc;
        joint_jerk = arg.joint_jerk;
        time = arg.time;
        blend_tolerance_ratio_of_movej = arg.blend_tolerance_ratio_of_movej;
        cart_pos = arg.cart_pos;
        cart_vel = arg.cart_vel;
        cart_acc = arg.cart_acc;
        cart_jerk = arg.cart_jerk;
        quaternion_point = arg.quaternion_point;
        is_joint_point = arg.is_joint_point;
        return *this;
    };

    TrajectoryParameter::TrajectoryParameter(const JointVector& _max_vel,
                                             const JointVector& _max_acc,
                                             const JointVector& _max_jerk)
        : max_joint_velocity(_max_vel), max_joint_acceleration(_max_acc),
          max_joint_jerk(_max_jerk)
    {
    };

    TrajectoryParameter::TrajectoryParameter(double _total_time, size_t n)
    {
        defaultParametersOfJoint(n);
        total_time = _total_time;
        time_mode = true;
    };

    void TrajectoryParameter::defaultParametersOfJoint(size_t n)
    {
        max_joint_velocity = JointVector::one(n) * 2.0;
        max_joint_acceleration = JointVector::one(n) * 4.0;
        max_joint_jerk = JointVector::one(n) * 20.0;
    };

    void TrajectoryParameter::defaultParametersOfCartesian()
    {
        max_linear_vel = 1.0;
        max_linear_acc = 2.0;
        max_linear_jerk = 10.0;
        max_angular_vel = 4.0;
        max_angular_acc = 8.0;
        max_angular_jerk = 40.0;
    };

    TrajectoryParameter::TrajectoryParameter(const JointVector& _max_vel,
                                             const JointVector& _max_acc,
                                             const JointVector& _max_jerk,
                                             double _total_time)
        : max_joint_velocity(_max_vel), max_joint_acceleration(_max_acc),
          max_joint_jerk(_max_jerk)
    {
        total_time = _total_time;
        time_mode = true;
    };

    TrajectoryParameter::TrajectoryParameter(
        const double _max_linear_vel, const double _max_linear_acc,
        const double _max_linear_jerk, const double _max_angular_vel,
        const double _max_angular_acc, const double _max_angular_jerk,
        const bool _use_slerp_interpolation_for_pose)
        : max_linear_vel(_max_linear_vel), max_linear_acc(_max_linear_acc),
          max_linear_jerk(_max_linear_jerk), max_angular_vel(_max_angular_vel),
          max_angular_acc(_max_angular_acc), max_angular_jerk(_max_angular_jerk),
          use_slerp_interpolation_for_pose(_use_slerp_interpolation_for_pose)
    {
        time_mode = false;
    };

    TrajectoryParameter::TrajectoryParameter(
        const double _max_linear_vel, const double _max_linear_acc,
        const double _max_linear_jerk, const double _max_angular_vel,
        const double _max_angular_acc, const double _max_angular_jerk,
        const double _total_time, const bool _use_slerp_interpolation_for_pose)
        : max_linear_vel(_max_linear_vel), max_linear_acc(_max_linear_acc),
          max_linear_jerk(_max_linear_jerk), max_angular_vel(_max_angular_vel),
          max_angular_acc(_max_angular_acc), max_angular_jerk(_max_angular_jerk),
          use_slerp_interpolation_for_pose(_use_slerp_interpolation_for_pose)
    {
        total_time = _total_time;
        time_mode = true;
    };

    TrajectoryParameter::TrajectoryParameter(
        const double _max_linear_vel, const double _max_linear_acc,
        const double _max_linear_jerk, const double _max_angular_vel,
        const double _max_angular_acc, const double _max_angular_jerk,
        const Eigen::Vector3d& _circular_rotation_axis,
        const Eigen::Vector3d& _center_of_circle, const double _circular_angle,
        const bool _use_slerp_interpolation_for_pose)
        : max_linear_vel(_max_linear_vel), max_linear_acc(_max_linear_acc),
          max_linear_jerk(_max_linear_jerk), max_angular_vel(_max_angular_vel),
          max_angular_acc(_max_angular_acc), max_angular_jerk(_max_angular_jerk),
          circular_rotation_axis(_circular_rotation_axis),
          center_of_circle(_center_of_circle), circular_angle(_circular_angle),
          use_slerp_interpolation_for_pose(_use_slerp_interpolation_for_pose)
    {
        no_need_to_calculate_circle_parameters_with_three_point = true;
    };

    TrajectoryParameter::TrajectoryParameter(
        const double _max_linear_vel, const double _max_linear_acc,
        const double _max_linear_jerk, const double _max_angular_vel,
        const double _max_angular_acc, const double _max_angular_jerk,
        const Eigen::Vector3d& _circular_rotation_axis,
        const Eigen::Vector3d& _center_of_circle, const double _circular_angle,
        const double _total_time, const bool _use_slerp_interpolation_for_pose)
        : max_linear_vel(_max_linear_vel), max_linear_acc(_max_linear_acc),
          max_linear_jerk(_max_linear_jerk), max_angular_vel(_max_angular_vel),
          max_angular_acc(_max_angular_acc), max_angular_jerk(_max_angular_jerk),
          circular_rotation_axis(_circular_rotation_axis),
          center_of_circle(_center_of_circle), circular_angle(_circular_angle),
          use_slerp_interpolation_for_pose(_use_slerp_interpolation_for_pose)
    {
        no_need_to_calculate_circle_parameters_with_three_point = true;
        total_time = _total_time;
        time_mode = true;
    };

    TrajectoryParameter::TrajectoryParameter(
        const bool _use_slerp_interpolation_for_pose, const double _total_time)
    {
        defaultParametersOfCartesian();
        no_need_to_calculate_circle_parameters_with_three_point = false;
        use_slerp_interpolation_for_pose = _use_slerp_interpolation_for_pose;
        total_time = _total_time;
        time_mode = true;
    };

    TrajectoryParameter::TrajectoryParameter(
        const Eigen::Vector3d& _circular_rotation_axis,
        const Eigen::Vector3d& _center_of_circle, const double _circular_angle,
        double _total_time, const bool _use_slerp_interpolation_for_pose)
        : circular_rotation_axis(_circular_rotation_axis),
          center_of_circle(_center_of_circle), circular_angle(_circular_angle),
          use_slerp_interpolation_for_pose(_use_slerp_interpolation_for_pose)
    {
        defaultParametersOfCartesian();
        no_need_to_calculate_circle_parameters_with_three_point = true;
        total_time = _total_time;
        time_mode = true;
    };

    TrajectoryParameter&
    TrajectoryParameter::operator=(const TrajectoryParameter& arg)
    {
        max_joint_velocity = arg.max_joint_velocity;
        max_joint_acceleration = arg.max_joint_acceleration;
        max_joint_jerk = arg.max_joint_jerk;
        max_linear_vel = arg.max_linear_vel;
        max_linear_acc = arg.max_linear_acc;
        max_linear_jerk = arg.max_linear_jerk;
        max_angular_vel = arg.max_angular_vel;
        max_angular_acc = arg.max_angular_acc;
        max_angular_jerk = arg.max_angular_jerk;
        circular_rotation_axis = arg.circular_rotation_axis;
        center_of_circle = arg.center_of_circle;
        circular_angle = arg.circular_angle;
        no_need_to_calculate_circle_parameters_with_three_point =
            arg.no_need_to_calculate_circle_parameters_with_three_point;
        use_slerp_interpolation_for_pose = arg.use_slerp_interpolation_for_pose;
        time_mode = arg.time_mode;
        total_time = arg.total_time;
        return *this;
    };

    TrajectoryParameter::TrajectoryParameter(const TrajectoryParameter& arg)
        : max_joint_velocity(arg.max_joint_velocity), max_joint_acceleration(arg.max_joint_acceleration),
          max_joint_jerk(arg.max_joint_jerk), max_linear_vel(arg.max_linear_vel),
          max_linear_acc(arg.max_linear_acc), max_linear_jerk(arg.max_linear_jerk),
          max_angular_vel(arg.max_angular_vel),
          max_angular_acc(arg.max_angular_acc),
          max_angular_jerk(arg.max_angular_jerk),
          circular_rotation_axis(arg.circular_rotation_axis),
          center_of_circle(arg.center_of_circle),
          circular_angle(arg.circular_angle),
          no_need_to_calculate_circle_parameters_with_three_point(
              arg.no_need_to_calculate_circle_parameters_with_three_point),
          use_slerp_interpolation_for_pose(arg.use_slerp_interpolation_for_pose),
          total_time(arg.total_time), time_mode(arg.time_mode)
    {
    };

    TrajectoryParameter::TrajectoryParameter(size_t n)
    {
        defaultParametersOfJoint(n);
    };

    TrajectoryInitParameters::TrajectoryInitParameters(
        const TrajectoryInitParameters& arg)
        : start_point(arg.start_point), end_point(arg.end_point),
          middle_point(arg.middle_point), has_middle_point(arg.has_middle_point),
          multiple_points(arg.multiple_points), parameter(arg.parameter),
          multiple_parameters(arg.multiple_parameters), period(arg.period)
    {
    };

    TrajectoryInitParameters::TrajectoryInitParameters(size_t n)
        : start_point(n), end_point(n), parameter(n)
    {
    };

    TrajectoryInitParameters::TrajectoryInitParameters(
        const TrajectPoint& _start_point, const TrajectPoint& _end_point,
        const TrajectoryParameter& para, double _period)
        : start_point(_start_point), end_point(_end_point), parameter(para),
          period(_period)
    {
    };

    TrajectoryInitParameters::TrajectoryInitParameters(
        const TrajectPoint& _start_point, const TrajectPoint& _middle_point,
        const TrajectPoint& _end_point, const TrajectoryParameter& _para,
        double _period)
        : start_point(_start_point), end_point(_end_point),
          middle_point(_middle_point), parameter(_para), period(_period)
    {
        has_middle_point = true;
    };

    TrajectoryInitParameters::TrajectoryInitParameters(
        const std::vector<TrajectPoint>& _multiple_points,
        const std::vector<TrajectoryParameter>& _multiple_parameters,
        double _period)
        : multiple_points(_multiple_points),
          multiple_parameters(_multiple_parameters), period(_period)
    {
        use_common_parameters = false;
    };

    TrajectoryInitParameters::TrajectoryInitParameters(
        const std::vector<TrajectPoint>& _multiple_points,
        TrajectoryParameter& common_parameters, double _period)
        : multiple_points(_multiple_points), parameter(common_parameters),
          period(_period)
    {
        use_common_parameters = true;
    };
} // namespace planning
