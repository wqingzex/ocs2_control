#pragma once
#include "joint_vector.h"
#include "motion_planning/common/quaternion.h"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace planning
{
    class TrajectPoint
    {
    public:
        TrajectPoint()
        {
        };
        explicit TrajectPoint(size_t n);
        TrajectPoint(const TrajectPoint& arg);
        TrajectPoint(const JointVector& joint_pos, const JointVector& joint_vel,
                     const JointVector& joint_acc, const JointVector& joint_jerk,
                     double blend_tolerance_ratio_of_movej = 0.0);
        TrajectPoint(const JointVector& joint_pos, const JointVector& joint_vel,
                     const JointVector& joint_acc,
                     double blend_tolerance_ratio_of_movej = 0.0);
        TrajectPoint(const JointVector& joint_pos, const JointVector& joint_vel,
                     double blend_tolerance_ratio_of_movej = 0.0);
        TrajectPoint(const JointVector& joint_pos,
                     double blend_tolerance_ratio_of_movej = 0.0);
        TrajectPoint(const Eigen::Vector3d& cart_pos,
                     const QuaternionPoint& quaternion_point,
                     const Eigen::Vector3d& cart_vel = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d& cart_acc = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d& cart_jerk = Eigen::Vector3d::Zero());
        TrajectPoint& operator=(const TrajectPoint& arg);
        ~TrajectPoint() = default;

    public:
        /** 关节类型 */
        JointVector joint_pos;
        JointVector joint_vel;
        JointVector joint_acc;
        JointVector joint_jerk;
        double time;
        double blend_tolerance_ratio_of_movej = 0.0;
        // movej的转接比例，由于movej轨迹对于双臂加上全身的运动不是很好计算一个转接半径，
        // 所以这里使用一个比例，如果是0.2,则使用当前点前后的轨迹法法分别是百分之20的路径点作为转接点

        /** 笛卡尔 */
        Eigen::Vector3d cart_pos;
        Eigen::Vector3d cart_vel;
        Eigen::Vector3d cart_acc;
        Eigen::Vector3d cart_jerk;
        QuaternionPoint quaternion_point;

        bool is_joint_point = true;
    };

    class TrajectoryParameter
    {
    public:
        TrajectoryParameter()
        {
        };
        TrajectoryParameter(const TrajectoryParameter& arg);
        TrajectoryParameter(size_t n);
        TrajectoryParameter(const JointVector& max_vel, const JointVector& max_acc,
                            const JointVector& max_jerk);
        TrajectoryParameter(double total_time, size_t n);
        TrajectoryParameter(const JointVector& max_vel, const JointVector& max_acc,
                            const JointVector& max_jerk, double total_time);
        TrajectoryParameter(const double max_linear_vel, const double max_linear_acc,
                            const double max_linear_jerk,
                            const double max_angular_vel,
                            const double max_angular_acc,
                            const double max_angular_jerk,
                            const bool use_slerp_interpolation_for_pose);
        TrajectoryParameter(const double max_linear_vel, const double max_linear_acc,
                            const double max_linear_jerk,
                            const double max_angular_vel,
                            const double max_angular_acc,
                            const double max_angular_jerk, const double total_time,
                            const bool use_slerp_interpolation_for_pose);

        TrajectoryParameter(const bool use_slerp_interpolation_for_pose, const double total_time);

        TrajectoryParameter(const double max_linear_vel, const double max_linear_acc,
                            const double max_linear_jerk,
                            const double max_angular_vel,
                            const double max_angular_acc,
                            const double max_angular_jerk,
                            const Eigen::Vector3d& circular_rotation_axis,
                            const Eigen::Vector3d& center_of_circle,
                            const double circular_angle,
                            const bool use_slerp_interpolation_for_pose);
        TrajectoryParameter(const double max_linear_vel, const double max_linear_acc,
                            const double max_linear_jerk,
                            const double max_angular_vel,
                            const double max_angular_acc,
                            const double max_angular_jerk,
                            const Eigen::Vector3d& circular_rotation_axis,
                            const Eigen::Vector3d& center_of_circle,
                            const double circular_angle, const double total_time,
                            const bool use_slerp_interpolation_for_pose);
        TrajectoryParameter(const Eigen::Vector3d& circular_rotation_axis,
                            const Eigen::Vector3d& center_of_circle,
                            const double circular_angle, double total_time,
                            const bool use_slerp_interpolation_for_pose);
        void setCircleAngle(const double theta) { circular_angle = theta; };
        double getCircleAngle() { return circular_angle; };
        TrajectoryParameter& operator=(const TrajectoryParameter& arg);
        ~TrajectoryParameter() = default;
        // 关节参数
        JointVector max_joint_velocity;
        JointVector max_joint_acceleration;
        JointVector max_joint_jerk;
        // 笛卡尔参数
        double max_linear_vel;
        double max_linear_acc;
        double max_linear_jerk;
        double max_angular_vel;
        double max_angular_acc;
        double max_angular_jerk;
        // 圆形的参数：
        Eigen::Vector3d circular_rotation_axis;
        Eigen::Vector3d center_of_circle;
        double circular_angle = -1.0;
        bool no_need_to_calculate_circle_parameters_with_three_point = false;
        bool use_slerp_interpolation_for_pose =
            true; // 针对圆形，如果是false姿态就选择绕着圆弧的旋转轴
        // 时间模式选择
        double total_time = 3.0;
        bool time_mode = false;

    private:
        void defaultParametersOfJoint(size_t n);
        void defaultParametersOfCartesian();
    };

    class TrajectoryInitParameters
    {
    public:
        TrajectoryInitParameters()
        {
        };
        TrajectoryInitParameters(const TrajectoryInitParameters& arg);
        TrajectoryInitParameters(size_t n);
        TrajectoryInitParameters(const TrajectPoint& start_point,
                                 const TrajectPoint& end_point,
                                 const TrajectoryParameter& joint_para,
                                 double period = 0.001);
        TrajectoryInitParameters(const TrajectPoint& start_point,
                                 const TrajectPoint& middle_point,
                                 const TrajectPoint& end_point,
                                 const TrajectoryParameter& para,
                                 double period = 0.001);
        TrajectoryInitParameters(
            const std::vector<TrajectPoint>& multiple_points,
            const std::vector<TrajectoryParameter>& multiple_parameters,
            double period = 0.001);
        TrajectoryInitParameters(const std::vector<TrajectPoint>& multiple_points,
                                 TrajectoryParameter& common_parameters,
                                 double period = 0.001);

        ~TrajectoryInitParameters() = default;
        // 单段规划点
        TrajectPoint start_point;
        TrajectPoint end_point;
        TrajectPoint middle_point;
        bool has_middle_point = false;

        // 多点规划点
        std::vector<TrajectPoint>
        multiple_points; // 输入的点是从起点到终点共n+1个点，规划n段轨迹

        // 参数
        TrajectoryParameter parameter; // 单段参数，或者多点的公共参数
        std::vector<TrajectoryParameter> multiple_parameters; // 输入n段轨迹的参数
        bool use_common_parameters =
            true; // true，则统一用parameter,时间在多个关节规划结束后再计算，false则使用用户分别定义的不同速度

        double period = 0.001;
    };

    class BlendMessage
    {
    public:
        bool start_point_setting_blend_point = false;
        double blend_start_time;
        bool end_point_setting_blend_point = false;
        double blend_end_time;
        bool blend_point_met = false;
        bool is_blend_curve = false;

        BlendMessage()
        {
        };
        ~BlendMessage() = default;
    };
} // namespace planning
