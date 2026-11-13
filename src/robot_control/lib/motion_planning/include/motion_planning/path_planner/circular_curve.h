#pragma once
#include "motion_planning/path_planner/planner.h"
#include "motion_planning/common/quaternion.h"
#include <eigen3/Eigen/Dense>
#include <memory>

namespace planning
{
    // 前向声明，避免暴露内部实现细节
    class DoublesWithZeroBoundaryOfVelAndAcc;

    class CircularCurver : public Planner
    {
    public:
        CircularCurver();
        ~CircularCurver();

        bool init(const TrajectoryInitParameters& traj_para) override final;

        TrajectPoint calculatePointAtTInCurve(double t) override final;

    private:
        // 使用指针隐藏实现细节
        std::unique_ptr<DoublesWithZeroBoundaryOfVelAndAcc> velocity_curve;
        bool calculatCircularParameterWithThreePoint(
            const TrajectoryInitParameters& traj_para);
        bool
        setCircularParameterWithInputPara(const TrajectoryInitParameters& traj_para);
        void convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point);
        bool use_slerp_interpolation_for_pose =
            true; // true就是slerp插值，false就是绕着圆弧的轴旋转
        // 圆形的参数：
        Eigen::Vector3d circular_rotation_axis;
        Eigen::Vector3d center_of_circle;
        double circular_angle;
        double radius_of_circle;
        Eigen::Matrix3d circular_plane_system;

        QuaternionPoint q0; // 四元数起点
        Quaternion q1; // 四元数终点，如果是slerp插值的模式需要使用

        const double min_val = 1.0e-6;
    };
} // namespace planning
