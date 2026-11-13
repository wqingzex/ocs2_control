#pragma once
#include "motion_planning/path_planner/planner.h"
#include "motion_planning/common/quaternion.h"
#include <eigen3/Eigen/Dense>
#include <memory>

namespace planning
{
    // 前向声明，避免暴露内部实现细节
    class DoublesWithZeroBoundaryOfVelAndAcc;

    class moveL : public Planner
    {
    public:
        moveL();
        ~moveL();
        bool init(const TrajectoryInitParameters& traj_para) override final;
        TrajectPoint calculatePointAtTInCurve(double t) override final;
        Eigen::Vector3d getStartPoint() { return start_pos; };

    private:
        // 使用指针隐藏实现细节
        std::unique_ptr<DoublesWithZeroBoundaryOfVelAndAcc> velocity_curve;
        void convertVelAccUsingTimeScalingfactor(TrajectPoint& point);
        // 直线运动的起始位置
        Eigen::Vector3d start_pos;
        Eigen::Vector3d delta_pos;
        QuaternionPoint q0;
        Quaternion q1;
        double linear_distance;
        const double min_val = 1.0e-6;
    };
} // namespace planning
