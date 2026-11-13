#pragma once
#include "motion_planning/path_planner/planner.h"

namespace planning
{
    class BezierCurve : public Planner
    {
    public:
        BezierCurve()
        {
        };
        bool init(const TrajectoryInitParameters& traj_para) override final;

        ~BezierCurve() = default;

    protected:
        TrajectPoint calculatePointAtTInCurve(double t) override final;

    private:
        double totaltime_of_bezier;
        TrajectPoint start_point;
        TrajectPoint middle_point;
        TrajectPoint end_point;
        // 使用五阶贝塞尔曲线，后续如果要使用其他阶的再增加
        // 关节空间相关计算
        JointVector control_point_of_fifth_order_bezier_curve_in_joint_space[6];
        JointVector vel_control_point_of_fifth_order_bezier_curve_in_joint_space[5];
        JointVector acc_control_point_of_fifth_order_bezier_curve_in_joint_space[4];
        JointVector jerk_control_point_of_fifth_order_bezier_curve_in_joint_space[3];
        void calculateControlPointOf5thBezierCurveInJointSpace(
            const JointVector& p0, const JointVector& v0, const JointVector& a0,
            const JointVector& p5, const JointVector& v5, const JointVector& a5);
        void calculateVelControlPointOf5thBezierCurveInJointSpace();
        void calculateAccControlPointOf5thBezierCurveInJointSpace();
        void calculateJerkControlPointOf5thBezierCurveInJointSpace();
        JointVector calculatePosAtTof5thBezierCurveInJointSpace(double t);
        JointVector calculateVelAtTof5thBezierCurveInJointSpace(double t);
        JointVector calculateAccAtTof5thBezierCurveInJointSpace(double t);
        JointVector calculateJerkAtTof5thBezierCurveInJointSpace(double t);

        TrajectPoint calculatePointAtTof5thBezierCurveInJointSpace(double t);
        void convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point);
        // 限制贝塞尔曲线的位置：
        // 计算边界条件的归一化参数
        bool calcualeNormBoundaryConditionsWithMiddlePointInJointSpaceOf5thBezier(
            double* norm_v0, double* norm_a0, double* r0, double* norm_v5,
            double* norm_a5, double* r1);
        // 1.有中间点的时候，限制贝塞尔控制点在中间点的凸包内的最大时间
        // a,满足起点条件的最大时间
        bool calculateMaxTimeOf5thBezierCurveStartConstraintInNormSpace(
            double v0, double a0, double r0, double* maxtime);
        // b.满足终点条件的最大时间
        bool calculateMaxTimeOf5thBezierCurveEndConstraintInNormSpace(
            double v5, double a5, double r1, double* maxtime);
        bool calculateMaxTimeOf5thBezierCurveWithPosConstraintInJointSpace(
            double* max_time, bool* vel_of_both_sides_is_not_zero);
        // 2.没有中间点，是同一段轨迹的时候，为了不出现后退的情况限制最大时间

        // todo:笛卡尔空间计算
    };
} // namespace planning
