#pragma once
#include "motion_planning/path_planner/planner.h"
#include <memory>

namespace planning
{
    // 前向声明，避免暴露内部实现细节
    class DoublesWithZeroBoundaryOfVelAndAcc;

    class moveJ : public Planner
    {
    public:
        moveJ();
        ~moveJ();
        bool init(const TrajectoryInitParameters& traj_para) override final;
        TrajectPoint calculatePointAtTInCurve(double t) override final;
        JointVector getStartPoint() { return start_pos; };

    private:
        void convertVelAccJerkUsingTimeScalingfactor(TrajectPoint& point);
        JointVector start_pos;
        JointVector delta_pos;

        void setBlendMessage(const TrajectPoint& start_point,
                             const TrajectPoint& end_point);

        // 使用指针隐藏实现细节
        std::unique_ptr<DoublesWithZeroBoundaryOfVelAndAcc> velocity_curve;
    };
} // namespace planning
