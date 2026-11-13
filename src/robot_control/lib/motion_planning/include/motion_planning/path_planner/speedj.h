#pragma once
#include "motion_planning/path_planner/planner.h"
#include <memory>
namespace planning {
    class PlanVelocityUsingDoubles;

    class SpeedJ : public Planner {

    public:
        SpeedJ();
        ~SpeedJ();
        bool init(const TrajectoryInitParameters &traj_para) override final;
        TrajectPoint calculatePointAtTInCurve(double t) override final;
        JointVector getStartPoint() { return start_pos; };
        double calculateDecDistanceOfV(double delta_v,double max_acc,double max_jerk);

    private:
        void convertVelAccJerkUsingTimeScalingfactor(TrajectPoint &point);
        JointVector start_pos;
        JointVector start_vel;
        JointVector delta_vel;
        // 使用指针隐藏实现细节
        std::unique_ptr<PlanVelocityUsingDoubles> velocity_curve;
    };

} // namespace planning
