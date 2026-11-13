#pragma once
#include "motion_planning/path_planner/planner.h"
#include "motion_planning/path_planner/movej.h"
#include <memory>
#include <vector>

namespace planning
{
    class SmoothCurveOfMultiJointsUsingBlending : public Planner
    {
    public:
        SmoothCurveOfMultiJointsUsingBlending()
        {
        };
        bool init(const TrajectoryInitParameters& traj_para) override final;
        TrajectPoint calculatePointAtTInRealTime(double t) override final;
        TrajectPoint calculatePointAtTInCurve(double t) override final;
        TrajectPoint run() override final;
        ~SmoothCurveOfMultiJointsUsingBlending() = default;

    private:
        std::vector<std::shared_ptr<Planner>>
        planners; // 规划好的多个planner（对于一系列点来说就是movej、bezier、movej...movej、bezier、movej)
        std::vector<double> total_time_of_planners;
        std::shared_ptr<Planner> current_planner; // 正在运行的planner
        size_t current_planner_index = 0;
        std::vector<std::shared_ptr<moveJ>> init_movej_planners; // 初始规划的movej

        size_t nr_of_joints;
        size_t nr_of_init_movej;
        size_t nr_of_planners;
        void getIthStartAndEndPoints(TrajectPoint& start_point,
                                     TrajectPoint& end_point,
                                     const TrajectoryInitParameters& traj_para,
                                     size_t i);
        bool addSingleMoveJ(const TrajectPoint& current_start_point,
                            const TrajectPoint& current_end_point,
                            const TrajectoryParameter& para, double period);
        bool addMoveJ(const TrajectoryInitParameters& traj_para);
        bool addAllPlanners();
        void setVectorTotalTimes();
    };
} // namespace planning
