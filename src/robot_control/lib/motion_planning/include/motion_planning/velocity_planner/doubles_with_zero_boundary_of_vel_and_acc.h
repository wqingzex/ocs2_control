#pragma once
#include "motion_planning/velocity_planner/doubles.h"
#include "motion_planning/common/normalization_point.h"

namespace planning
{
    /**
         * @brief 使用doubles规划一段初始速度和终止速度均为0的速度轨迹
           @param
         */
    class DoublesWithZeroBoundaryOfVelAndAcc : public Doubles
    {
    public:
        DoublesWithZeroBoundaryOfVelAndAcc()
        {
        };
        bool init(double start_pos = 0.0, double end_pos = 1.0, double max_vel = 0.0,
                  double max_acc = 0.0, double max_jerk = 0.0, double period = 0.001);

        double calculateTimeOfPos(double p);
        ~DoublesWithZeroBoundaryOfVelAndAcc() = default;

    private:
        double bisectionMethodCalculateTime(double pos, double a, double b);
    };
} // namespace planning
