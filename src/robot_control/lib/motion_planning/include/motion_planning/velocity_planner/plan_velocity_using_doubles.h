#pragma once
#include "motion_planning/velocity_planner/doubles.h"
#include "motion_planning/common/normalization_point.h"

namespace planning
{
    class PlanVelocityUsingDoubles : public Doubles
    {
    public:
        PlanVelocityUsingDoubles() = default;
        ~PlanVelocityUsingDoubles() = default;

        // 初始化参数
        bool init(double p0, double v0, double v1, double max_acc, double max_jerk,
                  double period = 0.001, double totaltime = 0.0);
        double getAccDistance() { return distance_acc; };

    protected:
        void setZeroPara();
        double min_val = 1.0e-9;
        // 如果是加速，需要初始化参数,仅考虑初始加速度为0的情况
        bool calculateAccelerationParameter(double v0, double v1, double max_acc,
                                            double max_jerk, double* taa,
                                            double* tua);
    };
} // namespace planning
