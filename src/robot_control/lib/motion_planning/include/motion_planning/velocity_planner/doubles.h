#pragma once
#include "motion_planning/common/normalization_point.h"

namespace planning
{
    class Doubles
    {
    public:
        Doubles() = default;
        ~Doubles() = default;
        NormalizationPoint calculatePointAtT(double t);
        double getTotalTime() { return totaltime; };

    protected:
        double start_pos;
        double end_pos;
        double start_vel = 0.0;
        double end_vel = 0.0;
        double direction = 1.0;
        double max_vel;
        double max_acc;
        double max_jerk;
        double period;
        double tj1;
        double tj2;
        double ta;
        double td;
        double tv;
        double totaltime;
        double distance_aa;
        double distance_ua;
        double distance_da;
        double distance_acc;
        double distance_v;
        double distance_ad;
        double distance_ud;
        double distance_dd;
        double distance_dec;
        void calculateDistanceOf7thPhase();
    };
} // namespace planning
