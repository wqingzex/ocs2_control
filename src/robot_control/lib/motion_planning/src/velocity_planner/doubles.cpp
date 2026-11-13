#include "motion_planning/velocity_planner/doubles.h"
#include <math.h>

namespace planning
{
    NormalizationPoint Doubles::calculatePointAtT(double t)
    {
        NormalizationPoint point;
        double alima = max_jerk * tj1;
        double alimd = -max_jerk * tj2;
        double vlim = start_vel + (ta - tj1) * alima;
        if (t >= 0 && t <= tj1)
        {
            point.jerk = max_jerk;
            point.acc = max_jerk * t;
            point.vel = start_vel + 0.5 * max_jerk * t * t;
            point.pos = start_pos + start_vel * t + max_jerk * t * t * t / 6.0;
        }
        else if (t > tj1 && t <= ta - tj1)
        {
            point.jerk = 0.0;
            point.acc = alima;
            point.vel = start_vel + alima * (t - 0.5 * tj1);
            point.pos = start_pos + start_vel * t +
                alima * (3.0 * t * t - 3.0 * tj1 * t + tj1 * tj1) / 6.0;
        }
        else if (t > ta - tj1 && t <= ta)
        {
            point.jerk = -max_jerk;
            point.acc = max_jerk * (ta - t);
            point.vel = vlim - 0.5 * max_jerk * pow(ta - t, 2.0);
            point.pos = start_pos + 0.5 * (vlim + start_vel) * ta - vlim * (ta - t) +
                max_jerk * pow(ta - t, 3.0) / 6.0;
        }
        else if (t > ta && t <= ta + tv)
        {
            point.jerk = 0.0;
            point.acc = 0.0;
            point.vel = vlim;
            point.pos = start_pos + 0.5 * (vlim + start_vel) * ta + vlim * (t - ta);
        }
        else if (t > ta + tv && t <= ta + tv + tj2)
        {
            point.jerk = -max_jerk;
            point.acc = -max_jerk * (t - totaltime + td);
            point.vel = vlim - 0.5 * max_jerk * pow(t - totaltime + td, 2.0);
            point.pos = end_pos - 0.5 * (vlim + end_vel) * td +
                vlim * (t - totaltime + td) -
                max_jerk * pow(t - totaltime + td, 3.0) / 6.0;
        }
        else if (t > ta + tv + tj2 && t <= totaltime - tj2)
        {
            point.jerk = 0.0;
            point.acc = alimd;
            point.vel = vlim + alimd * (t - totaltime + td - 0.5 * tj2);
            point.pos = end_pos - 0.5 * (vlim + end_vel) * td +
                vlim * (t - totaltime + td) +
                alimd *
                (3.0 * pow(t - totaltime + td, 2.0) -
                    3.0 * tj2 * (t - totaltime + td) + tj2 * tj2) /
                6.0;
        }
        else if (t > totaltime - tj2 && t <= totaltime)
        {
            point.jerk = max_jerk;
            point.acc = -max_jerk * (totaltime - t);
            point.vel = end_vel + 0.5 * max_jerk * pow(totaltime - t, 2.0);
            point.pos = end_pos - end_vel * (totaltime - t) -
                max_jerk * pow(totaltime - t, 3.0) / 6.0;
        }
        else if (t >= totaltime)
        {
            point.jerk = 0.0;
            point.acc = 0.0;
            point.vel = end_vel;
            point.pos = end_pos;
        }
        point.time = t;
        return point * direction;
    };

    void Doubles::calculateDistanceOf7thPhase()
    {
        // phase1:
        double alima = max_jerk * tj1;
        double v1 = 0.5 * max_jerk * tj1 * tj1;
        distance_aa = max_jerk * pow(tj1, 3.0) / 6.0;
        // phase2:
        double v2 = v1 + alima * (ta - 2.0 * tj1);
        distance_ua = v1 * (ta - 2.0 * tj1) + 0.5 * alima * pow(ta - 2.0 * tj1, 2.0);
        // phase3:
        double vlim = alima * (ta - tj1);
        distance_da =
            v2 * tj1 + 0.5 * alima * tj1 * tj1 - max_jerk * pow(tj1, 3.0) / 6.0;
        distance_acc = distance_aa + distance_ua + distance_da;
        // phase4:
        distance_v = vlim * tv;
        // phase5:
        double alimd = -max_jerk * tj2;
        double v5 = vlim - 0.5 * max_jerk * tj2 * tj2;
        distance_ad = vlim * tj2 - max_jerk * pow(tj2, 3.0) / 6.0;
        // phase6:
        double v6 = v5 + alimd * (td - 2.0 * tj2);
        distance_ud = v5 * (td - 2.0 * tj2) + 0.5 * alimd * pow(td - 2.0 * tj2, 2.0);
        // phase7:
        distance_dd =
            v6 * tj2 + 0.5 * alimd * tj2 * tj2 + max_jerk * pow(tj2, 3.0) / 6.0;

        distance_dec = distance_ad + distance_ud + distance_dd;
    };
} // namespace planning
