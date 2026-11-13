#include "motion_planning/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.h"
#include <iostream>
#include <math.h>

namespace planning
{
    bool DoublesWithZeroBoundaryOfVelAndAcc::init(double _start_pos,
                                                  double _end_pos, double _max_vel,
                                                  double _max_acc, double _max_jerk,
                                                  double _period)
    {
        const double min_value = 1.0e-6;
        start_pos = _start_pos;
        end_pos = _end_pos;
        start_vel = 0.0;
        end_vel = 0.0;
        max_vel = fabs(_max_vel);
        max_acc = fabs(_max_acc);
        max_jerk = fabs(_max_jerk);
        period = fabs(_period);
        if (max_jerk < min_value || max_acc < min_value || max_vel < min_value)
        {
            return false;
        }

        double delta_pos = end_pos - start_pos;
        if (delta_pos < 0)
        {
            delta_pos = fabs(delta_pos);
            direction = -1.0;
            start_pos *= -1.0;
            end_pos *= -1.0;
        }
        if (delta_pos < min_value)
        {
            tj1 = 0.0;
            ta = 0.0;
            tj2 = 0.0;
            td = 0.0;
            tv = 0.0;
            totaltime = 0.0;
            return true;
        }

        if (max_vel * max_jerk < pow(max_acc, 2.0))
        {
            tj1 = sqrt(max_vel / max_jerk);
            ta = 2.0 * tj1;
        }
        else
        {
            tj1 = max_acc / max_jerk;
            ta = tj1 + max_vel / max_acc;
        }
        tv = delta_pos / max_vel - ta;
        if (tv > 0)
        {
            tj2 = tj1;
            td = ta;
            totaltime = ta + tv + td;
        }
        else
        {
            if (delta_pos < 2.0 * pow(max_acc, 3.0) / pow(max_jerk, 2.0))
            {
                tj1 = cbrt(0.5 * delta_pos / max_jerk);
                ta = 2.0 * tj1;
                tv = 0.0;
                tj2 = tj1;
                td = ta;
                totaltime = ta + tv + td;
            }
            else
            {
                tj1 = max_acc / max_jerk;
                ta = 0.5 * tj1 + sqrt(pow(0.5 * tj1, 2.0) + delta_pos / max_acc);
                tv = 0.0;
                tj2 = tj1;
                td = ta;
                totaltime = ta + tv + td;
            }
        }
        return true;
    };

    double DoublesWithZeroBoundaryOfVelAndAcc::bisectionMethodCalculateTime(
        double pos, double a, double b)
    {
        double minval = 1.0e-6;
        double res;
        NormalizationPoint pa = calculatePointAtT(a);
        NormalizationPoint pb = calculatePointAtT(b);
        double fa = pa.pos - pos;
        double fb = pb.pos - pos;
        if (fabs(fa) < minval)
        {
            res = a;
            return res;
        }
        if (fabs(fb) < minval)
        {
            res = b;
            return res;
        }

        double delta_pos = fabs(fa - fb);
        if (delta_pos < minval)
        {
            res = 0.5 * (a + b);
            return res;
        }
        NormalizationPoint pc;
        double c, fc;
        while (delta_pos > minval)
        {
            c = 0.5 * (a + b);
            pc = calculatePointAtT(c);
            fc = pc.pos - pos;
            if (fabs(fc) < minval)
            {
                res = c;
                break;
            }
            if (fa * fc < 0)
            {
                b = c;
                fb = fc;
            }
            else
            {
                a = c;
                fa = fc;
            }
            delta_pos = fabs(fa - fb);
            res = 0.5 * (a + b);
        }

        return res;
    };

    double DoublesWithZeroBoundaryOfVelAndAcc::calculateTimeOfPos(double p)
    {
        calculateDistanceOf7thPhase();
        double res;
        double minval = 1.0e-6;
        double abspos = (p - start_pos) * direction;
        if (abspos < minval)
        {
            res = 0.0;
        }
        else if (abspos < distance_aa)
        {
            res = bisectionMethodCalculateTime(p, 0.0, tj1);
        }
        else if (abspos < distance_aa + distance_ua)
        {
            res = bisectionMethodCalculateTime(p, tj1, ta - tj1);
        }
        else if (abspos < distance_acc)
        {
            res = bisectionMethodCalculateTime(p, ta - tj1, ta);
        }
        else if (abspos < distance_acc + distance_v)
        {
            double vlim = (ta - tj1) * max_jerk * tj1;
            res = ta + (abspos - distance_acc) / vlim;
        }
        else if (abspos < distance_acc + distance_v + distance_ad)
        {
            res = bisectionMethodCalculateTime(p, ta + tv, ta + tv + tj2);
        }
        else if (abspos < distance_acc + distance_v + distance_ad + distance_ud)
        {
            res = bisectionMethodCalculateTime(p, ta + tv + tj2, totaltime - tj2);
        }
        else if (abspos < distance_acc + distance_v + distance_dec)
        {
            res = bisectionMethodCalculateTime(p, totaltime - tj2, totaltime);
        }
        else
        {
            res = totaltime;
        }
        return res;
    };
} // namespace planning
