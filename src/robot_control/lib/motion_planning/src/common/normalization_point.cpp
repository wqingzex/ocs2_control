#include "motion_planning/common/normalization_point.h"

namespace planning
{
    NormalizationPoint::NormalizationPoint(double _time, double _pos, double _vel,
                                           double _acc, double _jerk)
        : time(_time), pos(_pos), vel(_vel), acc(_acc), jerk(_jerk)
    {
    };
} // namespace planning
