#pragma once

#include <cmath>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace target_generator
{
    /// Limits of a teleop target before it is sent to the MPC.
    struct TargetGuardLimits
    {
        bool enabled = true;
        double max_distance = 0.3;  ///< m, target to the current end effector
        double max_step = 0.05;     ///< m, target to the previous accepted target
    };

    enum class TargetVerdict
    {
        Accept,
        NotFinite,        ///< NaN / inf in the position or orientation, or a zero quaternion
        TooFarFromArm,    ///< further than max_distance from where the end effector is now
        StepTooLarge,     ///< further than max_step from the previous accepted target
    };

    inline const char* toString(TargetVerdict verdict)
    {
        switch (verdict)
        {
        case TargetVerdict::Accept: return "accepted";
        case TargetVerdict::NotFinite: return "not finite";
        case TargetVerdict::TooFarFromArm: return "too far from the end effector";
        case TargetVerdict::StepTooLarge: return "step from the previous target too large";
        }
        return "unknown";
    }

    /// Pure check, no state: the caller remembers the last accepted target.
    inline TargetVerdict checkTarget(const TargetGuardLimits& limits,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation,
                                     const Eigen::Vector3d& current_position,
                                     const std::optional<Eigen::Vector3d>& previous_position)
    {
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-6)
        {
            return TargetVerdict::NotFinite;  // never let this through, even with the guard switched off
        }
        if (!limits.enabled)
        {
            return TargetVerdict::Accept;
        }
        if ((position - current_position).norm() > limits.max_distance)
        {
            return TargetVerdict::TooFarFromArm;
        }
        if (previous_position && (position - *previous_position).norm() > limits.max_step)
        {
            return TargetVerdict::StepTooLarge;
        }
        return TargetVerdict::Accept;
    }
} // namespace target_generator
