#pragma once

#include <Eigen/Core>

namespace ocs2::mpc_core
{
    /// A target pose (x y z qx qy qz qw) the reference manager can use: seven finite numbers and a quaternion that is
    /// not (close to) zero. Whatever publishes a target, a NaN or a zero quaternion must never reach the solver.
    inline bool targetPoseUsable(const Eigen::VectorXd& goal7)
    {
        if (goal7.size() < 7 || !goal7.head(7).allFinite())
        {
            return false;
        }
        return goal7.segment(3, 4).norm() > 1e-6;
    }
}  // namespace ocs2::mpc_core
