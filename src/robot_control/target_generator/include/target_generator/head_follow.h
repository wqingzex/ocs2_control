#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace target_generator
{
    /// Where the headset and the robot head were when the head started to follow.
    struct HeadFollowBase
    {
        Eigen::Vector3d xr_position;
        Eigen::Quaterniond xr_orientation;
        Eigen::Vector3d robot_position;
        Eigen::Quaterniond robot_orientation;
    };

    struct HeadTarget
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    /// Head target: what the headset moved and turned since the base, put on top of the robot head at the base.
    /// The headset pose arrives already in robot axes (xr_teleop converts it) and head_current_pose is in the base
    /// frame of the model, so the movement is added as it is.
    inline HeadTarget headTarget(const HeadFollowBase& base,
                                 const Eigen::Vector3d& xr_position,
                                 const Eigen::Quaterniond& xr_orientation)
    {
        HeadTarget target;
        target.position = base.robot_position + (xr_position - base.xr_position);
        target.orientation =
            (xr_orientation * base.xr_orientation.inverse() * base.robot_orientation).normalized();
        return target;
    }

    /// A pose the head may use: finite numbers and a quaternion that is not (close to) zero.
    inline bool headPoseUsable(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        return position.allFinite() && orientation.coeffs().allFinite() && orientation.norm() > 1e-6;
    }
} // namespace target_generator
