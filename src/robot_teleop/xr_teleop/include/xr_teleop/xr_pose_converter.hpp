#pragma once

#include <optional>

#include <Eigen/Geometry>

#include "xr_teleop/xr_types.hpp"

namespace xr_teleop
{
    /// OpenXR -> robot basis (T_ROBOT_OPENXR * M * T_OPENXR_ROBOT), port of XroboToolkitWrapper.
    /// Non-finite values or a zero quaternion keep the previous pose (reference pose before the first).
    class XrPoseConverter
    {
    public:
        struct Poses
        {
            Eigen::Isometry3d head;
            Eigen::Isometry3d left;
            Eigen::Isometry3d right;
            /// This frame carried a valid sample for the device (the pose above is that sample). False: the pose is the
            /// last good one, or the reference fallback before the first, and must not be published as a measurement.
            bool head_updated = false;
            bool left_updated = false;
            bool right_updated = false;
        };

        XrPoseConverter();

        /// Convert one frame; missing devices keep their last pose.
        const Poses& update(const XrFrame& frame);
        const Poses& current() const { return poses_; }

        /// Single conversion; std::nullopt if the sample is invalid.
        static std::optional<Eigen::Isometry3d> openXrToRobot(const Pose7& pose);
        /// Robot-frame pose as x, y, z, qx, qy, qz, qw.
        static Pose7 toPose7(const Eigen::Isometry3d& pose);

    private:
        static bool updateOne(const Pose7& sample, Eigen::Isometry3d& last);

        Poses poses_;
    };
} // namespace xr_teleop
