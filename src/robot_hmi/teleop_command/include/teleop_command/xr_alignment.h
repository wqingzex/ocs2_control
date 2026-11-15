#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace teleop_command
{
    /// Heading (yaw about +Z) of the frame's local +X axis projected on the XY plane.
    /// Returns false if that axis is (almost) vertical and the heading is undefined.
    inline bool headingYaw(const Eigen::Quaterniond& orientation, double& yaw)
    {
        const Eigen::Vector3d forward = orientation * Eigen::Vector3d::UnitX();
        const double horizontal = std::hypot(forward.x(), forward.y());
        if (horizontal < 1e-3)
        {
            return false;
        }
        yaw = std::atan2(forward.y(), forward.x());
        return true;
    }

    /// Frame for XR hand displacements: the controller orientation at follow start (legacy), or with
    /// align_head_yaw a yaw-only frame facing the operator's head, so "forward" is robot +X whatever
    /// the room origin or controller tilt (falls back to the controller heading if the head is vertical).
    inline Eigen::Quaterniond motionFrame(bool align_head_yaw,
                                          const Eigen::Quaterniond& head_orientation,
                                          const Eigen::Quaterniond& controller_base_orientation)
    {
        if (!align_head_yaw)
        {
            return controller_base_orientation;
        }
        double yaw = 0.0;
        if (!headingYaw(head_orientation, yaw) && !headingYaw(controller_base_orientation, yaw))
        {
            yaw = 0.0;
        }
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }
} // namespace teleop_command
