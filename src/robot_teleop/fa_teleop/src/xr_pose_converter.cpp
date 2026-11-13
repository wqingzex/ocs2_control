#include "fa_teleop/xr_pose_converter.hpp"

#include <cmath>

namespace fa_teleop
{
    namespace
    {
        Eigen::Matrix4d robotFromOpenXr()
        {
            Eigen::Matrix4d m;
            m << 0, 0, -1, 0,
                -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;
            return m;
        }

        Eigen::Matrix4d openXrFromRobot()
        {
            Eigen::Matrix4d m;
            m << 0, -1, 0, 0,
                0, 0, 1, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
            return m;
        }

        Eigen::Isometry3d translation(double x, double y, double z)
        {
            Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
            t.translation() << x, y, z;
            return t;
        }
    } // namespace

    XrPoseConverter::XrPoseConverter()
    {
        // Reference fallbacks (tv_wrapper CONST_*_POSE), used until the first valid sample.
        poses_.head = translation(0.0, 1.5, -0.2);
        poses_.left = translation(-0.15, 1.13, -0.3);
        poses_.right = translation(0.15, 1.13, -0.3);
    }

    std::optional<Eigen::Isometry3d> XrPoseConverter::openXrToRobot(const Pose7& pose)
    {
        for (double v : pose)
        {
            if (!std::isfinite(v))
            {
                return std::nullopt;
            }
        }
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);  // w, x, y, z
        const double norm = q.norm();
        if (!(norm > 0.0))
        {
            return std::nullopt;
        }
        q.coeffs() /= norm;

        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.topLeftCorner<3, 3>() = q.toRotationMatrix();
        m.topRightCorner<3, 1>() << pose[0], pose[1], pose[2];

        const Eigen::Matrix4d robot = robotFromOpenXr() * m * openXrFromRobot();
        const double det = robot.determinant();
        if (!std::isfinite(det) || std::abs(det) <= 1e-6)
        {
            return std::nullopt;
        }
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.matrix() = robot;
        return out;
    }

    Pose7 XrPoseConverter::toPose7(const Eigen::Isometry3d& pose)
    {
        const Eigen::Quaterniond q(pose.rotation());
        const Eigen::Vector3d p = pose.translation();
        return {p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()};
    }

    void XrPoseConverter::updateOne(const Pose7& sample, Eigen::Isometry3d& last)
    {
        if (auto converted = openXrToRobot(sample))
        {
            last = *converted;
        }
    }

    const XrPoseConverter::Poses& XrPoseConverter::update(const XrFrame& frame)
    {
        if (frame.has_head)
        {
            updateOne(frame.head, poses_.head);
        }
        if (frame.has_left)
        {
            updateOne(frame.left.pose, poses_.left);
        }
        if (frame.has_right)
        {
            updateOne(frame.right.pose, poses_.right);
        }
        return poses_;
    }
} // namespace fa_teleop
