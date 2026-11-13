#include "motion_planning/common/frame.h"

namespace planning
{
    Frame::Frame()
    {
        rot.data = Eigen::Matrix3d::Identity();
        p = Eigen::Vector3d::Zero();
    };

    Frame::Frame(const RotationMatrix& _rot, const Eigen::Vector3d& _p)
    {
        rot.data = _rot.data;
        p = _p;
        q = Quaternion::convertRotationMatrixToQuaternion(rot.data);
    };

    Frame::Frame(const Quaternion& _q, const Eigen::Vector3d& _p)
    {
        q = _q;
        p = _p;
        rot.data = q.toRotationMatrix();
    };

    Frame Frame::operator*(const Frame& arg) const
    {
        Frame res;
        res.rot = rot * arg.rot;
        res.p = p + rot.data * arg.p;
        return res;
    };

    Frame Frame::Inverse() const
    {
        Frame res;
        res.rot = rot.Inverse();
        res.p = res.rot * (-p);
        return res;
    };

    Frame Frame::Zero()
    {
        Frame res;
        res.rot.data = Eigen::Matrix3d::Identity();
        res.p = Eigen::Vector3d::Zero();
        res.q.w = 1.0;
        res.q.x = 0.0;
        res.q.y = 0.0;
        res.q.z = 0.0;
        return res;
    };

    Frame Frame::mdh(double theta, double d, double a, double alpha)
    {
        Frame res;
        res.rot(0, 0) = cos(theta);
        res.rot(0, 1) = -sin(theta);
        res.rot(0, 2) = 0.0;
        res.rot(1, 0) = cos(alpha) * sin(theta);
        res.rot(1, 1) = cos(alpha) * cos(theta);
        res.rot(1, 2) = -sin(alpha);
        res.rot(2, 0) = sin(alpha) * sin(theta);
        res.rot(2, 1) = sin(alpha) * cos(theta);
        res.rot(2, 2) = cos(alpha);
        res.p(0) = a;
        res.p(1) = -d * sin(alpha);
        res.p(2) = d * cos(alpha);
        return res;
    };

    Frame Frame::sdh(double theta, double d, double a, double alpha)
    {
        Frame res;
        res.rot(0, 0) = cos(theta);
        res.rot(0, 1) = -cos(alpha) * sin(theta);
        res.rot(0, 2) = sin(alpha) * sin(theta);
        res.rot(1, 0) = sin(theta);
        res.rot(1, 1) = cos(alpha) * cos(theta);
        res.rot(1, 2) = -sin(alpha) * cos(theta);
        res.rot(2, 0) = 0.0;
        res.rot(2, 1) = sin(alpha);
        res.rot(2, 2) = cos(alpha);
        res.p(0) = a * cos(theta);
        res.p(1) = a * sin(theta);
        res.p(2) = d;
        return res;
    };
} // namespace planning
