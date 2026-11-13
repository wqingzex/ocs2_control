#include "motion_planning/common/rotation_matrix.h"

namespace planning
{
    RotationMatrix::RotationMatrix(const Eigen::Matrix3d& arg) { data = arg; };

    RotationMatrix RotationMatrix::Inverse() const
    {
        return RotationMatrix(data.transpose());
    };

    RotationMatrix RotationMatrix::Transpose() const
    {
        return RotationMatrix(data.transpose());
    };

    double RotationMatrix::operator()(size_t m, size_t n) const
    {
        return data(m, n);
    };
    double& RotationMatrix::operator()(size_t m, size_t n) { return data(m, n); };

    RotationMatrix operator*(const RotationMatrix& lhs, const RotationMatrix& rhs)
    {
        return RotationMatrix(lhs.data * rhs.data);
    };

    Eigen::Vector3d RotationMatrix::operator*(const Eigen::Vector3d& arg)
    {
        Eigen::Vector3d res = data * arg;
        return res;
    };

    RotationMatrix RotationMatrix::rotX(double angle)
    {
        RotationMatrix res;
        res.data << 1.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0, sin(angle),
            cos(angle);
        return res;
    };

    RotationMatrix RotationMatrix::rotY(double angle)
    {
        RotationMatrix res;
        res.data << cos(angle), 0.0, sin(angle), 0.0, 1.0, 0.0, -sin(angle), 0.0,
            cos(angle);
        return res;
    };

    RotationMatrix RotationMatrix::rotZ(double angle)
    {
        RotationMatrix res;
        res.data << cos(angle), -sin(angle), 0.0, sin(angle), cos(angle), 0.0, 0.0,
            0.0, 1.0;
        return res;
    };

    RotationMatrix RotationMatrix::rodrigues(const Eigen::Vector3d& axis,
                                             double angle)
    {
        RotationMatrix res;
        Eigen::Matrix3d axis_skew = toSkew(axis).data;
        res.data = Eigen::Matrix3d::Identity() + sin(angle) * axis_skew +
            (1.0 - cos(angle)) * axis_skew * axis_skew;
        return res;
    };

    RotationMatrix RotationMatrix::toSkew(const Eigen::Vector3d& arg)
    {
        RotationMatrix res;
        res.data << 0.0, -arg(2), arg(1), arg(2), 0.0, -arg(0), -arg(1), arg(0), 0.0;
        return res;
    };
} // namespace planning
