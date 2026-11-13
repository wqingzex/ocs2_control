#pragma once
#include <Eigen/Dense>

namespace planning
{
    class RotationMatrix
    {
    public:
        RotationMatrix() = default;
        RotationMatrix(const Eigen::Matrix3d& arg);

        RotationMatrix Inverse() const;
        RotationMatrix Transpose() const;

        double operator()(size_t m, size_t n) const;
        double& operator()(size_t m, size_t n);
        friend RotationMatrix operator*(const RotationMatrix& lhs,
                                        const RotationMatrix& rhs);
        Eigen::Vector3d operator*(const Eigen::Vector3d& arg);

        static RotationMatrix rotX(double angle);
        static RotationMatrix rotY(double angle);
        static RotationMatrix rotZ(double angle);
        static RotationMatrix rodrigues(const Eigen::Vector3d& axis, double angle);
        static RotationMatrix toSkew(const Eigen::Vector3d& arg);
        ~RotationMatrix() = default;

    public:
        Eigen::Matrix3d data = Eigen::Matrix3d::Identity();
    };
} // namespace planning
