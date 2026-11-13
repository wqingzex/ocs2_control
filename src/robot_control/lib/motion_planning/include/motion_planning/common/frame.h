#pragma once
#include "quaternion.h"
#include "rotation_matrix.h"
#include <Eigen/Dense>

namespace planning
{
    class Frame
    {
    public:
        Frame();
        ~Frame() = default;

        Frame(const RotationMatrix& rot, const Eigen::Vector3d& p);

        Frame(const Quaternion& q, const Eigen::Vector3d& p);

        Frame operator*(const Frame& arg) const;

        Frame Inverse() const;

        static Frame Zero();

        static Frame mdh(double theta, double d, double a, double alpha);

        static Frame sdh(double theta, double d, double a, double alpha);

    public:
        RotationMatrix rot;
        Eigen::Vector3d p;
        Quaternion q; // 只是作为和旋转矩阵相互转换的时候用
    };
} // namespace planning
