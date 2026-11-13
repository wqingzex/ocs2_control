#pragma once
#include <eigen3/Eigen/Dense>

namespace planning
{
    class Quaternion
    {
    public:
        Quaternion() = default;
        Quaternion(double w, double x, double y, double z);
        Quaternion(const Quaternion& arg);
        ~Quaternion() = default;

        // slerp 插值
        void checkTargetPointWithShortCurve(Quaternion& target);
        Quaternion slerp(const Quaternion& target, double h);
        // 绕固定轴旋转一个角度
        Quaternion rotateAroundAxisByTheta(const Eigen::Vector3d axis,
                                           const double theta, bool is_body_system);
        // 运算
        Quaternion& operator=(const Quaternion& arg);
        Quaternion operator+(const Quaternion& arg) const;
        Quaternion operator-(const Quaternion& arg) const;
        Quaternion operator*(const Quaternion& arg) const;
        Quaternion operator*(const double arg) const;
        double norm() const;
        Quaternion normalize() const;
        void normalized();
        double dot(const Quaternion& arg) const;
        Quaternion conj() const;
        Quaternion inverse() const;
        Quaternion exp() const;
        Quaternion log() const;
        Quaternion pow(const double t) const;
        Eigen::Matrix3d toRotationMatrix() const;
        static Quaternion convertRotationMatrixToQuaternion(const Eigen::Matrix3d& arg);
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

    private:
        double min_val = 1.0e-9;
    };

    class QuaternionPoint
    {
    public:
        QuaternionPoint() = default;
        QuaternionPoint(const QuaternionPoint& arg);
        ~QuaternionPoint() = default;
        QuaternionPoint& operator=(const QuaternionPoint& arg);
        // slerp插值
        QuaternionPoint slerp(const Quaternion& target, const double h);
        QuaternionPoint slerp(const Quaternion& target, const double h,
                              const double hdot);
        QuaternionPoint slerp(const Quaternion& target, const double h,
                              const double hdot, const double hdotdot);
        QuaternionPoint slerp(const Quaternion& target, const double h,
                              const double hdot, const double hdotdot,
                              const double hdotdotdot);
        // 这个的加加速度计算结果有点儿误差，暂时没有找到问题在哪里

        // 四元数绕固定轴旋转
        QuaternionPoint rotateAroundAxisByTheta(const Eigen::Vector3d axis,
                                                const double theta,
                                                bool is_body_system);
        QuaternionPoint rotateAroundAxisByTheta(const Eigen::Vector3d axis,
                                                const double theta,
                                                const double theta_dot,
                                                bool is_body_system);
        QuaternionPoint rotateAroundAxisByTheta(const Eigen::Vector3d axis,
                                                const double theta,
                                                const double theta_dot,
                                                const double theta_dotdot,
                                                bool is_body_system);
        QuaternionPoint
        rotateAroundAxisByTheta(const Eigen::Vector3d axis, const double theta,
                                const double theta_dot, const double theta_dotdot,
                                const double theta_dotdotdot, bool is_body_system);

        // data
        Quaternion q;
        Quaternion qd;
        Quaternion qdd;
        Quaternion qddd;

        Eigen::Matrix3d rot;
        Eigen::Vector3d angle_vel_in_body_system;
        Eigen::Vector3d angle_acc_in_body_system;
        Eigen::Vector3d angle_jerk_in_body_system;
        Eigen::Vector3d angle_vel_in_space_system;
        Eigen::Vector3d angle_acc_in_space_system;
        Eigen::Vector3d angle_jerk_in_space_system;

    private:
        Eigen::Vector3d convertQuaternionVelToAngleVel(const Quaternion& q,
                                                       const Quaternion& qd,
                                                       bool is_body_system);
        void convertQuaternionVelAccToAngleVelAcc(const Quaternion& q,
                                                  const Quaternion& qd,
                                                  const Quaternion& qdd,
                                                  bool is_body_system,
                                                  Eigen::Vector3d& angle_vel,
                                                  Eigen::Vector3d& angle_acc);

        void convertQuaternionVelAccJerkToAngleVelAccJerk(
            const Quaternion& q, const Quaternion& qd, const Quaternion& qdd,
            const Quaternion& qddd, bool is_body_system, Eigen::Vector3d& angle_vel,
            Eigen::Vector3d& angle_acc, Eigen::Vector3d& angle_jerk);
    };
} // namespace planning
