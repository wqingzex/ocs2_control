#pragma once
#include <eigen3/Eigen/Dense>

namespace planning
{
    class JointVector
    {
    public:
        JointVector();
        ~JointVector() = default;

        explicit JointVector(size_t n);

        JointVector(const JointVector& arg);

        JointVector(Eigen::VectorXd arg);

        void resize(size_t n);

        size_t getJointSize() const;

        JointVector& operator=(const JointVector& arg);

        double operator()(size_t index) const;

        double& operator()(size_t index);

        JointVector operator-() const;

        JointVector operator+(const JointVector& arg) const;

        JointVector operator-(const JointVector& arg) const;

        JointVector operator*(double arg) const;

        JointVector operator/(double arg) const;

        static JointVector zero(size_t n);

        static JointVector one(size_t n);

        double norm() { return data.norm(); };

        bool dot(const JointVector& arg, double* res) const;

        bool dot(const JointVector& lhs, const JointVector& rhs, double* res);

        static bool theProjectionOfVectorAontoVectorB(const JointVector& a,
                                                      const JointVector& b, double* res);
        bool theProjectionOntoOtherVector(const JointVector& arg, double* res) const;

        Eigen::VectorXd data;
    };
} // namespace planning
