#include "motion_planning/common/joint_vector.h"

namespace planning
{
    JointVector::JointVector() = default;

    JointVector::JointVector(size_t n) : data(n) { data.setZero(); };

    JointVector::JointVector(const JointVector& arg) : data(arg.data)
    {
    };

    JointVector::JointVector(Eigen::VectorXd arg) : data(arg)
    {
    };

    void JointVector::resize(size_t n)
    {
        data.resize(n);
        data.setZero();
    };

    size_t JointVector::getJointSize() const { return data.size(); };

    JointVector& JointVector::operator=(const JointVector& arg)
    {
        data = arg.data;
        return *this;
    }

    double JointVector::operator()(size_t index) const { return data(index); };

    double& JointVector::operator()(size_t index) { return data(index); };

    JointVector JointVector::operator-() const { return JointVector(-data); };

    JointVector JointVector::operator+(const JointVector& arg) const
    {
        return JointVector(data + arg.data);
    };

    JointVector JointVector::operator-(const JointVector& arg) const
    {
        return JointVector(data - arg.data);
    }

    JointVector JointVector::operator*(double arg) const
    {
        return JointVector(data * arg);
    }

    JointVector JointVector::operator/(double arg) const
    {
        return JointVector(data / arg);
    };

    JointVector JointVector::zero(size_t n)
    {
        return JointVector(Eigen::VectorXd::Zero(n));
    };

    JointVector JointVector::one(size_t n)
    {
        return JointVector(Eigen::VectorXd::Ones(n));
    };

    bool JointVector::dot(const JointVector& arg, double* res) const
    {
        if (this->getJointSize() != arg.getJointSize())
        {
            return false;
        }

        *res = this->data.dot(arg.data);
        return true;
    }

    bool JointVector::dot(const JointVector& lhs, const JointVector& rhs,
                          double* res)
    {
        if (lhs.getJointSize() != rhs.getJointSize())
        {
            return false;
        }

        *res = lhs.data.dot(rhs.data);
        return true;
    };

    bool JointVector::theProjectionOfVectorAontoVectorB(const JointVector& a,
                                                        const JointVector& b,
                                                        double* res)
    {
        double minval = 1.0e-9;
        double adotb, bdotb;
        if (!a.dot(b, &adotb) || !b.dot(b, &bdotb))
        {
            return false;
        }
        if (bdotb < minval)
        {
            return false;
        }

        *res = adotb / bdotb;
        return true;
    };

    bool JointVector::theProjectionOntoOtherVector(const JointVector& arg,
                                                   double* res) const
    {
        double minval = 1.0e-9;
        double adotb, bdotb;
        if (!this->dot(arg, &adotb) || !arg.dot(arg, &bdotb))
        {
            return false;
        }
        if (bdotb < minval)
        {
            return false;
        }

        *res = adotb / bdotb;
        return true;
    };
} // namespace planning
