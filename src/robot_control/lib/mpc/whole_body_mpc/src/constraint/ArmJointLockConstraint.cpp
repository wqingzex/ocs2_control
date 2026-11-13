/******************************************************************************
 * Arm joint velocity lock when leftArmActive / rightArmActive is false
 ******************************************************************************/

#include "whole_body_mpc/constraint/ArmJointLockConstraint.h"

#include <stdexcept>

namespace ocs2::whole_body_mpc {

ArmJointLockConstraint::ArmJointLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                                               size_t leftInputStart, size_t leftArmDof, size_t rightInputStart,
                                               size_t rightArmDof)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      leftInputStart_(leftInputStart),
      leftArmDof_(leftArmDof),
      rightInputStart_(rightInputStart),
      rightArmDof_(rightArmDof) {
    if (leftArmDof_ == 0 && rightArmDof_ == 0) {
        throw std::invalid_argument("[ArmJointLockConstraint] leftArmDof and rightArmDof cannot both be zero");
    }
}

size_t ArmJointLockConstraint::getNumConstraints(scalar_t time) const {
    size_t n = 0;
    if (!referenceManagerPtr_->isLeftArmActive(time)) {
        n += leftArmDof_;
    }
    if (!referenceManagerPtr_->isRightArmActive(time)) {
        n += rightArmDof_;
    }
    return n;
}

vector_t ArmJointLockConstraint::getValue(scalar_t time, const vector_t& /*state*/, const vector_t& input,
                                          const PreComputation& /*preComputation*/) const {
    const size_t n = getNumConstraints(time);
    if (n == 0) {
        return vector_t();
    }
    vector_t out(static_cast<Eigen::Index>(n));
    Eigen::Index row = 0;
    if (!referenceManagerPtr_->isLeftArmActive(time)) {
        const Eigen::Index ld = static_cast<Eigen::Index>(leftArmDof_);
        const Eigen::Index ls = static_cast<Eigen::Index>(leftInputStart_);
        out.segment(row, ld) = input.segment(ls, ld);
        row += ld;
    }
    if (!referenceManagerPtr_->isRightArmActive(time)) {
        const Eigen::Index rd = static_cast<Eigen::Index>(rightArmDof_);
        const Eigen::Index rs = static_cast<Eigen::Index>(rightInputStart_);
        out.segment(row, rd) = input.segment(rs, rd);
    }
    return out;
}

VectorFunctionLinearApproximation ArmJointLockConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComputation) const {
    const size_t n = getNumConstraints(time);
    if (n == 0) {
        return VectorFunctionLinearApproximation::Zero(0, state.size(), input.size());
    }

    const Eigen::Index nIdx = static_cast<Eigen::Index>(n);
    VectorFunctionLinearApproximation approx(nIdx, state.size(), input.size());
    approx.f.setZero();
    approx.dfdx.setZero();
    approx.dfdu.setZero();

    Eigen::Index row = 0;
    if (!referenceManagerPtr_->isLeftArmActive(time)) {
        const Eigen::Index ld = static_cast<Eigen::Index>(leftArmDof_);
        const Eigen::Index ls = static_cast<Eigen::Index>(leftInputStart_);
        approx.f.segment(row, ld) = input.segment(ls, ld);
        approx.dfdu.block(row, ls, ld, ld).setIdentity();
        row += ld;
    }
    if (!referenceManagerPtr_->isRightArmActive(time)) {
        const Eigen::Index rd = static_cast<Eigen::Index>(rightArmDof_);
        const Eigen::Index rs = static_cast<Eigen::Index>(rightInputStart_);
        approx.f.segment(row, rd) = input.segment(rs, rd);
        approx.dfdu.block(row, rs, rd, rd).setIdentity();
    }
    return approx;
}

}  // namespace ocs2::whole_body_mpc
