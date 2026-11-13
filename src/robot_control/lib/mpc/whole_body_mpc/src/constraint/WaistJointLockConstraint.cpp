/******************************************************************************
 * Waist joint lock constraint (mode == LOCKED):
 *   u_body1 = 0, u_body2 = 0, ..., u_bodyN = 0
 ******************************************************************************/

#include "whole_body_mpc/constraint/WaistJointLockConstraint.h"

#include <stdexcept>

namespace ocs2::whole_body_mpc {

WaistJointLockConstraint::WaistJointLockConstraint(
    const SwitchedHumanoidReferenceManager& referenceManager,
    size_t waistInputStartIndex, size_t waistDof)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      waistInputStart_(waistInputStartIndex),
      waistDof_(waistDof) {
    if (waistDof_ == 0) {
        throw std::invalid_argument("[WaistJointLockConstraint] waistDof must be > 0");
    }
}

size_t WaistJointLockConstraint::getNumConstraints(scalar_t time) const {
    return referenceManagerPtr_->getBodyMode(time) == BodyConstraintMode::LOCKED ? waistDof_ : 0;
}

vector_t WaistJointLockConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                            const PreComputation&) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::LOCKED) {
        return vector_t();
    }
    return input.segment(static_cast<Eigen::Index>(waistInputStart_),
                         static_cast<Eigen::Index>(waistDof_));
}

VectorFunctionLinearApproximation WaistJointLockConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::LOCKED) {
        return VectorFunctionLinearApproximation::Zero(0, state.size(), input.size());
    }

    const Eigen::Index n = static_cast<Eigen::Index>(waistDof_);
    const Eigen::Index s = static_cast<Eigen::Index>(waistInputStart_);
    VectorFunctionLinearApproximation approx(n, state.size(), input.size());

    approx.f = input.segment(s, n);
    approx.dfdx.setZero();
    approx.dfdu.setZero();
    approx.dfdu.middleCols(s, n).setIdentity();

    return approx;
}

} // namespace ocs2::whole_body_mpc
