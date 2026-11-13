/******************************************************************************
 * Head-waist coupling constraint (mode == LOCKED):
 *   For each locked waist joint i:  u[lockedWaistJoints[i]] = 0
 *   Coupling equation:              u[waistJointIndex] - alpha * u[headJointIndex] = 0
 ******************************************************************************/

#include "whole_body_mpc/constraint/HeadCouplingConstraint.h"

namespace ocs2::whole_body_mpc {

HeadCouplingConstraint::HeadCouplingConstraint(
    const SwitchedHumanoidReferenceManager& referenceManager,
    size_t waistJointIndex, size_t headJointIndex, scalar_t alpha,
    std::vector<size_t> lockedWaistJoints)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      waistJointIndex_(waistJointIndex),
      headJointIndex_(headJointIndex),
      alpha_(alpha),
      lockedWaistJoints_(std::move(lockedWaistJoints)) {}

size_t HeadCouplingConstraint::getNumConstraints(scalar_t time) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::HEAD_COUPLED) {
        return 0;
    }
    return lockedWaistJoints_.size() + 1;  // locked waist joints + coupling equation
}

vector_t HeadCouplingConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation&) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::HEAD_COUPLED) {
        return vector_t();
    }

    const Eigen::Index numConstraints = static_cast<Eigen::Index>(lockedWaistJoints_.size() + 1);
    vector_t constraint(numConstraints);

    // Lock constraints: u[lockedWaistJoints[i]] = 0
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(lockedWaistJoints_.size()); ++i) {
        constraint(i) = input(static_cast<Eigen::Index>(lockedWaistJoints_[i]));
    }

    // Coupling: u[waistJointIndex] - alpha * u[headJointIndex] = 0
    constraint(numConstraints - 1) =
        input(static_cast<Eigen::Index>(waistJointIndex_)) -
        alpha_ * input(static_cast<Eigen::Index>(headJointIndex_));

    return constraint;
}

VectorFunctionLinearApproximation HeadCouplingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::HEAD_COUPLED) {
        return VectorFunctionLinearApproximation::Zero(0, state.size(), input.size());
    }

    const Eigen::Index numConstraints = static_cast<Eigen::Index>(lockedWaistJoints_.size() + 1);
    VectorFunctionLinearApproximation approx(numConstraints, state.size(), input.size());

    // f values
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(lockedWaistJoints_.size()); ++i) {
        approx.f(i) = input(static_cast<Eigen::Index>(lockedWaistJoints_[i]));
    }
    approx.f(numConstraints - 1) =
        input(static_cast<Eigen::Index>(waistJointIndex_)) -
        alpha_ * input(static_cast<Eigen::Index>(headJointIndex_));

    approx.dfdx.setZero();
    approx.dfdu.setZero();

    // dfdu: d(u[lockedWaistJoints[i]])/du = e_i (unit vector at that index)
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(lockedWaistJoints_.size()); ++i) {
        approx.dfdu(i, static_cast<Eigen::Index>(lockedWaistJoints_[i])) = 1.0;
    }

    // dfdu: d(u[waistJointIndex] - alpha * u[headJointIndex])/du
    approx.dfdu(numConstraints - 1, static_cast<Eigen::Index>(waistJointIndex_)) = 1.0;
    approx.dfdu(numConstraints - 1, static_cast<Eigen::Index>(headJointIndex_)) = -alpha_;

    return approx;
}

} // namespace ocs2::whole_body_mpc
