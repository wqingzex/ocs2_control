/******************************************************************************
 * Custom joint velocity lock constraint.
 ******************************************************************************/

#include "whole_body_mpc/constraint/CustomJointLockConstraint.h"

#include <stdexcept>
#include <utility>

namespace ocs2::whole_body_mpc {

CustomJointLockConstraint::CustomJointLockConstraint(
    const SwitchedHumanoidReferenceManager& referenceManager,
    std::vector<size_t> lockedJointIndices, size_t headInputStart, size_t headInputDof)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      lockedJointIndices_(std::move(lockedJointIndices)),
      headInputStart_(headInputStart), headInputDof_(headInputDof) {
    if (lockedJointIndices_.empty()) {
        throw std::invalid_argument("[CustomJointLockConstraint] lockedJointIndices cannot be empty");
    }
}

std::vector<size_t> CustomJointLockConstraint::activeIndices(scalar_t time) const {
    std::vector<size_t> indices;
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::CUSTOM_LOCKED) return indices;
    const bool head_task = referenceManagerPtr_->isHeadMidpointGazeActive(time) ||
                           referenceManagerPtr_->isHeadActive(time);
    for (const auto index : lockedJointIndices_) {
        if (head_task && index >= headInputStart_ && index - headInputStart_ < headInputDof_) continue;
        indices.push_back(index);
    }
    return indices;
}

size_t CustomJointLockConstraint::getNumConstraints(scalar_t time) const {
    return activeIndices(time).size();
}

vector_t CustomJointLockConstraint::getValue(scalar_t time, const vector_t&, const vector_t& input,
                                            const PreComputation&) const {
    const auto indices = activeIndices(time);
    vector_t value(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) value(i) = input(indices[i]);
    return value;
}

VectorFunctionLinearApproximation CustomJointLockConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const {
    const auto indices = activeIndices(time);
    auto approximation = VectorFunctionLinearApproximation::Zero(indices.size(), state.size(), input.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        approximation.f(i) = input(indices[i]);
        approximation.dfdu(i, indices[i]) = 1.0;
    }
    return approximation;
}

}  // namespace ocs2::whole_body_mpc
