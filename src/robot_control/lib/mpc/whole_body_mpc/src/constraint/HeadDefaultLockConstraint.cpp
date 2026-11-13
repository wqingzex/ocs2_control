/******************************************************************************
 * Implicit head joint velocity lock (per-joint; coupled head exempt in HEAD_COUPLED).
 ******************************************************************************/

#include "whole_body_mpc/constraint/HeadDefaultLockConstraint.h"

#include <stdexcept>

#include "whole_body_mpc/mode/HumanoidModeSchedule.h"

namespace ocs2::whole_body_mpc {

HeadDefaultLockConstraint::HeadDefaultLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                                                     size_t headInputStartIndex, size_t headDof,
                                                     size_t coupledHeadInputIndex, bool hasCoupledHeadJoint)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      headInputStart_(headInputStartIndex),
      headDof_(headDof),
      coupledHeadInputIndex_(coupledHeadInputIndex),
      hasCoupledHeadJoint_(hasCoupledHeadJoint) {
    if (headDof_ == 0) {
        throw std::invalid_argument("[HeadDefaultLockConstraint] headDof must be > 0");
    }
}

void HeadDefaultLockConstraint::collectLockedInputIndices(scalar_t time, std::vector<size_t>& lockedIndices) const {
    lockedIndices.clear();
    lockedIndices.reserve(headDof_);

    const auto bodyMode = referenceManagerPtr_->getBodyMode(time);
    if (referenceManagerPtr_->isHeadActive(time) || referenceManagerPtr_->isHeadMidpointGazeActive(time)) {
        return;
    }
    for (size_t offset = 0; offset < headDof_; ++offset) {
        const size_t inputIndex = headInputStart_ + offset;
        if (bodyMode == BodyConstraintMode::HEAD_COUPLED && hasCoupledHeadJoint_ &&
            inputIndex == coupledHeadInputIndex_) {
            continue;
        }
        lockedIndices.push_back(inputIndex);
    }
}

size_t HeadDefaultLockConstraint::getNumConstraints(scalar_t time) const {
    std::vector<size_t> lockedIndices;
    collectLockedInputIndices(time, lockedIndices);
    return lockedIndices.size();
}

vector_t HeadDefaultLockConstraint::getValue(scalar_t time, const vector_t& /*state*/, const vector_t& input,
                                             const PreComputation& /*preComputation*/) const {
    std::vector<size_t> lockedIndices;
    collectLockedInputIndices(time, lockedIndices);
    if (lockedIndices.empty()) {
        return vector_t();
    }

    vector_t out(static_cast<Eigen::Index>(lockedIndices.size()));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(lockedIndices.size()); ++i) {
        out(i) = input(static_cast<Eigen::Index>(lockedIndices[static_cast<size_t>(i)]));
    }
    return out;
}

VectorFunctionLinearApproximation HeadDefaultLockConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& /*preComputation*/) const {
    std::vector<size_t> lockedIndices;
    collectLockedInputIndices(time, lockedIndices);
    if (lockedIndices.empty()) {
        return VectorFunctionLinearApproximation::Zero(0, state.size(), input.size());
    }

    const Eigen::Index n = static_cast<Eigen::Index>(lockedIndices.size());
    VectorFunctionLinearApproximation approx(n, state.size(), input.size());
    approx.f.setZero();
    approx.dfdx.setZero();
    approx.dfdu.setZero();

    for (Eigen::Index i = 0; i < n; ++i) {
        const Eigen::Index inputIndex = static_cast<Eigen::Index>(lockedIndices[static_cast<size_t>(i)]);
        approx.f(i) = input(inputIndex);
        approx.dfdu(i, inputIndex) = 1.0;
    }

    return approx;
}

}  // namespace ocs2::whole_body_mpc
