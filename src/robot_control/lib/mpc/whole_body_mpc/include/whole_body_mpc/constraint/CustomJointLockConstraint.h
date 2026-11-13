/******************************************************************************
 * Custom joint velocity lock constraint.
 *
 * Locks arbitrary input indices to zero when BodyConstraintMode == CUSTOM_LOCKED.
 ******************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

class CustomJointLockConstraint final : public StateInputConstraint {
public:
    CustomJointLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                              std::vector<size_t> lockedJointIndices,
                              size_t headInputStart = 0, size_t headInputDof = 0);

    ~CustomJointLockConstraint() override = default;

    CustomJointLockConstraint* clone() const override { return new CustomJointLockConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComputation) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComputation) const override;

private:
    CustomJointLockConstraint(const CustomJointLockConstraint& other) = default;

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    std::vector<size_t> lockedJointIndices_;
    size_t headInputStart_;
    size_t headInputDof_;
    std::vector<size_t> activeIndices(scalar_t time) const;
};

}  // namespace ocs2::whole_body_mpc
