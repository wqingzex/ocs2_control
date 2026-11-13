/******************************************************************************
 * Implicit default head joint velocity lock:
 *   u[i] = 0 for every head input index i that is not explicitly allowed to move.
 *
 * In HEAD_COUPLED mode only the coupled head joint (headCoupling.headJointIndex)
 * is excluded; all other head joints stay locked.
 *
 * Enabled automatically when the model has head DOF; no task-file switch.
 ******************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

class HeadDefaultLockConstraint final : public StateInputConstraint {
public:
    /**
     * @param coupledHeadInputIndex  Input index of the head joint in headCoupling (ignored when hasCoupledHeadJoint=false).
     * @param hasCoupledHeadJoint    True when headCoupling is configured and the coupled index lies in the head block.
     */
    HeadDefaultLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager, size_t headInputStartIndex,
                              size_t headDof, size_t coupledHeadInputIndex, bool hasCoupledHeadJoint);

    ~HeadDefaultLockConstraint() override = default;

    HeadDefaultLockConstraint* clone() const override { return new HeadDefaultLockConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComputation) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComputation) const override;

private:
    HeadDefaultLockConstraint(const HeadDefaultLockConstraint& other) = default;

    void collectLockedInputIndices(scalar_t time, std::vector<size_t>& lockedIndices) const;

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    size_t headInputStart_{0};
    size_t headDof_{0};
    size_t coupledHeadInputIndex_{0};
    bool hasCoupledHeadJoint_{false};
};

}  // namespace ocs2::whole_body_mpc
