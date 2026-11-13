/******************************************************************************
 * Waist joint lock constraint (mode == LOCKED):
 *   u_body1 = 0, u_body2 = 0, ..., u_bodyN = 0
 *
 * Locks all waist joint velocities to zero when BodyConstraintMode == LOCKED.
 * For head-waist coupling, see HeadCouplingConstraint.
 ******************************************************************************/

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

class WaistJointLockConstraint final : public StateInputConstraint {
public:
    /**
     * @param referenceManager     Reference manager for mode queries.
     * @param waistInputStartIndex Index in input where waist joint velocities start (after base inputs).
     * @param waistDof             Number of waist joints (e.g. 4 for body_joint1..4).
     */
    WaistJointLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                             size_t waistInputStartIndex, size_t waistDof);

    ~WaistJointLockConstraint() override = default;

    WaistJointLockConstraint* clone() const override { return new WaistJointLockConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComputation) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComputation) const override;

private:
    WaistJointLockConstraint(const WaistJointLockConstraint& other) = default;

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    size_t waistInputStart_{0};
    size_t waistDof_{0};
};

} // namespace ocs2::whole_body_mpc
