/******************************************************************************
 * Head-waist coupling constraint (mode == LOCKED):
 *
 *   For each locked waist joint i:  u[lockedWaistJoints[i]] = 0
 *   Coupling equation:              u[waistJointIndex] - alpha * u[headJointIndex] = 0
 *
 * Activated only when BodyConstraintMode == HEAD_COUPLED.
 * Used when headCoupling.activate = true in task.info (paired with HEAD_FORWARD mode command).
 *
 * Input layout assumed:
 *   [base(2) | waist(4) | head(2) | left_arm(7) | right_arm(7)]
 * All indices refer to the full input vector.
 ******************************************************************************/

#pragma once

#include <vector>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

class HeadCouplingConstraint final : public StateInputConstraint {
public:
    /**
     * @param referenceManager   Reference manager for mode queries.
     * @param waistJointIndex    Index in input of the waist joint participating in coupling.
     * @param headJointIndex     Index in input of the head joint participating in coupling.
     * @param alpha              Coupling coefficient: u[waistJointIndex] - alpha * u[headJointIndex] = 0.
     * @param lockedWaistJoints  Indices in input of other waist joints to lock (velocity forced to 0).
     */
    HeadCouplingConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                           size_t waistJointIndex, size_t headJointIndex, scalar_t alpha,
                           std::vector<size_t> lockedWaistJoints);

    ~HeadCouplingConstraint() override = default;

    HeadCouplingConstraint* clone() const override { return new HeadCouplingConstraint(*this); }

    /// Returns lockedWaistJoints.size() + 1 when LOCKED, else 0.
    size_t getNumConstraints(scalar_t time) const override;

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComputation) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComputation) const override;

private:
    HeadCouplingConstraint(const HeadCouplingConstraint& other) = default;

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    size_t waistJointIndex_{0};          ///< Input index of the coupled waist joint
    size_t headJointIndex_{0};           ///< Input index of the coupled head joint
    scalar_t alpha_{1.0};                ///< u[waistJoint] - alpha * u[headJoint] = 0
    std::vector<size_t> lockedWaistJoints_;  ///< Input indices of waist joints locked to zero
};

} // namespace ocs2::whole_body_mpc
