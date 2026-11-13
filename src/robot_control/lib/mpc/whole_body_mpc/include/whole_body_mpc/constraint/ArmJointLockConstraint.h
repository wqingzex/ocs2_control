/******************************************************************************
 * Lock arm joint velocities to zero when that arm is marked inactive in mode
 * (same mechanism as WaistJointLockConstraint for LOCKED).
 ******************************************************************************/

#pragma once

#include <cstddef>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

class ArmJointLockConstraint final : public StateInputConstraint {
public:
    ArmJointLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager, size_t leftInputStart,
                           size_t leftArmDof, size_t rightInputStart, size_t rightArmDof);

    ~ArmJointLockConstraint() override = default;

    ArmJointLockConstraint* clone() const override {
        return new ArmJointLockConstraint(*referenceManagerPtr_, leftInputStart_, leftArmDof_, rightInputStart_,
                                          rightArmDof_);
    }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComputation) const override;

private:
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    size_t leftInputStart_{0};
    size_t leftArmDof_{0};
    size_t rightInputStart_{0};
    size_t rightArmDof_{0};
};

}  // namespace ocs2::whole_body_mpc
