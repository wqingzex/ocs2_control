#pragma once

#include <memory>
#include <string>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/** Full-pose head tracking residual, active only when the independent head flag is enabled. */
class HeadTrackingConstraint final : public StateConstraint {
public:
    HeadTrackingConstraint(const EndEffectorKinematics<scalar_t>& headKinematics,
                           const SwitchedHumanoidReferenceManager& referenceManager,
                           std::string headLinkName);

    HeadTrackingConstraint* clone() const override {
        return new HeadTrackingConstraint(*headKinematicsPtr_, *referenceManagerPtr_, headLinkName_);
    }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

private:
    HeadTrackingConstraint(const HeadTrackingConstraint&) = default;
    void updateKinematics(const PreComputation& preComputation) const;

    std::unique_ptr<EndEffectorKinematics<scalar_t>> headKinematicsPtr_;
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_;
    std::string headLinkName_;
    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_{nullptr};
};

}  // namespace ocs2::whole_body_mpc
