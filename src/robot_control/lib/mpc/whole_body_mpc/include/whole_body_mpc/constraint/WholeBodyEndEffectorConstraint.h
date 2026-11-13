/******************************************************************************
 * Dual-arm end-effector tracking: left is master when bimanual COUPLED;
 * right-hand Cartesian tracking is disabled (zero residual / zero Jacobian rows)
 * so BimanualCouplingConstraint defines the follower arm.
 ******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * Same residual dimension as mobile_manipulator::EndEffectorConstraint (12 for dual arm) so
 * existing penalty vectors in WholeBodyInterface stay valid. In COUPLED mode the last
 * six components are held at zero with zero linearization so only the left arm tracks the
 * reference trajectory; relative pose is left to BimanualCouplingConstraint.
 */
class WholeBodyEndEffectorConstraint final : public StateConstraint {
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;

    WholeBodyEndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                       const SwitchedHumanoidReferenceManager& referenceManager);

    ~WholeBodyEndEffectorConstraint() override = default;

    WholeBodyEndEffectorConstraint* clone() const override {
        return new WholeBodyEndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_);
    }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const PreComputation& preComputation) const override;

private:
    std::pair<vector_t, quaternion_t> interpolateLeftArmPose(scalar_t time) const;
    std::pair<vector_t, quaternion_t> interpolateRightArmPose(scalar_t time) const;

    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_{nullptr};
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
};

} // namespace ocs2::whole_body_mpc
