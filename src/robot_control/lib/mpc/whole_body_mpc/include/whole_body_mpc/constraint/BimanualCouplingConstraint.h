/******************************************************************************
 * OCS2 Wheel Humanoid - Bimanual Coupling Constraint
 * 
 * Enforces relative pose constraint between left and right end effectors.
 * Used when both arms need to maintain a fixed relative pose (e.g., carrying).
 ******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * @brief Constraint for bimanual coordination (follower does not use right-arm trajectory).
 *
 * Relative pose is captured in the left EE frame and enforced against the *measured* left EE
 * (same Pinocchio frames as EndEffectorKinematics). This keeps the pair rigid when the waist
 * moves under BODY_TRACKING; anchoring on the left *target* would leave the right at a stale
 * world pose while the real left moves with the torso.
 */
class BimanualCouplingConstraint final : public StateConstraint {
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;
    
    /**
     * @brief Constructor
     * @param dualArmKinematics Kinematics with both end effectors [left, right]
     * @param referenceManager Reference manager for mode and target queries
     */
    BimanualCouplingConstraint(
        const EndEffectorKinematics<scalar_t>& dualArmKinematics,
        const SwitchedHumanoidReferenceManager& referenceManager);
    
    ~BimanualCouplingConstraint() override = default;
    
    BimanualCouplingConstraint* clone() const override {
        return new BimanualCouplingConstraint(*dualArmKinematicsPtr_, *referenceManagerPtr_);
    }
    
    /**
     * @brief Returns 6 when coupled, 0 otherwise
     */
    size_t getNumConstraints(scalar_t time) const override;
    
    /**
     * @brief Compute coupling constraint value
     */
    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;
    
    /**
     * @brief Compute linear approximation of coupling constraint
     */
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

private:
    BimanualCouplingConstraint(const BimanualCouplingConstraint& other) = default;
    
    /**
     * @brief Compute orientation error between two quaternions
     */
    vector3_t computeOrientationError(const quaternion_t& current,
                                       const quaternion_t& desired) const;
    
    std::unique_ptr<EndEffectorKinematics<scalar_t>> dualArmKinematicsPtr_;
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_;
    
    // Cached pointer for PinocchioEndEffectorKinematics
    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;
};

} // namespace ocs2::whole_body_mpc

