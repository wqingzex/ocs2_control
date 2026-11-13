/******************************************************************************
 * OCS2 Wheel Humanoid - Body Tracking Constraint
 * 
 * Shared implementation for separate RELATIVE and TRACKING constraints.
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * @brief Body constraint active in exactly one configured body mode
 * 
 * RELATIVE constrains base-frame local x only when it leaves a configured
 * interval, plus world-upright roll/pitch. TRACKING follows the full body pose.
 */
class BodyTrackingConstraint final : public StateConstraint {
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;
    
    /**
     * @brief Constructor
     * @param bodyKinematics Kinematics for body link
     * @param referenceManager Reference manager for mode and target queries
     * @param bodyLinkName Name of the body link in URDF
     * @param constraintMode Mode handled by this instance (RELATIVE or TRACKING)
     * @param relativeXLowerBound Lower local-x bound used in RELATIVE mode
     * @param relativeXUpperBound Upper local-x bound used in RELATIVE mode
     */
    BodyTrackingConstraint(
        const EndEffectorKinematics<scalar_t>& bodyKinematics,
        const SwitchedHumanoidReferenceManager& referenceManager,
        const std::string& bodyLinkName,
        BodyConstraintMode constraintMode,
        scalar_t relativeXLowerBound,
        scalar_t relativeXUpperBound,
        size_t baseDim = 0);
    
    ~BodyTrackingConstraint() override = default;
    
    BodyTrackingConstraint* clone() const override {
        return new BodyTrackingConstraint(*bodyKinematicsPtr_, *referenceManagerPtr_, bodyLinkName_,
                                          constraintMode_, relativeXLowerBound_, relativeXUpperBound_, baseDim_);
    }
    
    /**
     * @brief Returns 3 for active RELATIVE, 6 for active TRACKING, otherwise 0
     */
    size_t getNumConstraints(scalar_t time) const override;
    
    /**
     * @brief Compute body constraint value based on current mode
     */
    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;
    
    /**
     * @brief Compute linear approximation of body constraint
     */
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

private:
    BodyTrackingConstraint(const BodyTrackingConstraint& other) = default;
    
    /**
     * @brief Compute constraint for RELATIVE mode (keep balanced)
     */
    vector_t computeRelativeConstraint(const vector_t& state,
                                        const PreComputation& preComputation) const;
    
    /**
     * @brief Compute constraint for TRACKING mode (follow target)
     */
    vector_t computeTrackingConstraint(scalar_t time, const vector_t& state,
                                        const PreComputation& preComputation) const;
    
    /**
     * @brief Compute linear approximation for RELATIVE mode
     */
    VectorFunctionLinearApproximation computeRelativeLinearApproximation(
        const vector_t& state, const PreComputation& preComputation) const;
    
    /**
     * @brief Compute linear approximation for TRACKING mode
     */
    VectorFunctionLinearApproximation computeTrackingLinearApproximation(
        scalar_t time, const vector_t& state, const PreComputation& preComputation) const;
    
    std::unique_ptr<EndEffectorKinematics<scalar_t>> bodyKinematicsPtr_;
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_;
    std::string bodyLinkName_;
    BodyConstraintMode constraintMode_;
    scalar_t relativeXLowerBound_;
    scalar_t relativeXUpperBound_;
    
    // Cached pointer for PinocchioEndEffectorKinematics
    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;
    
    // Number of base (chassis) states at the head of the state vector (0 for fixed_base)
    size_t baseDim_ = 0;

    // Target for RELATIVE mode (identity = keep upright)
    quaternion_t relativeTargetOrientation_ = quaternion_t::Identity();
};

} // namespace ocs2::whole_body_mpc
