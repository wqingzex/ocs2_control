/******************************************************************************
 * OCS2 Wheel Humanoid - Body Tracking Constraint Implementation
 ******************************************************************************/

#include "whole_body_mpc/constraint/BodyTrackingConstraint.h"
#include <rclcpp/logging.hpp>

#include <cmath>
#include <stdexcept>

// Reuse PreComputation from mobile_manipulator
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

namespace ocs2::whole_body_mpc {

using mobile_manipulator::MobileManipulatorPreComputation;

//=============================================================================
// Constructor
//=============================================================================

BodyTrackingConstraint::BodyTrackingConstraint(
    const EndEffectorKinematics<scalar_t>& bodyKinematics,
    const SwitchedHumanoidReferenceManager& referenceManager,
    const std::string& bodyLinkName,
    BodyConstraintMode constraintMode,
    scalar_t relativeXLowerBound,
    scalar_t relativeXUpperBound,
    size_t baseDim)
    : StateConstraint(ConstraintOrder::Linear)
    , bodyKinematicsPtr_(bodyKinematics.clone())
    , referenceManagerPtr_(&referenceManager)
    , bodyLinkName_(bodyLinkName)
    , constraintMode_(constraintMode)
    , relativeXLowerBound_(relativeXLowerBound)
    , relativeXUpperBound_(relativeXUpperBound)
    , baseDim_(baseDim) {
    if (constraintMode_ != BodyConstraintMode::RELATIVE &&
        constraintMode_ != BodyConstraintMode::TRACKING) {
        throw std::invalid_argument("[BodyTrackingConstraint] constraintMode must be RELATIVE or TRACKING");
    }
    if (relativeXLowerBound_ > relativeXUpperBound_) {
        throw std::invalid_argument("[BodyTrackingConstraint] relative x lower bound exceeds upper bound");
    }
    
    // Cache PinocchioEndEffectorKinematics pointer
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(
        bodyKinematicsPtr_.get());
    
    // Verify body link is in kinematics
    const auto& ids = bodyKinematics.getIds();
    bool found = false;
    for (const auto& id : ids) {
        if (id == bodyLinkName_) {
            found = true;
            break;
        }
    }
    if (!found) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("BodyTrackingConstraint"), "Warning: body link '" << bodyLinkName_
                  << "' not found in kinematics IDs");
    }
}

//=============================================================================
// Constraint Interface
//=============================================================================

size_t BodyTrackingConstraint::getNumConstraints(scalar_t time) const {
    BodyConstraintMode mode = referenceManagerPtr_->getBodyMode(time);
    if (mode != constraintMode_) {
        return 0;
    }
    
    switch (mode) {
        case BodyConstraintMode::FREE:
            return 0;
        case BodyConstraintMode::RELATIVE:
            return 3;  // local-x bound violation, roll, pitch
        case BodyConstraintMode::TRACKING:
            return 6;  // full pose
        case BodyConstraintMode::LOCKED:
            return 0;  // waist held by WaistJointLockConstraint on inputs
        default:
            return 0;
    }
}

vector_t BodyTrackingConstraint::getValue(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    BodyConstraintMode mode = referenceManagerPtr_->getBodyMode(time);
    if (mode != constraintMode_) {
        return vector_t::Zero(0);
    }
    
    switch (mode) {
        case BodyConstraintMode::FREE:
            return vector_t::Zero(0);
        case BodyConstraintMode::RELATIVE:
            return computeRelativeConstraint(state, preComputation);
        case BodyConstraintMode::TRACKING:
            return computeTrackingConstraint(time, state, preComputation);
        case BodyConstraintMode::LOCKED:
            return vector_t::Zero(0);
        default:
            return vector_t::Zero(0);
    }
}

VectorFunctionLinearApproximation BodyTrackingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    BodyConstraintMode mode = referenceManagerPtr_->getBodyMode(time);
    if (mode != constraintMode_) {
        return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
    
    switch (mode) {
        case BodyConstraintMode::FREE:
            return VectorFunctionLinearApproximation(0, state.size(), 0);
        case BodyConstraintMode::RELATIVE:
            return computeRelativeLinearApproximation(state, preComputation);
        case BodyConstraintMode::TRACKING:
            return computeTrackingLinearApproximation(time, state, preComputation);
        case BodyConstraintMode::LOCKED:
            return VectorFunctionLinearApproximation(0, state.size(), 0);
        default:
            return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
}

//=============================================================================
// RELATIVE Mode Implementation
//=============================================================================

vector_t BodyTrackingConstraint::computeRelativeConstraint(
    const vector_t& state,
    const PreComputation& preComputation) const {
    
    // Update PinocchioInterface if using precomputation
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }
    
    // Get current body orientation error (relative to upright)
    const auto orientationErrors = bodyKinematicsPtr_->getOrientationError(
        state, {relativeTargetOrientation_});
    
    // Get current body position
    const auto positions = bodyKinematicsPtr_->getPosition(state);
    
    const bool mobileBase = baseDim_ >= 3;
    const scalar_t baseX = mobileBase ? state[0] : 0.0;
    const scalar_t baseY = mobileBase ? state[1] : 0.0;
    const scalar_t baseYaw = mobileBase ? state[2] : 0.0;
    const scalar_t dx = positions[0].x() - baseX;
    const scalar_t dy = positions[0].y() - baseY;
    const scalar_t localX = std::cos(baseYaw) * dx + std::sin(baseYaw) * dy;

    scalar_t xViolation = 0.0;
    if (localX < relativeXLowerBound_) {
        xViolation = localX - relativeXLowerBound_;
    } else if (localX > relativeXUpperBound_) {
        xViolation = localX - relativeXUpperBound_;
    }

    vector_t constraint(3);
    constraint[0] = xViolation;
    constraint.tail<2>() = orientationErrors[0].head<2>();
    
    return constraint;
}

VectorFunctionLinearApproximation BodyTrackingConstraint::computeRelativeLinearApproximation(
    const vector_t& state,
    const PreComputation& preComputation) const {
    
    // Update PinocchioInterface if using precomputation
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }
    
    const auto orientationApprox = bodyKinematicsPtr_->getOrientationErrorLinearApproximation(
        state, {relativeTargetOrientation_});
    const auto positionApprox = bodyKinematicsPtr_->getPositionLinearApproximation(state);
    
    auto approximation = VectorFunctionLinearApproximation(3, state.size(), 0);
    approximation.f.setZero();
    approximation.dfdx.setZero();

    const bool mobileBase = baseDim_ >= 3;
    const scalar_t baseX = mobileBase ? state[0] : 0.0;
    const scalar_t baseY = mobileBase ? state[1] : 0.0;
    const scalar_t baseYaw = mobileBase ? state[2] : 0.0;
    const scalar_t cosYaw = std::cos(baseYaw);
    const scalar_t sinYaw = std::sin(baseYaw);
    const scalar_t dx = positionApprox[0].f.x() - baseX;
    const scalar_t dy = positionApprox[0].f.y() - baseY;
    const scalar_t localX = cosYaw * dx + sinYaw * dy;

    if (localX < relativeXLowerBound_ || localX > relativeXUpperBound_) {
        approximation.f[0] = localX < relativeXLowerBound_
            ? localX - relativeXLowerBound_
            : localX - relativeXUpperBound_;
        approximation.dfdx.row(0) =
            cosYaw * positionApprox[0].dfdx.row(0) +
            sinYaw * positionApprox[0].dfdx.row(1);
        if (mobileBase) {
            approximation.dfdx(0, 0) -= cosYaw;
            approximation.dfdx(0, 1) -= sinYaw;
            approximation.dfdx(0, 2) += -sinYaw * dx + cosYaw * dy;
        }
    }

    approximation.f.tail<2>() = orientationApprox[0].f.head<2>();
    approximation.dfdx.bottomRows<2>() = orientationApprox[0].dfdx.topRows<2>();
    
    return approximation;
}

//=============================================================================
// TRACKING Mode Implementation
//=============================================================================

vector_t BodyTrackingConstraint::computeTrackingConstraint(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    // Update PinocchioInterface if using precomputation
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }
    
    // Get target body pose from reference manager
    auto [targetPos, targetQuat] = referenceManagerPtr_->interpolateBodyPose(time);
    
    // Get current body position and orientation error
    const auto positions = bodyKinematicsPtr_->getPosition(state);
    const auto orientationErrors = bodyKinematicsPtr_->getOrientationError(
        state, {targetQuat});
    
    // Full pose constraint: [position_error, orientation_error]
    vector_t constraint(6);
    constraint.head<3>() = positions[0] - targetPos;
    constraint.tail<3>() = orientationErrors[0];
    
    return constraint;
}

VectorFunctionLinearApproximation BodyTrackingConstraint::computeTrackingLinearApproximation(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    // Update PinocchioInterface if using precomputation
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }
    
    auto [targetPos, targetQuat] = referenceManagerPtr_->interpolateBodyPose(time);
    
    const auto positionApprox = bodyKinematicsPtr_->getPositionLinearApproximation(state);
    const auto orientationApprox = bodyKinematicsPtr_->getOrientationErrorLinearApproximation(
        state, {targetQuat});
    
    auto approximation = VectorFunctionLinearApproximation(6, state.size(), 0);
    
    // Position
    approximation.f.head<3>() = positionApprox[0].f - targetPos;
    approximation.dfdx.topRows<3>() = positionApprox[0].dfdx;
    
    // Orientation
    approximation.f.tail<3>() = orientationApprox[0].f;
    approximation.dfdx.bottomRows<3>() = orientationApprox[0].dfdx;
    
    return approximation;
}

} // namespace ocs2::whole_body_mpc
