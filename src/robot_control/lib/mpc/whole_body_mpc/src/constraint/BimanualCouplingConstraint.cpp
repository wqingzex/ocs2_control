/******************************************************************************
 * OCS2 Wheel Humanoid - Bimanual Coupling Constraint Implementation
 ******************************************************************************/

#include "whole_body_mpc/constraint/BimanualCouplingConstraint.h"

#include <cmath>
#include <stdexcept>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2::whole_body_mpc {

using mobile_manipulator::MobileManipulatorPreComputation;

//=============================================================================
// Constructor
//=============================================================================

BimanualCouplingConstraint::BimanualCouplingConstraint(
    const EndEffectorKinematics<scalar_t>& dualArmKinematics,
    const SwitchedHumanoidReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear)
    , dualArmKinematicsPtr_(dualArmKinematics.clone())
    , referenceManagerPtr_(&referenceManager) {
    
    if (dualArmKinematics.getIds().size() != 2) {
        throw std::runtime_error(
            "[BimanualCouplingConstraint] Expected 2 end effectors (left, right), got " +
            std::to_string(dualArmKinematics.getIds().size()));
    }
    
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(
        dualArmKinematicsPtr_.get());
    if (pinocchioEEKinPtr_ == nullptr) {
        throw std::runtime_error(
            "[BimanualCouplingConstraint] Requires PinocchioEndEffectorKinematics "
            "(model_settings.usePreComputation=true). CppAd kinematics is not supported for measured-left coupling.");
    }
}

//=============================================================================
// Constraint Interface
//=============================================================================

size_t BimanualCouplingConstraint::getNumConstraints(scalar_t time) const {
    if (!referenceManagerPtr_->isBimanualCoupled(time)) {
        return 0;
    }
    if (!referenceManagerPtr_->isLeftArmActive(time) || !referenceManagerPtr_->isRightArmActive(time)) {
        return 0;
    }
    return 6;
}

namespace {
BimanualCouplingConstraint::quaternion_t eeQuatFromKinematicsFrame(const PinocchioInterface& pinocchio,
                                                                   const std::string& frameName) {
    // Must match PinocchioEndEffectorKinematics frame resolution (getBodyId).
    const auto frameId = pinocchio.getModel().getBodyId(frameName);
    return matrixToQuaternion(pinocchio.getData().oMf[frameId].rotation());
}

/** Inverse right Jacobian of SO(3): d Log(R Exp(dtheta)) / dtheta. */
BimanualCouplingConstraint::matrix3_t rightJacobianInverseSo3(
    const BimanualCouplingConstraint::vector3_t& rotationVector) {
    using scalar_t = ocs2::scalar_t;
    const scalar_t theta = rotationVector.norm();
    const auto rotationVectorSkew = skewSymmetricMatrix(rotationVector);

    scalar_t secondOrderCoefficient;
    if (theta < static_cast<scalar_t>(1e-3)) {
        // 1 / theta^2 - (1 + cos(theta)) / (2 theta sin(theta))
        secondOrderCoefficient = static_cast<scalar_t>(1.0 / 12.0) + theta * theta / 720.0;
    } else {
        secondOrderCoefficient =
            static_cast<scalar_t>(1.0) / (theta * theta) -
            static_cast<scalar_t>(0.5) / (theta * std::tan(static_cast<scalar_t>(0.5) * theta));
    }

    return BimanualCouplingConstraint::matrix3_t::Identity() +
           static_cast<scalar_t>(0.5) * rotationVectorSkew +
           secondOrderCoefficient * rotationVectorSkew * rotationVectorSkew;
}
}  // namespace

vector_t BimanualCouplingConstraint::getValue(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    if (!referenceManagerPtr_->isBimanualCoupled(time)) {
        return vector_t::Zero(0);
    }
    
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    
    // Anchor relative pose on the *measured* left EE (same frames as getPosition).
    // Using left *target* breaks under BODY_TRACKING: waist motion makes actual left diverge
    // from the world-fixed left target, so right is pulled to target-relative pose ("above" the
    // real left) while markers still look correct.
    const auto positions = dualArmKinematicsPtr_->getPosition(state);
    const auto [relPosLocal, relQuat] = referenceManagerPtr_->getCouplingParameters(time);

    quaternion_t leftQuat = eeQuatFromKinematicsFrame(preCompMM.getPinocchioInterface(),
                                                      dualArmKinematicsPtr_->getIds()[0]);
    leftQuat.normalize();

    const vector3_t targetRelPos = leftQuat * relPosLocal;
    const quaternion_t rightTargetQuat = (leftQuat * relQuat).normalized();

    vector_t constraint(6);
    constraint.head<3>() = (positions[1] - positions[0]) - targetRelPos;

    const quaternion_t rightQuat = eeQuatFromKinematicsFrame(
        preCompMM.getPinocchioInterface(), dualArmKinematicsPtr_->getIds()[1]);
    constraint.tail<3>() = computeOrientationError(rightQuat.normalized(), rightTargetQuat);
    
    return constraint;
}

VectorFunctionLinearApproximation BimanualCouplingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state,
    const PreComputation& preComputation) const {
    
    if (!referenceManagerPtr_->isBimanualCoupled(time)) {
        return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
    
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    
    const auto [relPosLocal, relQuat] = referenceManagerPtr_->getCouplingParameters(time);

    quaternion_t leftQuat = eeQuatFromKinematicsFrame(preCompMM.getPinocchioInterface(),
                                                      dualArmKinematicsPtr_->getIds()[0]);
    leftQuat.normalize();

    const vector3_t targetRelPos = leftQuat * relPosLocal;
    const quaternion_t rightTargetQuat = (leftQuat * relQuat).normalized();
    
    const auto positions = dualArmKinematicsPtr_->getPosition(state);
    const auto spatialJacobians = pinocchioEEKinPtr_->getSpatialJacobian(state);
    quaternion_t rightQuat = eeQuatFromKinematicsFrame(preCompMM.getPinocchioInterface(),
                                                       dualArmKinematicsPtr_->getIds()[1]);
    rightQuat.normalize();

    const auto leftAngularJacobianWorld = spatialJacobians[0].bottomRows<3>();
    const auto rightAngularJacobianWorld = spatialJacobians[1].bottomRows<3>();

    auto approximation = VectorFunctionLinearApproximation(6, state.size(), 0);
    
    approximation.f.head<3>() = positions[1] - positions[0] - targetRelPos;
    // d(R_L r)/dx = -[R_L r]_x J_omega,L, hence the residual
    // p_R - p_L - R_L r receives the positive skew term below.
    approximation.dfdx.topRows<3>() =
        spatialJacobians[1].topRows<3>() - spatialJacobians[0].topRows<3>() +
        skewSymmetricMatrix(targetRelPos) * leftAngularJacobianWorld;
    
    const vector3_t orientationError = computeOrientationError(rightQuat, rightTargetQuat);
    approximation.f.tail<3>() = orientationError;
    approximation.dfdx.bottomRows<3>() =
        rightJacobianInverseSo3(orientationError) * rightQuat.toRotationMatrix().transpose() *
        (rightAngularJacobianWorld - leftAngularJacobianWorld);
    
    return approximation;
}

BimanualCouplingConstraint::vector3_t 
BimanualCouplingConstraint::computeOrientationError(
    const quaternion_t& current,
    const quaternion_t& desired) const {
    // Principal SO(3) logarithm in the desired frame, evaluated directly from
    // the quaternion. Avoid matrix round trips and the generic AD log branches.
    quaternion_t error = desired.conjugate() * current;
    error.normalize();
    if (error.w() < 0.0) {
        error.coeffs() *= -1.0;
    }
    const scalar_t squaredSinHalfAngle = error.vec().squaredNorm();
    if (squaredSinHalfAngle < 1e-8) {
        // 2 asin(s) / s, including its limit at the identity.
        return (2.0 + squaredSinHalfAngle / 3.0 +
                3.0 * squaredSinHalfAngle * squaredSinHalfAngle / 20.0) * error.vec();
    }
    const scalar_t sinHalfAngle = std::sqrt(squaredSinHalfAngle);
    return (2.0 * std::atan2(sinHalfAngle, error.w()) / sinHalfAngle) * error.vec();
}

} // namespace ocs2::whole_body_mpc
