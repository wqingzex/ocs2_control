#include "whole_body_mpc/constraint/HeadTrackingConstraint.h"
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <iostream>
#include <utility>

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

namespace ocs2::whole_body_mpc {

using mobile_manipulator::MobileManipulatorPreComputation;

HeadTrackingConstraint::HeadTrackingConstraint(
    const EndEffectorKinematics<scalar_t>& headKinematics,
    const SwitchedHumanoidReferenceManager& referenceManager,
    std::string headLinkName)
    : StateConstraint(ConstraintOrder::Linear),
      headKinematicsPtr_(headKinematics.clone()),
      referenceManagerPtr_(&referenceManager),
      headLinkName_(std::move(headLinkName)) {
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(headKinematicsPtr_.get());
    const auto& ids = headKinematics.getIds();
    if (std::find(ids.begin(), ids.end(), headLinkName_) == ids.end()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("HeadTrackingConstraint"), "Warning: head link '" << headLinkName_
                  << "' not found in kinematics IDs");
    }
}

void HeadTrackingConstraint::updateKinematics(const PreComputation& preComputation) const {
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preComp = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preComp.getPinocchioInterface());
    }
}

size_t HeadTrackingConstraint::getNumConstraints(scalar_t time) const {
    return referenceManagerPtr_->isHeadActive(time) ? 6 : 0;
}

vector_t HeadTrackingConstraint::getValue(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    if (getNumConstraints(time) == 0) {
        return vector_t::Zero(0);
    }
    updateKinematics(preComputation);
    const auto [targetPos, targetQuat] = referenceManagerPtr_->interpolateHeadPose(time);
    const auto positions = headKinematicsPtr_->getPosition(state);
    const auto orientationErrors = headKinematicsPtr_->getOrientationError(state, {targetQuat});
    vector_t residual(6);
    residual.head<3>() = positions[0] - targetPos;
    residual.tail<3>() = orientationErrors[0];
    return residual;
}

VectorFunctionLinearApproximation HeadTrackingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    if (getNumConstraints(time) == 0) {
        return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
    updateKinematics(preComputation);
    const auto [targetPos, targetQuat] = referenceManagerPtr_->interpolateHeadPose(time);
    const auto positionApprox = headKinematicsPtr_->getPositionLinearApproximation(state);
    const auto orientationApprox =
        headKinematicsPtr_->getOrientationErrorLinearApproximation(state, {targetQuat});
    VectorFunctionLinearApproximation approximation(6, state.size(), 0);
    approximation.f.head<3>() = positionApprox[0].f - targetPos;
    approximation.dfdx.topRows<3>() = positionApprox[0].dfdx;
    approximation.f.tail<3>() = orientationApprox[0].f;
    approximation.dfdx.bottomRows<3>() = orientationApprox[0].dfdx;
    return approximation;
}

}  // namespace ocs2::whole_body_mpc
