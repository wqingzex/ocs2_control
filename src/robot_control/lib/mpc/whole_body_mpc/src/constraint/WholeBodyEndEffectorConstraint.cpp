/******************************************************************************
 * Wheel humanoid dual-arm EE soft constraint (left master when coupled)
 ******************************************************************************/

#include "whole_body_mpc/constraint/WholeBodyEndEffectorConstraint.h"

#include <cmath>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

namespace ocs2::whole_body_mpc {

using mobile_manipulator::MobileManipulatorPreComputation;

WholeBodyEndEffectorConstraint::WholeBodyEndEffectorConstraint(
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    const SwitchedHumanoidReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager) {
    if (endEffectorKinematics.getIds().size() != 2) {
        throw std::runtime_error(
            "[WholeBodyEndEffectorConstraint] Expected exactly 2 end effectors, got " +
            std::to_string(endEffectorKinematics.getIds().size()));
    }
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

size_t WholeBodyEndEffectorConstraint::getNumConstraints(scalar_t /*time*/) const {
    return 12;
}

vector_t WholeBodyEndEffectorConstraint::getValue(scalar_t time, const vector_t& state,
                                                     const PreComputation& preComputation) const {
    const bool leftOn = referenceManagerPtr_->isLeftArmActive(time);
    const bool rightOn = referenceManagerPtr_->isRightArmActive(time);
    const bool coupled = referenceManagerPtr_->isBimanualCoupled(time);

    // Fast path: both arms inactive, or coupled but one arm inactive -> EE soft term fully disabled.
    if ((!leftOn && !rightOn) || (coupled && (!leftOn || !rightOn))) {
        return vector_t::Zero(12);
    }

    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto leftArmPose = interpolateLeftArmPose(time);
    const auto positions = endEffectorKinematicsPtr_->getPosition(state);

    vector_t constraint(12);

    if (coupled) {
        const scalar_t kCoupledLeftScale = std::sqrt(static_cast<scalar_t>(2.0));  // equivalent to doubling left-arm weight
        const auto orientationErrors =
            endEffectorKinematicsPtr_->getOrientationError(state, {leftArmPose.second, leftArmPose.second});
        constraint.head<3>() = kCoupledLeftScale * (positions[0] - leftArmPose.first);
        constraint.segment<3>(3) = kCoupledLeftScale * orientationErrors[0];
        constraint.tail<6>().setZero();
        return constraint;
    }

    const auto rightArmPose = interpolateRightArmPose(time);
    const auto orientationErrors =
        endEffectorKinematicsPtr_->getOrientationError(state, {leftArmPose.second, rightArmPose.second});
    constraint.head<3>() = positions[0] - leftArmPose.first;
    constraint.segment<3>(3) = orientationErrors[0];
    constraint.segment<3>(6) = positions[1] - rightArmPose.first;
    constraint.tail<3>() = orientationErrors[1];
    if (!leftOn) {
        constraint.head<6>().setZero();
    }
    if (!rightOn) {
        constraint.tail<6>().setZero();
    }
    return constraint;
}

VectorFunctionLinearApproximation WholeBodyEndEffectorConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    const bool leftOn = referenceManagerPtr_->isLeftArmActive(time);
    const bool rightOn = referenceManagerPtr_->isRightArmActive(time);
    const bool coupled = referenceManagerPtr_->isBimanualCoupled(time);

    auto approximation = VectorFunctionLinearApproximation(12, state.rows(), 0);

    // Fast path: both arms inactive, or coupled but one arm inactive -> EE soft term fully disabled.
    if ((!leftOn && !rightOn) || (coupled && (!leftOn || !rightOn))) {
        approximation.f.setZero();
        approximation.dfdx.setZero();
        return approximation;
    }

    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto leftArmPose = interpolateLeftArmPose(time);
    const auto positions = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);

    if (coupled) {
        const scalar_t kCoupledLeftScale = std::sqrt(static_cast<scalar_t>(2.0));  // equivalent to doubling left-arm weight
        const auto orientationErrors =
            endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {leftArmPose.second, leftArmPose.second});
        approximation.f.head<3>() = kCoupledLeftScale * (positions[0].f - leftArmPose.first);
        approximation.dfdx.topRows<3>() = kCoupledLeftScale * positions[0].dfdx;
        approximation.f.segment<3>(3) = kCoupledLeftScale * orientationErrors[0].f;
        approximation.dfdx.middleRows<3>(3) = kCoupledLeftScale * orientationErrors[0].dfdx;
        approximation.f.tail<6>().setZero();
        approximation.dfdx.bottomRows<6>().setZero();
        return approximation;
    }

    const auto rightArmPose = interpolateRightArmPose(time);
    const auto orientationErrors =
        endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {leftArmPose.second, rightArmPose.second});
    approximation.f.head<3>() = positions[0].f - leftArmPose.first;
    approximation.dfdx.topRows<3>() = positions[0].dfdx;
    approximation.f.segment<3>(3) = orientationErrors[0].f;
    approximation.dfdx.middleRows<3>(3) = orientationErrors[0].dfdx;
    approximation.f.segment<3>(6) = positions[1].f - rightArmPose.first;
    approximation.dfdx.middleRows<3>(6) = positions[1].dfdx;
    approximation.f.tail<3>() = orientationErrors[1].f;
    approximation.dfdx.bottomRows<3>() = orientationErrors[1].dfdx;
    if (!leftOn) {
        approximation.f.head<6>().setZero();
        approximation.dfdx.topRows<6>().setZero();
    }
    if (!rightOn) {
        approximation.f.tail<6>().setZero();
        approximation.dfdx.bottomRows<6>().setZero();
    }
    return approximation;
}

std::pair<vector_t, WholeBodyEndEffectorConstraint::quaternion_t>
WholeBodyEndEffectorConstraint::interpolateLeftArmPose(scalar_t time) const {
    const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;

    vector_t position;
    quaternion_t orientation;

    if (stateTrajectory.size() > 1) {
        int index;
        scalar_t alpha;
        std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

        const auto& lhs = stateTrajectory[index];
        const auto& rhs = stateTrajectory[index + 1];

        const quaternion_t q_lhs(lhs(6), lhs(3), lhs(4), lhs(5));
        const quaternion_t q_rhs(rhs(6), rhs(3), rhs(4), rhs(5));

        position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
        orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {
        position = stateTrajectory.front().head<3>();
        const auto& s = stateTrajectory.front();
        orientation = quaternion_t(s(6), s(3), s(4), s(5));
    }

    return {position, orientation};
}

std::pair<vector_t, WholeBodyEndEffectorConstraint::quaternion_t>
WholeBodyEndEffectorConstraint::interpolateRightArmPose(scalar_t time) const {
    const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;

    vector_t position;
    quaternion_t orientation;

    if (stateTrajectory.size() > 1) {
        int index;
        scalar_t alpha;
        std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

        const auto& lhs = stateTrajectory[index];
        const auto& rhs = stateTrajectory[index + 1];

        const quaternion_t q_lhs(lhs(13), lhs(10), lhs(11), lhs(12));
        const quaternion_t q_rhs(rhs(13), rhs(10), rhs(11), rhs(12));

        position = alpha * lhs.segment<3>(7) + (1.0 - alpha) * rhs.segment<3>(7);
        orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {
        position = stateTrajectory.front().segment<3>(7);
        const auto& s = stateTrajectory.front();
        orientation = quaternion_t(s(13), s(10), s(11), s(12));
    }

    return {position, orientation};
}

} // namespace ocs2::whole_body_mpc
