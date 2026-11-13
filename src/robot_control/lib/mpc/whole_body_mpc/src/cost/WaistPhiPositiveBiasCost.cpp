#include "whole_body_mpc/cost/WaistPhiPositiveBiasCost.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2::whole_body_mpc {

WaistPhiPositiveBiasCost::WaistPhiPositiveBiasCost(
    const SwitchedHumanoidReferenceManager& referenceManager,
    index_array_t waistStateIndices,
    vector3_t rotationDirection,
    vector3_t angleOffset,
    size_t yawStateIndex,
    scalar_t yawOffset,
    scalar_t lowerBoundRad,
    scalar_t mu)
    : referenceManagerPtr_(&referenceManager),
      waistType_(WaistType::THREE_JOINT),
      waistStateIndices_(waistStateIndices),
      rotationDirection_(std::move(rotationDirection)),
      angleOffset_(std::move(angleOffset)),
      yawStateIndex_(yawStateIndex),
      yawOffset_(yawOffset),
      lowerBoundRad_(lowerBoundRad),
      mu_(mu) {
    if (mu_ < 0.0) {
        throw std::invalid_argument("[WaistPhiPositiveBiasCost] mu must be non-negative");
    }
}

WaistPhiPositiveBiasCost::WaistPhiPositiveBiasCost(
    const SwitchedHumanoidReferenceManager& referenceManager,
    size_t pitchStateIndex,
    scalar_t pitchDirection,
    scalar_t pitchOffset,
    scalar_t lowerBoundRad,
    scalar_t mu)
    : referenceManagerPtr_(&referenceManager),
      waistType_(WaistType::SINGLE_JOINT),
      pitchStateIndex_(pitchStateIndex),
      pitchDirection_(pitchDirection),
      pitchOffset_(pitchOffset),
      lowerBoundRad_(lowerBoundRad),
      mu_(mu) {
    if (mu_ < 0.0) {
        throw std::invalid_argument("[WaistPhiPositiveBiasCost] mu must be non-negative");
    }
}

scalar_t WaistPhiPositiveBiasCost::getValue(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::CUSTOM_LOCKED) {
        return 0.0;
    }

    validateStateSize(state);

    const scalar_t phi = computePhi(state);
    const scalar_t diff = phi - lowerBoundRad_;
    if (diff >= 0.0) {
        return 0.0;
    }
    return 0.5 * mu_ * diff * diff;
}

ScalarFunctionQuadraticApproximation WaistPhiPositiveBiasCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const {
    if (referenceManagerPtr_->getBodyMode(time) != BodyConstraintMode::CUSTOM_LOCKED) {
        return ScalarFunctionQuadraticApproximation::Zero(state.size());
    }

    validateStateSize(state);

    auto approximation = ScalarFunctionQuadraticApproximation::Zero(state.size());
    const scalar_t phi = computePhi(state);
    const scalar_t diff = phi - lowerBoundRad_;
    if (diff < 0.0 && mu_ > 0.0) {
        addPhiQuadratic(approximation, state, diff, mu_);
    }
    return approximation;
}

void WaistPhiPositiveBiasCost::addPhiQuadratic(
    ScalarFunctionQuadraticApproximation& approximation,
    const vector_t& state,
    scalar_t diff,
    scalar_t weight) const {
    approximation.f += 0.5 * weight * diff * diff;

    if (waistType_ == WaistType::SINGLE_JOINT) {
        const auto row = static_cast<Eigen::Index>(pitchStateIndex_);
        const scalar_t dPhi = pitchDirection_;
        approximation.dfdx(row) += weight * diff * dPhi;
        approximation.dfdxx(row, row) += weight * dPhi * dPhi;
        return;
    }

    const scalar_t yawSign = computeYawSign(state);
    for (size_t i = 0; i < waistStateIndices_.size(); ++i) {
        const auto row = static_cast<Eigen::Index>(waistStateIndices_[i]);
        const scalar_t dPhi_i =
            rotationDirection_(static_cast<Eigen::Index>(i)) * yawSign;
        approximation.dfdx(row) += weight * diff * dPhi_i;
        for (size_t j = 0; j < waistStateIndices_.size(); ++j) {
            const auto col = static_cast<Eigen::Index>(waistStateIndices_[j]);
            approximation.dfdxx(row, col) +=
                weight * dPhi_i * rotationDirection_(static_cast<Eigen::Index>(j)) * yawSign;
        }
    }
}

scalar_t WaistPhiPositiveBiasCost::computePhi(const vector_t& state) const {
    if (waistType_ == WaistType::SINGLE_JOINT) {
        return state(static_cast<Eigen::Index>(pitchStateIndex_)) * pitchDirection_ + pitchOffset_;
    }

    return computePlanarPhi(state) * computeYawSign(state);
}

scalar_t WaistPhiPositiveBiasCost::computePlanarPhi(const vector_t& state) const {
    scalar_t phi = 0.0;
    for (size_t i = 0; i < waistStateIndices_.size(); ++i) {
        const auto stateIndex = static_cast<Eigen::Index>(waistStateIndices_[i]);
        const auto vectorIndex = static_cast<Eigen::Index>(i);
        phi += state(stateIndex) * rotationDirection_(vectorIndex) + angleOffset_(vectorIndex);
    }
    return phi;
}

scalar_t WaistPhiPositiveBiasCost::computeYaw(const vector_t& state) const {
    return state(static_cast<Eigen::Index>(yawStateIndex_)) + yawOffset_;
}

scalar_t WaistPhiPositiveBiasCost::computeYawSign(const vector_t& state) const {
    return std::cos(computeYaw(state)) >= 0.0 ? 1.0 : -1.0;
}

void WaistPhiPositiveBiasCost::validateStateSize(const vector_t& state) const {
    if (waistType_ == WaistType::SINGLE_JOINT) {
        if (pitchStateIndex_ >= static_cast<size_t>(state.size())) {
            throw std::out_of_range("[WaistPhiPositiveBiasCost] pitch state index is outside state dimension");
        }
        return;
    }

    for (const auto stateIndex : waistStateIndices_) {
        if (stateIndex >= static_cast<size_t>(state.size())) {
            throw std::out_of_range("[WaistPhiPositiveBiasCost] waist state index is outside state dimension");
        }
    }
    if (yawStateIndex_ >= static_cast<size_t>(state.size())) {
        throw std::out_of_range("[WaistPhiPositiveBiasCost] yaw state index is outside state dimension");
    }
}

} // namespace ocs2::whole_body_mpc
