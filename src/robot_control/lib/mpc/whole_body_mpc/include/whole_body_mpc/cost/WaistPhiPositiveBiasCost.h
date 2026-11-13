#pragma once

#include <array>

#include <ocs2_core/cost/StateCost.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * @brief Soft state cost that discourages waist pitch from going below a threshold.
 *
 * For three-joint waist, planar phi uses the same definition as
 * WaistLiftingPlanner::threeLinkPlannerFK:
 *   planarPhi = sum(state[waistStateIndices[i]] * rotationDirection[i] + angleOffset[i])
 *
 * The constrained value is flipped by body yaw half-plane:
 *   yawSign = cos(yaw) >= 0 ? +1 : -1
 *   effectivePhi = planarPhi * yawSign
 */
class WaistPhiPositiveBiasCost final : public StateCost {
public:
    enum class WaistType {
        THREE_JOINT,
        SINGLE_JOINT
    };

    using index_array_t = std::array<size_t, 3>;
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

    WaistPhiPositiveBiasCost(const SwitchedHumanoidReferenceManager& referenceManager,
                             index_array_t waistStateIndices,
                             vector3_t rotationDirection,
                             vector3_t angleOffset,
                             size_t yawStateIndex,
                             scalar_t yawOffset,
                             scalar_t lowerBoundRad,
                             scalar_t mu);

    WaistPhiPositiveBiasCost(const SwitchedHumanoidReferenceManager& referenceManager,
                             size_t pitchStateIndex,
                             scalar_t pitchDirection,
                             scalar_t pitchOffset,
                             scalar_t lowerBoundRad,
                             scalar_t mu);

    WaistPhiPositiveBiasCost* clone() const override {
        return new WaistPhiPositiveBiasCost(*this);
    }

    scalar_t getValue(scalar_t time, const vector_t& state,
                      const TargetTrajectories& targetTrajectories,
                      const PreComputation& preComputation) const override;

    ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        scalar_t time, const vector_t& state,
        const TargetTrajectories& targetTrajectories,
        const PreComputation& preComputation) const override;

private:
    WaistPhiPositiveBiasCost(const WaistPhiPositiveBiasCost& other) = default;

    scalar_t computePhi(const vector_t& state) const;
    scalar_t computePlanarPhi(const vector_t& state) const;
    scalar_t computeYaw(const vector_t& state) const;
    scalar_t computeYawSign(const vector_t& state) const;
    void validateStateSize(const vector_t& state) const;
    void addPhiQuadratic(ScalarFunctionQuadraticApproximation& approximation,
                         const vector_t& state,
                         scalar_t diff,
                         scalar_t weight) const;

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    WaistType waistType_{WaistType::THREE_JOINT};
    index_array_t waistStateIndices_;
    vector3_t rotationDirection_;
    vector3_t angleOffset_;
    size_t yawStateIndex_{0};
    scalar_t yawOffset_{0.0};
    size_t pitchStateIndex_{0};
    scalar_t pitchDirection_{1.0};
    scalar_t pitchOffset_{0.0};
    scalar_t lowerBoundRad_;
    scalar_t mu_;
};

} // namespace ocs2::whole_body_mpc
