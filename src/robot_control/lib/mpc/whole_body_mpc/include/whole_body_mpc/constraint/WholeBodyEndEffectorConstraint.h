/******************************************************************************
 * Dual-arm end-effector tracking: left is master when bimanual COUPLED;
 * right-hand Cartesian tracking is disabled (zero residual / zero Jacobian rows)
 * so BimanualCouplingConstraint defines the follower arm.
 ******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <cmath>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * A position given relative to the base (the state starts with the base x, y, yaw) in the world frame, and its
 * derivative with respect to those three entries (columns x, y, yaw).
 */
struct BodyFrameTarget {
    Eigen::Matrix<scalar_t, 3, 1> world;
    Eigen::Matrix<scalar_t, 3, 3> baseJacobian;
};

inline BodyFrameTarget bodyFrameTargetPosition(scalar_t x, scalar_t y, scalar_t yaw,
                                               const Eigen::Matrix<scalar_t, 3, 1>& target) {
    const scalar_t c = std::cos(yaw);
    const scalar_t s = std::sin(yaw);
    BodyFrameTarget result;
    result.world << c * target.x() - s * target.y() + x, s * target.x() + c * target.y() + y, target.z();
    result.baseJacobian << 1.0, 0.0, -s * target.x() - c * target.y(),
                           0.0, 1.0, c * target.x() - s * target.y(),
                           0.0, 0.0, 0.0;
    return result;
}

/**
 * Same residual dimension as mobile_manipulator::EndEffectorConstraint (12 for dual arm) so
 * existing penalty vectors in WholeBodyInterface stay valid. In COUPLED mode the last
 * six components are held at zero with zero linearization so only the left arm tracks the
 * reference trajectory; relative pose is left to BimanualCouplingConstraint.
 *
 * With a mobile base (baseStateDim = 3: x, y, yaw lead the state) the targets are given relative to the base, so a
 * hand stays where the operator's hand puts it relative to the body while the base drives. The target is moved to the
 * world frame with the base pose of the state at hand, which makes the residual independent of the base; its
 * linearization follows (position: the base columns lose the target's own derivative; orientation: no base columns).
 */
class WholeBodyEndEffectorConstraint final : public StateConstraint {
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;

    WholeBodyEndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                       const SwitchedHumanoidReferenceManager& referenceManager,
                                       size_t baseStateDim = 0);

    ~WholeBodyEndEffectorConstraint() override = default;

    WholeBodyEndEffectorConstraint* clone() const override {
        return new WholeBodyEndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_, baseStateDim_);
    }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const PreComputation& preComputation) const override;

private:
    /// A target pose in the world frame (first = position, second = orientation) and the derivative of the position
    /// with respect to the base x, y, yaw (zero without a mobile base).
    struct WorldPose {
        vector_t first;
        quaternion_t second;
        Eigen::Matrix<scalar_t, 3, 3> baseJacobian;
    };
    [[nodiscard]] bool bodyFrame() const { return baseStateDim_ >= 3; }
    [[nodiscard]] WorldPose toWorld(const vector_t& state, const std::pair<vector_t, quaternion_t>& pose) const;

    std::pair<vector_t, quaternion_t> interpolateLeftArmPose(scalar_t time) const;
    std::pair<vector_t, quaternion_t> interpolateRightArmPose(scalar_t time) const;

    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_{nullptr};
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_{nullptr};
    size_t baseStateDim_{0};
};

} // namespace ocs2::whole_body_mpc
