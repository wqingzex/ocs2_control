#pragma once

#include <array>
#include <memory>
#include <string>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

/**
 * Head midpoint gaze: keeps the projection of the left/right end-effector midpoint at the geometric
 * center of the head camera image, with independent horizontal/vertical angular deadbands. Active only when
 * SwitchedHumanoidReferenceManager::isHeadMidpointGazeActive(time), i.e. HEAD_GAZE head mode
 * with headMidpointGaze.activate=true. Independent of body mode.
 *
 * Geometry (world frame W, camera optical frame C, +X right/+Y down/+Z forward):
 *   m = (p_left + p_right) / 2 ; v = m - p_camera ; d = R_WC^T v = [x, y, z]
 *   angleH = atan2(x, max(z, epsilon)) - atan(a); angleV = atan2(y, max(z, epsilon)) - atan(b)
 *   r_align = [max(0, abs(angleH) - horizontalDeadbandDeg*pi/180),
 *              max(0, abs(angleV) - verticalDeadbandDeg*pi/180)]
 *   r_front = max(0, minDepth - z)
 * Optional 4th residual (exactly one of these; both off keeps dim 3):
 *   enableUpright (muUpright>0): r_upright = max(0, |R_WC(2,0)| - sin(uprightDeadbandDeg))
 *     gravity-level image x; deadband zeros the residual and its Jacobian inside the band.
 *   enableHandsHorizon (muHandsHorizon>0): r_handsHorizon = atan2(vR-vL, uR-uL)  // image LR segment
 *
 * On 3-DoF heads the look-at rows (H/V/depth) drop the roll column; the upright row keeps only roll.
 * where (a, b) map the image's geometric center pixel (not the principal point) through the pinhole
 * model. See docs/plans/2026-09-08-ocs2-head-midpoint-gaze.md (in the .agents repo) for the full
 * derivation.
 *
 * Residual value uses full-chain FK (hands + camera). The state Jacobian is masked to the head
 * joint window so HEAD_GAZE does not command waist/folding/arms; those stay on their own tasks.
 */
class HeadMidpointGazeConstraint final : public StateConstraint {
public:
    struct CameraIntrinsics {
        scalar_t fx{1.0};
        scalar_t fy{1.0};
        scalar_t cx{0.0};
        scalar_t cy{0.0};
        int width{1};
        int height{1};
        scalar_t minDepth{0.0};
        scalar_t horizontalDeadbandDeg{0.0};
        scalar_t verticalDeadbandDeg{0.0};
        scalar_t uprightDeadbandDeg{2.0};
    };

    /**
     * Analytical path (model_settings.usePreComputation=true).
     * @param gazeKinematics Must resolve exactly 3 ids, in order: {leftFrame, rightFrame, cameraFrame}.
     * @param enableHandsHorizon When false (default), no image-horizon residual.
     * @param enableUpright When false (default), no gravity-upright residual. Mutually exclusive with horizon.
     */
    HeadMidpointGazeConstraint(const EndEffectorKinematics<scalar_t>& gazeKinematics,
                               const PinocchioInterface& pinocchioInterface,
                               const PinocchioStateInputMapping<scalar_t>& mapping,
                               const SwitchedHumanoidReferenceManager& referenceManager,
                               CameraIntrinsics intrinsics, std::string cameraFrame,
                               size_t stateDim, size_t inputDim, size_t headInputStart, size_t headInputDof,
                               bool enableHandsHorizon = false, bool enableUpright = false);

    /** CppAD path (model_settings.usePreComputation=false): self-contained AD tape over the full residual. */
    HeadMidpointGazeConstraint(const PinocchioInterface& pinocchioInterface,
                               const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const SwitchedHumanoidReferenceManager& referenceManager,
                               CameraIntrinsics intrinsics, std::string leftFrame, std::string rightFrame,
                               std::string cameraFrame, size_t stateDim, size_t inputDim,
                               size_t headInputStart, size_t headInputDof, const std::string& modelName,
                               const std::string& modelFolder, bool recompileLibraries,
                               bool enableHandsHorizon = false, bool enableUpright = false);

    HeadMidpointGazeConstraint* clone() const override { return new HeadMidpointGazeConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state,
                      const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const override;

    // Fixed numerical-safety constant (avoids division by zero when d≈0); not a tunable config value.
    static constexpr scalar_t kEpsilon = 1e-6;

private:
    HeadMidpointGazeConstraint(const HeadMidpointGazeConstraint& rhs);
    void updateKinematics(const PreComputation& preComputation) const;
    vector_t computeResidual(const Eigen::Vector3d& pLeft, const Eigen::Vector3d& pRight,
                             const Eigen::Vector3d& pCam, const Eigen::Matrix3d& R) const;
    void configureHeadStateWindow(size_t stateDim, size_t inputDim, size_t headInputStart, size_t headInputDof);
    void maskNonHeadStateColumns(matrix_t& dfdx) const;
    void maskGazeRollCoupling(matrix_t& dfdx) const;
    void validateFourthResidualFlags() const;
    scalar_t uprightSinDeadband() const;
    size_t residualDim() const { return (enableHandsHorizon_ || enableUpright_) ? 4 : 3; }

    const SwitchedHumanoidReferenceManager* referenceManagerPtr_;
    CameraIntrinsics intrinsics_;
    scalar_t a_{0.0};  // (u0 - cx) / fx, u0 = geometric center pixel column
    scalar_t b_{0.0};  // (v0 - cy) / fy, v0 = geometric center pixel row
    bool enableHandsHorizon_{false};
    bool enableUpright_{false};
    size_t headStateStart_{0};
    size_t headStateDof_{0};

    // Analytical-path members (null when constructed via the CppAD constructor).
    std::unique_ptr<EndEffectorKinematics<scalar_t>> gazeKinematicsPtr_;
    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_{nullptr};
    std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
    std::array<size_t, 3> frameIds_{};
    std::string cameraFrame_;
    size_t cameraFrameId_{0};

    // CppAD-path member (null when constructed via the analytical constructor).
    std::unique_ptr<CppAdInterface> cppAdInterfacePtr_;
};

}  // namespace ocs2::whole_body_mpc
