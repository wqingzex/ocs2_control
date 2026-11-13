#include "whole_body_mpc/constraint/HeadMidpointGazeConstraint.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2::whole_body_mpc {

using mobile_manipulator::MobileManipulatorPreComputation;

namespace {

scalar_t handsHorizonResidual(const Eigen::Vector3d& dL, const Eigen::Vector3d& dR) {
    const scalar_t zL = std::max(dL.z(), HeadMidpointGazeConstraint::kEpsilon);
    const scalar_t zR = std::max(dR.z(), HeadMidpointGazeConstraint::kEpsilon);
    const scalar_t du = dR.x() / zR - dL.x() / zL;
    const scalar_t dv = dR.y() / zR - dL.y() / zL;
    return std::atan2(dv, du);
}

}  // namespace

//=============================================================================
// Analytical path
//=============================================================================

HeadMidpointGazeConstraint::HeadMidpointGazeConstraint(
    const EndEffectorKinematics<scalar_t>& gazeKinematics, const PinocchioInterface& pinocchioInterface,
    const PinocchioStateInputMapping<scalar_t>& mapping,
    const SwitchedHumanoidReferenceManager& referenceManager, CameraIntrinsics intrinsics,
    std::string cameraFrame, size_t stateDim, size_t inputDim, size_t headInputStart, size_t headInputDof,
    bool enableHandsHorizon, bool enableUpright)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      intrinsics_(intrinsics),
      enableHandsHorizon_(enableHandsHorizon),
      enableUpright_(enableUpright),
      gazeKinematicsPtr_(gazeKinematics.clone()),
      mappingPtr_(mapping.clone()),
      cameraFrame_(std::move(cameraFrame)) {
    validateFourthResidualFlags();
    configureHeadStateWindow(stateDim, inputDim, headInputStart, headInputDof);
    if (gazeKinematics.getIds().size() != 3) {
        throw std::runtime_error(
            "[HeadMidpointGazeConstraint] Expected 3 frames (left, right, camera), got " +
            std::to_string(gazeKinematics.getIds().size()));
    }
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(gazeKinematicsPtr_.get());
    if (pinocchioEEKinPtr_ == nullptr) {
        throw std::runtime_error(
            "[HeadMidpointGazeConstraint] Analytical constructor requires PinocchioEndEffectorKinematics "
            "(model_settings.usePreComputation=true).");
    }
    for (size_t i = 0; i < frameIds_.size(); ++i) {
        frameIds_[i] = pinocchioInterface.getModel().getBodyId(gazeKinematics.getIds()[i]);
    }
    cameraFrameId_ = pinocchioInterface.getModel().getBodyId(cameraFrame_);
    if (!std::isfinite(intrinsics_.horizontalDeadbandDeg) || intrinsics_.horizontalDeadbandDeg < 0.0 ||
        intrinsics_.horizontalDeadbandDeg >= 90.0) {
        throw std::invalid_argument("horizontalDeadbandDeg must be finite and in [0, 90).");
    }
    if (!std::isfinite(intrinsics_.verticalDeadbandDeg) || intrinsics_.verticalDeadbandDeg < 0.0 ||
        intrinsics_.verticalDeadbandDeg >= 90.0) {
        throw std::invalid_argument("verticalDeadbandDeg must be finite and in [0, 90).");
    }
    if (enableUpright_ && (!std::isfinite(intrinsics_.uprightDeadbandDeg) ||
                           intrinsics_.uprightDeadbandDeg < 0.0 ||
                           intrinsics_.uprightDeadbandDeg >= 45.0)) {
        throw std::invalid_argument("uprightDeadbandDeg must be finite and in [0, 45).");
    }
    a_ = ((static_cast<scalar_t>(intrinsics_.width) - 1.0) * 0.5 - intrinsics_.cx) / intrinsics_.fx;
    b_ = ((static_cast<scalar_t>(intrinsics_.height) - 1.0) * 0.5 - intrinsics_.cy) / intrinsics_.fy;
}

//=============================================================================
// CppAD path
//=============================================================================

HeadMidpointGazeConstraint::HeadMidpointGazeConstraint(
    const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const SwitchedHumanoidReferenceManager& referenceManager, CameraIntrinsics intrinsics,
    std::string leftFrame, std::string rightFrame, std::string cameraFrame, size_t stateDim, size_t inputDim,
    size_t headInputStart, size_t headInputDof, const std::string& modelName, const std::string& modelFolder,
    bool recompileLibraries, bool enableHandsHorizon, bool enableUpright)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      intrinsics_(intrinsics),
      enableHandsHorizon_(enableHandsHorizon),
      enableUpright_(enableUpright) {
    validateFourthResidualFlags();
    configureHeadStateWindow(stateDim, inputDim, headInputStart, headInputDof);
    if (!std::isfinite(intrinsics_.horizontalDeadbandDeg) || intrinsics_.horizontalDeadbandDeg < 0.0 ||
        intrinsics_.horizontalDeadbandDeg >= 90.0) {
        throw std::invalid_argument("horizontalDeadbandDeg must be finite and in [0, 90).");
    }
    if (!std::isfinite(intrinsics_.verticalDeadbandDeg) || intrinsics_.verticalDeadbandDeg < 0.0 ||
        intrinsics_.verticalDeadbandDeg >= 90.0) {
        throw std::invalid_argument("verticalDeadbandDeg must be finite and in [0, 90).");
    }
    if (enableUpright_ && (!std::isfinite(intrinsics_.uprightDeadbandDeg) ||
                           intrinsics_.uprightDeadbandDeg < 0.0 ||
                           intrinsics_.uprightDeadbandDeg >= 45.0)) {
        throw std::invalid_argument("uprightDeadbandDeg must be finite and in [0, 45).");
    }
    a_ = ((static_cast<scalar_t>(intrinsics_.width) - 1.0) * 0.5 - intrinsics_.cx) / intrinsics_.fx;
    b_ = ((static_cast<scalar_t>(intrinsics_.height) - 1.0) * 0.5 - intrinsics_.cy) / intrinsics_.fy;

    const auto leftId = pinocchioInterface.getModel().getBodyId(leftFrame);
    const auto rightId = pinocchioInterface.getModel().getBodyId(rightFrame);
    const auto camId = pinocchioInterface.getModel().getBodyId(cameraFrame);

    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
    std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
    mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

    const bool horizon = enableHandsHorizon_;
    const bool upright = enableUpright_;
    auto gazeFunc = [&, this, leftId, rightId, camId, horizon, upright](const ad_vector_t& x, ad_vector_t& y) {
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mappingPtr->getPinocchioJointPosition(x);
        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        const Eigen::Matrix<ad_scalar_t, 3, 1> pLeft = data.oMf[leftId].translation();
        const Eigen::Matrix<ad_scalar_t, 3, 1> pRight = data.oMf[rightId].translation();
        const Eigen::Matrix<ad_scalar_t, 3, 1> pCam = data.oMf[camId].translation();
        const Eigen::Matrix<ad_scalar_t, 3, 3> R = data.oMf[camId].rotation();

        const Eigen::Matrix<ad_scalar_t, 3, 1> m = ad_scalar_t(0.5) * (pLeft + pRight);
        const Eigen::Matrix<ad_scalar_t, 3, 1> v = m - pCam;
        const Eigen::Matrix<ad_scalar_t, 3, 1> d = R.transpose() * v;
        const ad_scalar_t dx = d(0);
        const ad_scalar_t dy = d(1);
        const ad_scalar_t dz = d(2);

        y.resize((horizon || upright) ? 4 : 3);
        const ad_scalar_t safeZ = CppAD::CondExpGt(dz, ad_scalar_t(kEpsilon), dz, ad_scalar_t(kEpsilon));
        const ad_scalar_t angle = atan2(dx, safeZ) - ad_scalar_t(std::atan(a_));
        const ad_scalar_t excess = abs(angle) - ad_scalar_t(intrinsics_.horizontalDeadbandDeg * std::acos(-1.0) / 180.0);
        y(0) = CppAD::CondExpGt(excess, ad_scalar_t(0.0), excess, ad_scalar_t(0.0));
        const ad_scalar_t verticalAngle = atan2(dy, safeZ) - ad_scalar_t(std::atan(b_));
        const ad_scalar_t verticalExcess = abs(verticalAngle) - ad_scalar_t(intrinsics_.verticalDeadbandDeg * std::acos(-1.0) / 180.0);
        y(1) = CppAD::CondExpGt(verticalExcess, ad_scalar_t(0.0), verticalExcess, ad_scalar_t(0.0));
        const ad_scalar_t depthMargin = ad_scalar_t(intrinsics_.minDepth) - dz;
        y(2) = CppAD::CondExpGt(depthMargin, ad_scalar_t(0.0), depthMargin, ad_scalar_t(0.0));
        if (horizon) {
            const Eigen::Matrix<ad_scalar_t, 3, 1> dL = R.transpose() * (pLeft - pCam);
            const Eigen::Matrix<ad_scalar_t, 3, 1> dR = R.transpose() * (pRight - pCam);
            const ad_scalar_t zL = CppAD::CondExpGt(dL(2), ad_scalar_t(kEpsilon), dL(2), ad_scalar_t(kEpsilon));
            const ad_scalar_t zR = CppAD::CondExpGt(dR(2), ad_scalar_t(kEpsilon), dR(2), ad_scalar_t(kEpsilon));
            const ad_scalar_t du = dR(0) / zR - dL(0) / zL;
            const ad_scalar_t dv = dR(1) / zR - dL(1) / zL;
            y(3) = atan2(dv, du);
        } else if (upright) {
            const ad_scalar_t r20 = R(2, 0);
            const ad_scalar_t uprightExcess =
                abs(r20) - ad_scalar_t(std::sin(intrinsics_.uprightDeadbandDeg * std::acos(-1.0) / 180.0));
            y(3) = CppAD::CondExpGt(uprightExcess, ad_scalar_t(0.0), uprightExcess, ad_scalar_t(0.0));
        }
    };

    // Encode the exact parameter bits using identifier-safe decimal digits for the AD cache.
    std::uint64_t deadbandBits;
    std::uint64_t verticalDeadbandBits;
    static_assert(sizeof(deadbandBits) == sizeof(intrinsics_.horizontalDeadbandDeg));
    std::memcpy(&deadbandBits, &intrinsics_.horizontalDeadbandDeg, sizeof(deadbandBits));
    std::memcpy(&verticalDeadbandBits, &intrinsics_.verticalDeadbandDeg, sizeof(verticalDeadbandBits));
    std::string cppAdName = modelName + "_angular_deadband_v2_" + std::to_string(deadbandBits) + "_" +
                            std::to_string(verticalDeadbandBits);
    if (enableHandsHorizon_) {
        cppAdName += "_hands_horizon_v1";
    } else if (enableUpright_) {
        cppAdName += "_upright_v1";
        std::uint64_t uprightDeadbandBits;
        static_assert(sizeof(uprightDeadbandBits) == sizeof(intrinsics_.uprightDeadbandDeg));
        std::memcpy(&uprightDeadbandBits, &intrinsics_.uprightDeadbandDeg, sizeof(uprightDeadbandBits));
        cppAdName += "_" + std::to_string(uprightDeadbandBits);
    }
    cppAdInterfacePtr_ = std::make_unique<CppAdInterface>(gazeFunc, stateDim, cppAdName, modelFolder);
    if (recompileLibraries) {
        cppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, false);
    } else {
        cppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, false);
    }
}

//=============================================================================
// Copy
//=============================================================================

HeadMidpointGazeConstraint::HeadMidpointGazeConstraint(const HeadMidpointGazeConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      intrinsics_(rhs.intrinsics_),
      a_(rhs.a_),
      b_(rhs.b_),
      enableHandsHorizon_(rhs.enableHandsHorizon_),
      enableUpright_(rhs.enableUpright_),
      headStateStart_(rhs.headStateStart_),
      headStateDof_(rhs.headStateDof_),
      frameIds_(rhs.frameIds_),
      cameraFrame_(rhs.cameraFrame_),
      cameraFrameId_(rhs.cameraFrameId_) {
    if (rhs.mappingPtr_) mappingPtr_.reset(rhs.mappingPtr_->clone());
    if (rhs.gazeKinematicsPtr_) {
        gazeKinematicsPtr_.reset(rhs.gazeKinematicsPtr_->clone());
        pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(gazeKinematicsPtr_.get());
    }
    if (rhs.cppAdInterfacePtr_) {
        cppAdInterfacePtr_ = std::make_unique<CppAdInterface>(*rhs.cppAdInterfacePtr_);
    }
}

//=============================================================================
// Shared residual math (analytical value path)
//=============================================================================

vector_t HeadMidpointGazeConstraint::computeResidual(const Eigen::Vector3d& pLeft, const Eigen::Vector3d& pRight,
                                                     const Eigen::Vector3d& pCam, const Eigen::Matrix3d& R) const {
    const Eigen::Vector3d m = 0.5 * (pLeft + pRight);
    const Eigen::Vector3d v = m - pCam;
    const Eigen::Vector3d d = R.transpose() * v;
    const scalar_t x = d.x();
    const scalar_t y = d.y();
    const scalar_t z = d.z();

    vector_t r(residualDim());
    const scalar_t angle = std::atan2(x, std::max(z, kEpsilon)) - std::atan(a_);
    r(0) = std::max(scalar_t(0.0), std::abs(angle) - intrinsics_.horizontalDeadbandDeg * std::acos(-1.0) / 180.0);
    const scalar_t verticalAngle = std::atan2(y, std::max(z, kEpsilon)) - std::atan(b_);
    r(1) = std::max(scalar_t(0.0), std::abs(verticalAngle) - intrinsics_.verticalDeadbandDeg * std::acos(-1.0) / 180.0);
    r(2) = std::max(scalar_t(0.0), intrinsics_.minDepth - z);
    if (enableHandsHorizon_) {
        r(3) = handsHorizonResidual(R.transpose() * (pLeft - pCam), R.transpose() * (pRight - pCam));
    } else if (enableUpright_) {
        r(3) = std::max(scalar_t(0.0), std::abs(R(2, 0)) - uprightSinDeadband());
    }
    return r;
}

void HeadMidpointGazeConstraint::validateFourthResidualFlags() const {
    if (enableHandsHorizon_ && enableUpright_) {
        throw std::invalid_argument(
            "[HeadMidpointGazeConstraint] muHandsHorizon and muUpright are mutually exclusive");
    }
}

void HeadMidpointGazeConstraint::configureHeadStateWindow(size_t stateDim, size_t inputDim, size_t headInputStart,
                                                          size_t headInputDof) {
    if (headInputDof == 0) {
        throw std::invalid_argument(
            "[HeadMidpointGazeConstraint] headInputDof must be > 0 to mask the gaze Jacobian to head joints");
    }
    if (stateDim < inputDim) {
        throw std::invalid_argument("[HeadMidpointGazeConstraint] stateDim must be >= inputDim");
    }
    if (headInputStart + headInputDof > inputDim) {
        throw std::invalid_argument("[HeadMidpointGazeConstraint] head input window exceeds inputDim");
    }
    // armStartInState = stateDim - armDim; armStartInInput = inputDim - armDim.
    headStateStart_ = stateDim - inputDim + headInputStart;
    headStateDof_ = headInputDof;
    if (headStateStart_ + headStateDof_ > stateDim) {
        throw std::invalid_argument("[HeadMidpointGazeConstraint] head state window exceeds stateDim");
    }
}

void HeadMidpointGazeConstraint::maskNonHeadStateColumns(matrix_t& dfdx) const {
    const auto cols = dfdx.cols();
    const auto start = static_cast<Eigen::Index>(headStateStart_);
    const auto dof = static_cast<Eigen::Index>(headStateDof_);
    if (start > 0) {
        dfdx.leftCols(start).setZero();
    }
    const auto tail = start + dof;
    if (tail < cols) {
        dfdx.rightCols(cols - tail).setZero();
    }
}

scalar_t HeadMidpointGazeConstraint::uprightSinDeadband() const {
    return std::sin(intrinsics_.uprightDeadbandDeg * std::acos(-1.0) / 180.0);
}

void HeadMidpointGazeConstraint::maskGazeRollCoupling(matrix_t& dfdx) const {
    // 2-DoF heads have no dedicated roll joint; keep the last look-at column.
    if (headStateDof_ < 3) {
        return;
    }
    const auto start = static_cast<Eigen::Index>(headStateStart_);
    const auto dof = static_cast<Eigen::Index>(headStateDof_);
    const auto rollCol = start + dof - 1;
    const auto lookAtRows = std::min<Eigen::Index>(3, dfdx.rows());
    dfdx.block(0, rollCol, lookAtRows, 1).setZero();
    if (enableUpright_ && dfdx.rows() >= 4) {
        dfdx.block(3, start, 1, dof - 1).setZero();
    }
}

//=============================================================================
// Constraint interface
//=============================================================================

void HeadMidpointGazeConstraint::updateKinematics(const PreComputation& preComputation) const {
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preComp = cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preComp.getPinocchioInterface());
    }
}

size_t HeadMidpointGazeConstraint::getNumConstraints(scalar_t time) const {
    return referenceManagerPtr_->isHeadMidpointGazeActive(time) ? residualDim() : 0;
}

vector_t HeadMidpointGazeConstraint::getValue(scalar_t time, const vector_t& state,
                                              const PreComputation& preComputation) const {
    if (getNumConstraints(time) == 0) {
        return vector_t::Zero(0);
    }
    if (cppAdInterfacePtr_) {
        return cppAdInterfacePtr_->getFunctionValue(state);
    }
    updateKinematics(preComputation);
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    const Eigen::Matrix3d R = preCompMM.getPinocchioInterface().getData().oMf[cameraFrameId_].rotation();
    const auto positions = gazeKinematicsPtr_->getPosition(state);  // {left, right, camera}
    return computeResidual(positions[0], positions[1], positions[2], R);
}

VectorFunctionLinearApproximation HeadMidpointGazeConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    if (getNumConstraints(time) == 0) {
        return VectorFunctionLinearApproximation(0, state.size(), 0);
    }
    if (cppAdInterfacePtr_) {
        VectorFunctionLinearApproximation approximation(residualDim(), state.size(), 0);
        approximation.f = cppAdInterfacePtr_->getFunctionValue(state);
        approximation.dfdx = cppAdInterfacePtr_->getJacobian(state);
        maskNonHeadStateColumns(approximation.dfdx);
        maskGazeRollCoupling(approximation.dfdx);
        return approximation;
    }

    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    const auto& pinocchioInterface = preCompMM.getPinocchioInterface();
    const auto& model = pinocchioInterface.getModel();
    // Pinocchio's frame Jacobian API takes mutable Data. Share one local copy across all three
    // queries without modifying the solver's precomputation or copying Data for each API call.
    auto data = pinocchioInterface.getData();
    mappingPtr_->setPinocchioInterface(pinocchioInterface);
    const Eigen::Matrix3d R = data.oMf[cameraFrameId_].rotation();
    const Eigen::Vector3d pLeft = data.oMf[frameIds_[0]].translation();
    const Eigen::Vector3d pRight = data.oMf[frameIds_[1]].translation();
    const Eigen::Vector3d pCam = data.oMf[frameIds_[2]].translation();

    matrix_t Jv = matrix_t::Zero(3, state.size());
    matrix_t Jrot;
    matrix_t JleftPos = matrix_t::Zero(3, state.size());
    matrix_t JrightPos = matrix_t::Zero(3, state.size());
    matrix_t JcamPos = matrix_t::Zero(3, state.size());
    matrix_t J = matrix_t::Zero(6, model.nv);
    const matrix_t zeroVelocityJacobian = matrix_t::Zero(6, model.nv);
    for (size_t i = 0; i < frameIds_.size(); ++i) {
        J.setZero();  // getFrameJacobian only fills columns along the frame's ancestor chain.
        pinocchio::getFrameJacobian(model, data, frameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, J);
        const matrix_t Jstate = mappingPtr_->getOcs2Jacobian(state, J, zeroVelocityJacobian).first;
        if (i < 2) {
            Jv += 0.5 * Jstate.topRows<3>();
            if (enableHandsHorizon_) {
                (i == 0 ? JleftPos : JrightPos) = Jstate.topRows<3>();
            }
        } else {
            Jv -= Jstate.topRows<3>();
            Jrot = Jstate.bottomRows<3>();
            if (enableHandsHorizon_) {
                JcamPos = Jstate.topRows<3>();
            }
        }
    }

    const Eigen::Vector3d m = 0.5 * (pLeft + pRight);
    const Eigen::Vector3d v = m - pCam;
    const Eigen::Vector3d d = R.transpose() * v;
    // d(R^T v)/dx = R^T * dv/dx + R^T * skew(v) * J_rot  (see plan doc for the derivation).
    const matrix_t Jd = R.transpose() * Jv + R.transpose() * skewSymmetricMatrix(v) * Jrot;

    const scalar_t x = d.x();
    const scalar_t y = d.y();
    const scalar_t z = d.z();
    const row_vector_t Jx = Jd.row(0);
    const row_vector_t Jy = Jd.row(1);
    const row_vector_t Jz = Jd.row(2);

    const scalar_t safeZ = std::max(z, kEpsilon);
    const scalar_t angle = std::atan2(x, safeZ) - std::atan(a_);
    const scalar_t rAlign0 = std::max(scalar_t(0.0),
        std::abs(angle) - intrinsics_.horizontalDeadbandDeg * std::acos(-1.0) / 180.0);
    const scalar_t verticalAngle = std::atan2(y, safeZ) - std::atan(b_);
    const scalar_t rAlign1 = std::max(scalar_t(0.0),
        std::abs(verticalAngle) - intrinsics_.verticalDeadbandDeg * std::acos(-1.0) / 180.0);
    const scalar_t depthMargin = intrinsics_.minDepth - z;
    const scalar_t rFront = std::max(scalar_t(0.0), depthMargin);

    VectorFunctionLinearApproximation approximation(residualDim(), state.size(), 0);
    approximation.f(0) = rAlign0;
    approximation.f(1) = rAlign1;
    approximation.f(2) = rFront;
    const row_vector_t JsafeZ = z > kEpsilon ? Jz : row_vector_t::Zero(state.size());
    approximation.dfdx.row(0).setZero();
    if (rAlign0 > 0.0) {
        approximation.dfdx.row(0) = (angle > 0.0 ? 1.0 : -1.0) *
            (safeZ * Jx - x * JsafeZ) / (x * x + safeZ * safeZ);
    }
    approximation.dfdx.row(1).setZero();
    if (rAlign1 > 0.0) {
        approximation.dfdx.row(1) = (verticalAngle > 0.0 ? 1.0 : -1.0) *
            (safeZ * Jy - y * JsafeZ) / (y * y + safeZ * safeZ);
    }
    approximation.dfdx.row(2) = (depthMargin > 0.0) ? row_vector_t(-Jz) : row_vector_t::Zero(state.size());

    if (enableHandsHorizon_) {
        const auto imageUvJacobian = [&](const Eigen::Vector3d& p, const matrix_t& Jpos, Eigen::Vector3d& dOut,
                                         row_vector_t& Ju, row_vector_t& Jv) {
            const Eigen::Vector3d rel = p - pCam;
            dOut = R.transpose() * rel;
            const matrix_t JdHand = R.transpose() * (Jpos - JcamPos) + R.transpose() * skewSymmetricMatrix(rel) * Jrot;
            const scalar_t hz = dOut.z();
            const scalar_t safeHz = std::max(hz, kEpsilon);
            const row_vector_t Jhz = JdHand.row(2);
            const row_vector_t JsafeHz = hz > kEpsilon ? Jhz : row_vector_t::Zero(state.size());
            Ju = (safeHz * JdHand.row(0) - dOut.x() * JsafeHz) / (safeHz * safeHz);
            Jv = (safeHz * JdHand.row(1) - dOut.y() * JsafeHz) / (safeHz * safeHz);
        };
        Eigen::Vector3d dL, dR;
        row_vector_t JuL, JvL, JuR, JvR;
        imageUvJacobian(pLeft, JleftPos, dL, JuL, JvL);
        imageUvJacobian(pRight, JrightPos, dR, JuR, JvR);
        const scalar_t zL = std::max(dL.z(), kEpsilon);
        const scalar_t zR = std::max(dR.z(), kEpsilon);
        const scalar_t du = dR.x() / zR - dL.x() / zL;
        const scalar_t dv = dR.y() / zR - dL.y() / zL;
        approximation.f(3) = std::atan2(dv, du);
        const scalar_t denom = du * du + dv * dv;
        if (denom > kEpsilon * kEpsilon) {
            approximation.dfdx.row(3) = (du * (JvR - JvL) - dv * (JuR - JuL)) / denom;
        } else {
            approximation.dfdx.row(3).setZero();
        }
    } else if (enableUpright_) {
        const scalar_t r20 = R(2, 0);
        const scalar_t uprightExcess = std::abs(r20) - uprightSinDeadband();
        approximation.f(3) = std::max(scalar_t(0.0), uprightExcess);
        approximation.dfdx.row(3).setZero();
        if (uprightExcess > 0.0) {
            // d|R(2,0)|/dx = sign(R(2,0)) * (R.col(0) × e_z)^T Jrot
            const Eigen::Vector3d camX = R.col(0);
            approximation.dfdx.row(3) =
                (r20 > 0.0 ? 1.0 : -1.0) * camX.cross(Eigen::Vector3d::UnitZ()).transpose() * Jrot;
        }
    }
    maskNonHeadStateColumns(approximation.dfdx);
    maskGazeRollCoupling(approximation.dfdx);
    return approximation;
}

}  // namespace ocs2::whole_body_mpc
