/******************************************************************************
 * OCS2 Wheel Humanoid - Main Interface Implementation
 ******************************************************************************/

#include "whole_body_mpc/WholeBodyInterface.h"
#include <rclcpp/logging.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <unordered_set>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

// Reuse from mobile_manipulator
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator/cost/QuadraticInputCost.h>
#include <ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h>
#include <ocs2_mobile_manipulator/dynamics/FloatingArmManipulatorDynamics.h>
#include <ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h>
#include <ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorDynamics.h>
#include <ocs2_mobile_manipulator/dynamics/OmniWheelBasedMobileManipulatorDynamics.h>
#include "whole_body_mpc/cost/WaistPhiPositiveBiasCost.h"
#include "whole_body_mpc/constraint/WholeBodyEndEffectorConstraint.h"
#include <ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h>
#include <ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h>

// New constraints
#include "whole_body_mpc/constraint/BimanualCouplingConstraint.h"
#include "whole_body_mpc/constraint/BodyTrackingConstraint.h"
#include "whole_body_mpc/constraint/HeadTrackingConstraint.h"
#include "whole_body_mpc/constraint/HeadMidpointGazeConstraint.h"
#include "whole_body_mpc/constraint/BaseLockConstraint.h"
#include "whole_body_mpc/constraint/WaistJointLockConstraint.h"
#include "whole_body_mpc/constraint/HeadCouplingConstraint.h"
#include "whole_body_mpc/constraint/HeadDefaultLockConstraint.h"
#include "whole_body_mpc/constraint/ArmJointLockConstraint.h"
#include "whole_body_mpc/constraint/CustomJointLockConstraint.h"
#include "whole_body_mpc/constraint/HomeJointReferenceConstraint.h"
#include "whole_body_mpc/mode/HumanoidModeSchedule.h"

#include <sstream>
#include <streambuf>
#include <ostream>
#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2::whole_body_mpc {

namespace {

// The start-up report below is written as a stream, OCS2 style (several statements make up one line). This stream
// cuts what is written to it into lines and sends every line to the ROS logger, so the report lands in the node log
// like every other message instead of going straight to std::cerr. Used while the interface is being constructed,
// from one thread.
class RosLineBuffer : public std::streambuf {
public:
    explicit RosLineBuffer(rclcpp::Logger logger) : logger_(std::move(logger)) {}

protected:
    int_type overflow(int_type character) override {
        if (character == traits_type::eof()) {
            return character;
        }
        if (character == '\n') {
            if (!line_.empty()) {
                RCLCPP_INFO(logger_, "%s", line_.c_str());
                line_.clear();
            }
        } else {
            line_.push_back(static_cast<char>(character));
        }
        return character;
    }

    std::streamsize xsputn(const char* text, std::streamsize count) override {
        for (std::streamsize i = 0; i < count; ++i) {
            overflow(traits_type::to_int_type(text[i]));
        }
        return count;
    }

private:
    rclcpp::Logger logger_;
    std::string line_;
};

std::ostream& reportStream() {
    static RosLineBuffer buffer(rclcpp::get_logger("WholeBodyInterface"));
    static std::ostream stream(&buffer);
    return stream;
}

}  // namespace


using namespace mobile_manipulator;

//=============================================================================
// Constructor
//=============================================================================

WholeBodyInterface::WholeBodyInterface(
    const std::string& taskFile,
    const std::string& libraryFolder,
    const std::string& urdfFile,
    const FrameOverrides& frameOverrides)
    : urdfFile_(urdfFile) {
    
    // Check task file exists
    boost::filesystem::path taskFilePath(taskFile);
    if (!boost::filesystem::exists(taskFilePath)) {
        throw std::invalid_argument(
            "[WholeBodyInterface] Task file not found: " + taskFilePath.string());
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Loading task file: " << taskFilePath);
    
    // Check URDF file exists
    boost::filesystem::path urdfFilePath(urdfFile);
    if (!boost::filesystem::exists(urdfFilePath)) {
        throw std::invalid_argument(
            "[WholeBodyInterface] URDF file not found: " + urdfFilePath.string());
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Loading Pinocchio model from: " << urdfFilePath);
    
    // Create library folder
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Generated library path: " << libraryFolderPath);
    
    // Read task file
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    loadData::loadPtreeValue(pt, bodyTrackingEeEnabled_, "bodyTrackingEE.activate", false);
    loadData::loadPtreeValue(pt, headTrackingEeEnabled_, "headTrackingEE.activate", false);
    loadData::loadPtreeValue(pt, headMidpointGazeEnabled_, "headMidpointGaze.activate", false);
    // Load manipulator model type directly
    // 0 = DefaultManipulator, 1 = WheelBasedMobileManipulator, 4 = OmniWheelBasedMobileManipulator
    ManipulatorModelType modelType = loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Model type: " << modelTypeEnumToString(modelType));
    
    // Load joints to remove
    std::vector<std::string> removeJointNames;
    loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", 
                                          removeJointNames, false);
    
    // Load frame names (aligned with MobileManipulatorInterface)
    std::string baseFrame, eeFrame, eeFrame1, bodyFrame, headFrame;
    loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
    loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
    loadData::loadPtreeValue<std::string>(pt, eeFrame1, "model_information.eeFrame1", false);
    if (const auto configuredBodyFrame = pt.get_optional<std::string>("model_information.bodyFrame")) {
        bodyFrame = *configuredBodyFrame;
    } else if (const auto legacyBodyFrame = pt.get_optional<std::string>("bodyRelative.bodyLinkName")) {
        bodyFrame = *legacyBodyFrame;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Deprecated config key bodyRelative.bodyLinkName; "
                     "use model_information.bodyFrame instead.");
    } else {
        throw std::invalid_argument(
            "[WholeBodyInterface] Missing required model_information.bodyFrame");
    }
    if (const auto configuredHeadFrame = pt.get_optional<std::string>("model_information.headFrame")) {
        headFrame = *configuredHeadFrame;
    }

    if (!frameOverrides.baseFrame.empty()) {
        baseFrame = frameOverrides.baseFrame;
    }
    if (!frameOverrides.eeFrame.empty()) {
        eeFrame = frameOverrides.eeFrame;
    }
    if (!frameOverrides.eeFrame1.empty()) {
        eeFrame1 = frameOverrides.eeFrame1;
    }
    if (!frameOverrides.bodyFrame.empty()) {
        bodyFrame = frameOverrides.bodyFrame;
    }
    if (!frameOverrides.headFrame.empty()) {
        headFrame = frameOverrides.headFrame;
    }
    if (headTrackingEeEnabled_ && headFrame.empty()) {
        throw std::invalid_argument(
            "[WholeBodyInterface] headTrackingEE.activate requires model_information.headFrame or a headFrame override");
    }
    
    reportStream() << "\n #### Wheel Humanoid Model Information:";
    reportStream() << "\n #### =============================================================================\n";
    reportStream() << " #### manipulatorModelType: " << static_cast<int>(modelType) << "\n";
    reportStream() << " #### baseFrame: \"" << baseFrame << "\"\n";
    reportStream() << " #### eeFrame (left):  \"" << eeFrame << "\"\n";
    reportStream() << " #### eeFrame1 (right): \"" << eeFrame1 << "\"\n";
    reportStream() << " #### bodyFrame: \"" << bodyFrame << "\"\n";
    reportStream() << " #### headFrame: \"" << headFrame << "\"\n";
    reportStream() << " #### =============================================================================\n";
    
    // Create Pinocchio interface (reuse from mobile_manipulator)
    pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
        createPinocchioInterface(urdfFile, modelType, removeJointNames));
    reportStream() << *pinocchioInterfacePtr_;

    // Print Pinocchio joint order for debugging
    {
        const auto& model = pinocchioInterfacePtr_->getModel();
        reportStream() << "\n #### Pinocchio Joint Order (nq=" << model.nq << ", nv=" << model.nv << "):\n";
        for (int i = 1; i < model.njoints; ++i) {
            reportStream() << " ####   joint[" << i << "] idx_q=" << model.idx_qs[i]
                      << " idx_v=" << model.idx_vs[i] << "  \"" << model.names[i] << "\"\n";
        }
        reportStream() << " #### =============================================================================\n";
    }

    // Create model info (extend from mobile_manipulator)
    auto baseModelInfo = createManipulatorModelInfo(
        *pinocchioInterfacePtr_, modelType, baseFrame, eeFrame, eeFrame1);
    
    modelInfo_ = WholeBodyModelInfo{};
    static_cast<ManipulatorModelInfo&>(modelInfo_) = baseModelInfo;
    modelInfo_.bodyFrame = bodyFrame;
    modelInfo_.headFrame = headFrame;
    hasMobileBase_ = (modelInfo_.manipulatorModelType == ManipulatorModelType::WheelBasedMobileManipulator ||
                      modelInfo_.manipulatorModelType == ManipulatorModelType::OmniWheelBasedMobileManipulator);
    
    // Load settings
    bool usePreComputation = true;
    bool recompileLibraries = true;
    loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
    loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
    if (frameOverrides.recompileLibraries.has_value()) {
        recompileLibraries = *frameOverrides.recompileLibraries;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "recompileLibraries override: "
                  << (recompileLibraries ? "true" : "false"));
    }
    
    // Load initial state
    initialState_.setZero(modelInfo_.stateDim);
    const int baseStateDim = modelInfo_.stateDim - modelInfo_.armDim;
    const int armStateDim = modelInfo_.armDim;
    
    if (baseStateDim > 0) {
        vector_t initialBaseState = vector_t::Zero(baseStateDim);
        loadData::loadEigenMatrix(taskFile, 
            "initialState.base." + modelTypeEnumToString(modelType), initialBaseState);
        initialState_.head(baseStateDim) = initialBaseState;
    }
    
    vector_t initialArmState = vector_t::Zero(armStateDim);
    loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
    initialState_.tail(armStateDim) = initialArmState;
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Initial State: " << initialState_.transpose());
    
    // DDP-MPC settings
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
    
    // Load initial mode schedule
    ModeSchedule initialModeSchedule = loadHumanoidModeSchedule(
        taskFile, "initialModeSchedule", true);

    // Degrade unavailable gaze before constructing the validating reference manager.
    for (auto& mode : initialModeSchedule.modeSequence) {
        auto hm = HumanoidMode::decode(mode);
        if (hm.headMode == HeadMode::GAZE && !headMidpointGazeEnabled_) {
            hm.headMode = HeadMode::DISABLED;
            mode = hm.encode();
        }
    }

    // Create reference manager with mode schedule support
    referenceManagerPtr_ = std::make_shared<SwitchedHumanoidReferenceManager>(
        TargetTrajectories(), std::move(initialModeSchedule),
        bodyTrackingEeEnabled_, headTrackingEeEnabled_, headMidpointGazeEnabled_);
    
    // Setup optimal control problem (also sets capability flags)
    setupOptimalControlProblem(taskFile, libraryFolder, usePreComputation, recompileLibraries);
    
    // Validate initial mode schedule against available capabilities
    {
        const auto& origSchedule = referenceManagerPtr_->getModeSchedule();
        auto modeSeq = origSchedule.modeSequence;
        bool anyDegraded = false;
        for (auto& mode : modeSeq) {
            auto hm = HumanoidMode::decode(mode);
            bool degraded = false;
            if (!hm.baseLocked && !hasMobileBase_) {
                hm.baseLocked = true; degraded = true;
            }
            if (hm.bodyMode == BodyConstraintMode::TRACKING && !bodyTrackingEeEnabled_) {
                hm.bodyMode = BodyConstraintMode::FREE; degraded = true;
            }
            if (hm.headMode == HeadMode::TRACKING && !headTrackingEeEnabled_) {
                hm.headMode = HeadMode::DISABLED; degraded = true;
            }
            if (hm.bodyMode == BodyConstraintMode::RELATIVE && !hasBodyRelativeConstraint_) {
                hm.bodyMode = BodyConstraintMode::FREE; degraded = true;
            }
            if (hm.bodyMode == BodyConstraintMode::LOCKED && !hasWaistLock_) {
                hm.bodyMode = BodyConstraintMode::FREE; degraded = true;
            }
            if (hm.headMode == HeadMode::FORWARD && !hasHeadCoupling_) {
                hm.headMode = HeadMode::DISABLED; degraded = true;
            }
            if (hm.bodyMode == BodyConstraintMode::CUSTOM_LOCKED && !hasCustomJointLock_) {
                hm.bodyMode = BodyConstraintMode::FREE; degraded = true;
            }
            if (hm.bimanualMode == BimanualMode::COUPLED && !hasBimanualCoupling_) {
                hm.bimanualMode = BimanualMode::INDEPENDENT; degraded = true;
            }
            if (degraded) {
                const size_t newMode = hm.encode();
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Initial mode " << modeToString(mode)
                          << " requires unsupported capabilities; degraded to " << modeToString(newMode));
                mode = newMode;
                anyDegraded = true;
            }
        }
        if (anyDegraded) {
            referenceManagerPtr_->setModeSchedule(ModeSchedule(origSchedule.eventTimes, std::move(modeSeq)));
        }
    }
    
    // Rollout
    const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
    rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);
    
    // Initializer
    initializerPtr_ = std::make_unique<DefaultInitializer>(modelInfo_.inputDim);
}

//=============================================================================
// Setup Optimal Control Problem
//=============================================================================

void WholeBodyInterface::setupOptimalControlProblem(
    const std::string& taskFile,
    const std::string& libraryFolder,
    bool usePreComputation,
    bool recompileLibraries) {
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    // Cost
    problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));
    if (auto waistPhiPositiveBiasCost = getWaistPhiPositiveBiasCost(taskFile)) {
        problem_.stateCostPtr->add("waistPhiPositiveBias", std::move(waistPhiPositiveBiasCost));
    }
    
    // Constraints
    // Joint limits
    problem_.softConstraintPtr->add("jointLimits",
        getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
    
    // End-effector constraint (reuse from mobile_manipulator, always dual-arm)
    problem_.stateSoftConstraintPtr->add("endEffector",
        getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                  usePreComputation, libraryFolder, recompileLibraries));
    problem_.finalSoftConstraintPtr->add("finalEndEffector",
        getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                  usePreComputation, libraryFolder, recompileLibraries));
    
    // Independent body-relative and body-tracking soft constraints.
    bool activateBodyConstraint = false;
    loadData::loadPtreeValue(pt, activateBodyConstraint, "bodyRelative.activate", false);
    hasBodyRelativeConstraint_ = activateBodyConstraint;
    if (activateBodyConstraint) {
        problem_.stateSoftConstraintPtr->add("bodyRelative",
            getBodyTrackingConstraint(*pinocchioInterfacePtr_, taskFile, "bodyRelative",
                                       BodyConstraintMode::RELATIVE,
                                       usePreComputation, libraryFolder, recompileLibraries));
    }
    if (bodyTrackingEeEnabled_) {
        problem_.stateSoftConstraintPtr->add("bodyTracking",
            getBodyTrackingConstraint(*pinocchioInterfacePtr_, taskFile, "bodyTrackingEE",
                                       BodyConstraintMode::TRACKING,
                                       usePreComputation, libraryFolder, recompileLibraries));
    }
    if (!bodyTrackingEeEnabled_) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "bodyTrackingEE.activate is false/missing: TRACKING mode is disabled.");
    }
    if (headTrackingEeEnabled_) {
        problem_.stateSoftConstraintPtr->add("headTracking",
            getHeadTrackingConstraint(*pinocchioInterfacePtr_, taskFile, "headTrackingEE",
                                      usePreComputation, libraryFolder, recompileLibraries));
    } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "headTrackingEE.activate is false/missing: HEAD_TRACKING mode is disabled.");
    }
    
    // Bimanual coupling constraint
    bool activateBimanualCoupling = false;
    loadData::loadPtreeValue(pt, activateBimanualCoupling, "bimanualCoupling.activate", false);
    hasBimanualCoupling_ = activateBimanualCoupling;
    if (activateBimanualCoupling) {
        problem_.stateSoftConstraintPtr->add("bimanualCoupling",
            getBimanualCouplingConstraint(*pinocchioInterfacePtr_, taskFile, "bimanualCoupling",
                                           usePreComputation, libraryFolder, recompileLibraries));
    }
    if (auto homeJointReference = getHomeJointReferenceConstraint(taskFile)) {
        problem_.stateSoftConstraintPtr->add("homeJointReference", std::move(homeJointReference));
    }
    
    // Base lock constraint (only for mobile base) - as HARD equality constraint
    if (hasMobileBase_) {
        const size_t baseInputDim = modelInfo_.inputDim - modelInfo_.armDim;  // 2 (diff) or 3 (omni)
        auto baseLockConstraint = std::make_unique<BaseLockConstraint>(*referenceManagerPtr_, baseInputDim);
        
        // Add as hard equality constraint (u_base = 0 when locked)
        problem_.equalityConstraintPtr->add("baseLock", std::move(baseLockConstraint));
        
        reportStream() << " #### Base lock HARD constraint added (base inputs will be forced to zero when locked)\n";
    }

    bool activateWaistLockConstraint = true;
    size_t waistLockDof = 4;
    loadData::loadPtreeValue(pt, activateWaistLockConstraint, "waistLock.activate", true);
    loadData::loadPtreeValue(pt, waistLockDof, "waistLock.numJoints", true);
    const size_t baseInputDimForLocks = modelInfo_.inputDim - modelInfo_.armDim;

    // Head-waist coupling: read from top-level headCoupling section
    bool activateHeadCoupling = false;
    size_t headCouplingWaistJointIdx = 0;
    size_t headCouplingHeadJointIdx = 0;
    scalar_t headCouplingAlpha = 1.0;
    std::vector<size_t> lockedWaistJoints;
    loadData::loadPtreeValue(pt, activateHeadCoupling, "headCoupling.activate", false);
    if (activateHeadCoupling) {
        loadData::loadPtreeValue(pt, headCouplingWaistJointIdx, "headCoupling.waistJointIndex", true);
        loadData::loadPtreeValue(pt, headCouplingHeadJointIdx, "headCoupling.headJointIndex", true);
        loadData::loadPtreeValue(pt, headCouplingAlpha, "headCoupling.alpha", true);
        loadData::loadStdVector<size_t>(taskFile, "headCoupling.lockedWaistJoints", lockedWaistJoints, false);
    }

    bool activateArmLockConstraint = true;
    size_t leftArmInputDof = 7;
    size_t rightArmInputDof = 7;
    loadData::loadPtreeValue(pt, activateArmLockConstraint, "armLock.activate", true);
    loadData::loadPtreeValue(pt, leftArmInputDof, "armLock.leftNumJoints", true);
    loadData::loadPtreeValue(pt, rightArmInputDof, "armLock.rightNumJoints", true);

    // Determine left arm input start based on head joint position in input vector.
    // Supported layouts:
    //   head after  arms: [base | waist | left_arm | right_arm | head]  (headInputStart >= waistEnd + leftDof + rightDof)
    //   head before arms: [base | waist | head | left_arm | right_arm]  (headInputStart < waistEnd + leftDof + rightDof)
    // headInputStart is read from the top-level config; falls back to headCoupling.headJointIndex if set.
    const size_t waistEnd = baseInputDimForLocks + waistLockDof;
    size_t headInputStart = 0;
    loadData::loadPtreeValue(pt, headInputStart, "headInputStart", false);
    if (headInputStart == 0 && activateHeadCoupling) {
        headInputStart = headCouplingHeadJointIdx;
    }
    size_t leftArmInputStart = waistEnd;  // default: no head, arms right after waist
    if (headInputStart >= waistEnd && headInputStart < waistEnd + leftArmInputDof + rightArmInputDof) {
        // Head is between waist and arms
        const size_t headDof = modelInfo_.armDim - waistLockDof - leftArmInputDof - rightArmInputDof;
        leftArmInputStart = waistEnd + headDof;
    }
    // else: head is after arms (or absent), leftArmInputStart = waistEnd

    // Add waist / head coupling constraints
    if (activateHeadCoupling) {
        problem_.equalityConstraintPtr->add("headCoupling",
            std::make_unique<HeadCouplingConstraint>(*referenceManagerPtr_,
                                                     headCouplingWaistJointIdx,
                                                     headCouplingHeadJointIdx,
                                                     headCouplingAlpha,
                                                     lockedWaistJoints));
        hasHeadCoupling_ = true;
        reportStream() << " #### Head coupling constraint added (u[" << headCouplingWaistJointIdx
                  << "] - " << headCouplingAlpha << " * u[" << headCouplingHeadJointIdx
                  << "] = 0), lockedWaistJoints=" << lockedWaistJoints.size() << "\n";
    }

    const size_t totalArmInputDim = static_cast<size_t>(modelInfo_.armDim);
    const size_t headInputDof =
        (totalArmInputDim > (waistLockDof + leftArmInputDof + rightArmInputDof))
            ? (totalArmInputDim - waistLockDof - leftArmInputDof - rightArmInputDof)
            : 0;

    if (headInputDof > 0 && headInputStart == 0) {
        // Default layout: [waist | left_arm | right_arm | head]
        headInputStart = waistEnd + leftArmInputDof + rightArmInputDof;
    }

    // customJointLock is registered here (after headInputStart/headInputDof are finalized above) so
    // the head-range exclusion used by CustomJointLockConstraint while head midpoint gaze is active
    // (see CustomJointLockConstraint::activeIndices) receives the real head input window instead of
    // the [0,0) default.
    bool activateCustomJointLock = false;
    std::vector<size_t> customLockedJointIndices;
    loadData::loadPtreeValue(pt, activateCustomJointLock, "customJointLock.activate", false);
    if (activateCustomJointLock) {
        hasCustomJointLock_ = true;
        loadData::loadStdVector<size_t>(taskFile, "customJointLock.lockedJointIndices", customLockedJointIndices, false);
        bool indicesOk = !customLockedJointIndices.empty();
        for (const auto index : customLockedJointIndices) {
            if (index >= static_cast<size_t>(modelInfo_.inputDim)) {
                indicesOk = false;
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "customJointLock index " << index
                          << " is outside inputDim=" << modelInfo_.inputDim << "; custom lock skipped.");
            }
        }
        if (indicesOk) {
            problem_.equalityConstraintPtr->add(
                "customJointLock",
                std::make_unique<CustomJointLockConstraint>(*referenceManagerPtr_, customLockedJointIndices,
                                                             headInputStart, headInputDof));
            reportStream() << " #### Custom joint lock HARD constraint added (CUSTOM_LOCKED mode), indices=";
            for (const auto index : customLockedJointIndices) {
                reportStream() << " " << index;
            }
            reportStream() << "\n";
        } else if (customLockedJointIndices.empty()) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "customJointLock.activate=true but lockedJointIndices is empty; "
                         "CUSTOM_LOCKED mode enabled without hard joint locks.");
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "customJointLock.activate=true but lockedJointIndices is invalid; "
                         "CUSTOM_LOCKED mode enabled without hard joint locks.");
        }
    }

    if (headMidpointGazeEnabled_) {
        problem_.stateSoftConstraintPtr->add("headMidpointGaze",
            getHeadMidpointGazeConstraint(*pinocchioInterfacePtr_, taskFile, "headMidpointGaze",
                                           headInputStart, headInputDof,
                                           usePreComputation, libraryFolder, recompileLibraries));
        reportStream() << " #### Head midpoint gaze soft constraint added (active in HEAD_GAZE)\n";
    }

    // Implicit head lock: lock every head input except the coupled joint in HEAD_COUPLED mode.
    if (headInputDof > 0) {
        const bool headLockLayoutOk = headInputStart + headInputDof <= static_cast<size_t>(modelInfo_.inputDim);
        if (headLockLayoutOk) {
            const bool hasCoupledHeadJoint = activateHeadCoupling && headCouplingHeadJointIdx >= headInputStart &&
                                             headCouplingHeadJointIdx < headInputStart + headInputDof;
            problem_.equalityConstraintPtr->add(
                "headDefaultLock",
                std::make_unique<HeadDefaultLockConstraint>(*referenceManagerPtr_, headInputStart, headInputDof,
                                                            headCouplingHeadJointIdx, hasCoupledHeadJoint));
            hasHeadDefaultLock_ = true;
            reportStream() << " #### Head default lock HARD constraint added (implicit; all head u=0 except coupled joint in "
                         "HEAD_COUPLED), dof="
                      << headInputDof << " input offset=" << headInputStart;
            if (hasCoupledHeadJoint) {
                reportStream() << " coupledHeadInputIndex=" << headCouplingHeadJointIdx;
            }
            reportStream() << "\n";
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "implicit head lock layout invalid (headInputStart/head dof out of range); "
                         "head default lock skipped.");
        }
    }

    if (activateWaistLockConstraint && waistLockDof > 0) {
        if (baseInputDimForLocks + waistLockDof <= static_cast<size_t>(modelInfo_.inputDim)) {
            problem_.equalityConstraintPtr->add("waistLock",
                std::make_unique<WaistJointLockConstraint>(*referenceManagerPtr_,
                                                            baseInputDimForLocks, waistLockDof));
            hasWaistLock_ = true;
            reportStream() << " #### Waist lock HARD constraint added (waist joint velocities zero when LOCKED), dof="
                      << waistLockDof << " input offset=" << baseInputDimForLocks << "\n";
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "waistLock.numJoints too large for inputDim; waist lock skipped.");
        }
    }

    if (activateArmLockConstraint && (leftArmInputDof > 0 || rightArmInputDof > 0)) {
        const size_t rightArmInputStart = leftArmInputStart + leftArmInputDof;
        const bool layoutOk =
            rightArmInputStart + rightArmInputDof <= static_cast<size_t>(modelInfo_.inputDim) &&
            leftArmInputStart + leftArmInputDof <= rightArmInputStart;
        if (layoutOk) {
            armJointLayout_.leftStart = leftArmInputStart - baseInputDimForLocks;
            armJointLayout_.leftDof = leftArmInputDof;
            armJointLayout_.rightStart = rightArmInputStart - baseInputDimForLocks;
            armJointLayout_.rightDof = rightArmInputDof;

            const bool hardwareLayoutOk =
                leftArmInputStart >= baseInputDimForLocks &&
                rightArmInputStart >= baseInputDimForLocks;
            armJointLayout_.valid = hardwareLayoutOk;

            problem_.equalityConstraintPtr->add(
                "armLock", std::make_unique<ArmJointLockConstraint>(*referenceManagerPtr_, leftArmInputStart, leftArmInputDof,
                                                                    rightArmInputStart, rightArmInputDof));
            reportStream() << " #### Arm lock HARD constraint added (inactive arm joint velocities u=0; indices L[" << leftArmInputStart
                      << "," << leftArmInputDof << "] R[" << rightArmInputStart << "," << rightArmInputDof << "])\n";
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "armLock layout does not fit inputDim (check waistLock.numJoints, "
                         "armLock.leftNumJoints, armLock.rightNumJoints); arm lock skipped.");
        }
    }

    // 6/7-axis coupling (Marvin CCS coupled wrist range). Parsed regardless of activation so the
    // degree-domain runtime check stays available; registered after the arm layout is known so that the
    // default J6/J7 indices follow the detected [base | waist | left | right | head] layout.
    bool activateJoint67Coupling = false;
    loadData::loadPtreeValue(pt, activateJoint67Coupling, "joint67Coupling.activate", false);
    hasJoint67Coupling_ = activateJoint67Coupling;
    if (pt.get_child_optional("joint67Coupling")) {
        const size_t defaultLeftArmStart = armJointLayout_.valid ? armJointLayout_.leftStart : waistLockDof;
        const size_t defaultRightArmStart =
            armJointLayout_.valid ? armJointLayout_.rightStart : waistLockDof + leftArmInputDof;
        auto joint67CouplingConstraint =
            getJoint67CouplingConstraint(taskFile, defaultLeftArmStart, defaultRightArmStart);
        if (activateJoint67Coupling) {
            problem_.stateSoftConstraintPtr->add("joint67Coupling", std::move(joint67CouplingConstraint));
        }
    }
    
    // Self-collision
    bool activateSelfCollision = true;
    loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
    if (activateSelfCollision) {
        problem_.stateSoftConstraintPtr->add("selfCollision",
            getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile_,
                                        "selfCollision", usePreComputation, 
                                        libraryFolder, recompileLibraries));
    } else {
        selfCollisionEnabled_ = false;
    }
    
    // Dynamics (reuse from mobile_manipulator)
    switch (modelInfo_.manipulatorModelType) {
        case ManipulatorModelType::DefaultManipulator:
            problem_.dynamicsPtr = std::make_unique<DefaultManipulatorDynamics>(
                modelInfo_, "dynamics", libraryFolder, recompileLibraries, true);
            break;
        case ManipulatorModelType::FloatingArmManipulator:
            problem_.dynamicsPtr = std::make_unique<FloatingArmManipulatorDynamics>(
                modelInfo_, "dynamics", libraryFolder, recompileLibraries, true);
            break;
        case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
            problem_.dynamicsPtr = std::make_unique<FullyActuatedFloatingArmManipulatorDynamics>(
                modelInfo_, "dynamics", libraryFolder, recompileLibraries, true);
            break;
        case ManipulatorModelType::WheelBasedMobileManipulator:
            problem_.dynamicsPtr = std::make_unique<WheelBasedMobileManipulatorDynamics>(
                modelInfo_, "dynamics", libraryFolder, recompileLibraries, true);
            break;
        case ManipulatorModelType::OmniWheelBasedMobileManipulator:
            problem_.dynamicsPtr = std::make_unique<OmniWheelBasedMobileManipulatorDynamics>(
                modelInfo_, "dynamics", libraryFolder, recompileLibraries, true);
            break;
        default:
            throw std::invalid_argument("Invalid manipulator model type");
    }
    
    reportStream() << "\n #### Mode Capabilities:";
    reportStream() << "\n #### =============================================================================\n";
    reportStream() << " #### hasMobileBase:             " << (hasMobileBase_ ? "YES" : "NO (base unlock disabled)") << "\n";
    reportStream() << " #### hasBodyRelativeConstraint:  " << (hasBodyRelativeConstraint_ ? "YES" : "NO (RELATIVE mode disabled)") << "\n";
    reportStream() << " #### hasWaistLock:               " << (hasWaistLock_ ? "YES" : "NO (LOCKED mode disabled)") << "\n";
    reportStream() << " #### hasHeadCoupling:            " << (hasHeadCoupling_ ? "YES" : "NO (HEAD_COUPLED mode disabled)") << "\n";
    reportStream() << " #### hasHeadDefaultLock:         " << (hasHeadDefaultLock_ ? "YES (implicit)" : "NO (no head DOF)") << "\n";
    reportStream() << " #### hasCustomJointLock:         " << (hasCustomJointLock_ ? "YES" : "NO (CUSTOM_LOCKED mode disabled)") << "\n";
    reportStream() << " #### headMidpointGazeEnabled:    " << (headMidpointGazeEnabled_ ? "YES" : "NO (gaze disabled)") << "\n";
    reportStream() << " #### hasBimanualCoupling:        " << (hasBimanualCoupling_ ? "YES" : "NO (COUPLED mode disabled)") << "\n";
    reportStream() << " #### hasJoint67Coupling:         " << (hasJoint67Coupling_ ? "YES" : "NO") << "\n";
    reportStream() << " #### bodyTrackingEeEnabled:      " << (bodyTrackingEeEnabled_ ? "YES" : "NO (TRACKING mode disabled)") << "\n";
    reportStream() << " #### headTrackingEeEnabled:      " << (headTrackingEeEnabled_ ? "YES" : "NO (HEAD_TRACKING mode disabled)") << "\n";
    reportStream() << " #### hasHomeJointReference:      " << (hasHomeJointReference_ ? "YES" : "NO") << "\n";
    reportStream() << " #### =============================================================================\n";

    // Pre-computation
    if (usePreComputation) {
        problem_.preComputationPtr = std::make_unique<MobileManipulatorPreComputation>(
            *pinocchioInterfacePtr_, modelInfo_);
    }
}

//=============================================================================
// Cost Functions
//=============================================================================

std::unique_ptr<StateInputCost> WholeBodyInterface::getQuadraticInputCost(
    const std::string& taskFile) {
    
    matrix_t R = matrix_t::Zero(modelInfo_.inputDim, modelInfo_.inputDim);
    const int baseInputDim = modelInfo_.inputDim - modelInfo_.armDim;
    const int armInputDim = modelInfo_.armDim;
    
    if (baseInputDim > 0) {
        matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
        loadData::loadEigenMatrix(taskFile,
            "inputCost.R.base." + modelTypeEnumToString(modelInfo_.manipulatorModelType), R_base);
        R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
    }
    
    matrix_t R_arm = matrix_t::Zero(armInputDim, armInputDim);
    loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
    R.bottomRightCorner(armInputDim, armInputDim) = R_arm;
    
    return std::make_unique<QuadraticInputCost>(std::move(R), modelInfo_.stateDim);
}

std::unique_ptr<StateCost> WholeBodyInterface::getWaistPhiPositiveBiasCost(
    const std::string& taskFile) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    bool activateCustomJointLock = false;
    loadData::loadPtreeValue(pt, activateCustomJointLock, "customJointLock.activate", false);
    if (!activateCustomJointLock) {
        return nullptr;
    }

    constexpr auto configPrefix = "customJointLock.waistPhiPositiveBias";
    bool activateWaistPhiPositiveBias = false;
    loadData::loadPtreeValue(
        pt, activateWaistPhiPositiveBias,
        std::string(configPrefix) + ".activate", false);
    if (!activateWaistPhiPositiveBias) {
        return nullptr;
    }

    const std::string waistType =
        pt.get<std::string>(std::string(configPrefix) + ".waistType", "three_joint");
    scalar_t lowerBoundRad = 0.0;
    scalar_t mu = 10.0;
    loadData::loadPtreeValue(pt, lowerBoundRad, std::string(configPrefix) + ".lowerBoundRad", true);
    loadData::loadPtreeValue(pt, mu, std::string(configPrefix) + ".mu", true);

    const size_t baseStateDim = static_cast<size_t>(modelInfo_.stateDim - modelInfo_.armDim);
    const size_t armStateDim = static_cast<size_t>(modelInfo_.armDim);

    if (waistType == "single_joint") {
        size_t pitchJointIndex = 0;
        scalar_t pitchDirection = 1.0;
        scalar_t pitchOffset = 0.0;
        loadData::loadPtreeValue(
            pt, pitchJointIndex, std::string(configPrefix) + ".pitchJointIndex", true);
        loadData::loadPtreeValue(
            pt, pitchDirection, std::string(configPrefix) + ".pitchDirection", true);
        loadData::loadPtreeValue(
            pt, pitchOffset, std::string(configPrefix) + ".pitchOffset", true);

        if (pitchJointIndex >= armStateDim) {
            throw std::runtime_error(
                "[WholeBodyInterface] customJointLock.waistPhiPositiveBias.pitchJointIndex is outside armDim");
        }

        const size_t pitchStateIndex = baseStateDim + pitchJointIndex;
        reportStream() << " #### Waist phi positive bias added: waistType=single_joint pitchStateIndex="
                  << pitchStateIndex << " pitchDirection=" << pitchDirection
                  << " pitchOffset=" << pitchOffset
                  << " lowerBoundRad=" << lowerBoundRad
                  << " mu=" << mu << "\n";

        return std::make_unique<WaistPhiPositiveBiasCost>(
            *referenceManagerPtr_, pitchStateIndex, pitchDirection, pitchOffset,
            lowerBoundRad, mu);
    }

    if (waistType != "three_joint") {
        throw std::runtime_error(
            "[WholeBodyInterface] customJointLock.waistPhiPositiveBias.waistType must be 'three_joint' or 'single_joint'");
    }

    std::vector<size_t> waistJointIndices;
    loadData::loadStdVector<size_t>(
        taskFile, std::string(configPrefix) + ".waistJointIndices", waistJointIndices, true);
    if (waistJointIndices.size() != 3) {
        throw std::runtime_error(
            "[WholeBodyInterface] customJointLock.waistPhiPositiveBias.waistJointIndices must contain exactly 3 indices");
    }

    vector_t rotationDirectionVector(3);
    vector_t angleOffsetVector(3);
    loadData::loadEigenMatrix(
        taskFile, std::string(configPrefix) + ".rotationDirection", rotationDirectionVector);
    loadData::loadEigenMatrix(
        taskFile, std::string(configPrefix) + ".angleOffset", angleOffsetVector);

    size_t yawJointIndex = 0;
    scalar_t yawOffset = 0.0;
    loadData::loadPtreeValue(
        pt, yawJointIndex, std::string(configPrefix) + ".yawJointIndex", true);
    loadData::loadPtreeValue(
        pt, yawOffset, std::string(configPrefix) + ".yawOffset", true);

    WaistPhiPositiveBiasCost::index_array_t waistStateIndices{};
    for (size_t i = 0; i < waistJointIndices.size(); ++i) {
        if (waistJointIndices[i] >= armStateDim) {
            throw std::runtime_error(
                "[WholeBodyInterface] customJointLock.waistPhiPositiveBias.waistJointIndices contains index outside armDim");
        }
        waistStateIndices[i] = baseStateDim + waistJointIndices[i];
    }

    if (yawJointIndex >= armStateDim) {
        throw std::runtime_error(
            "[WholeBodyInterface] customJointLock.waistPhiPositiveBias.yawJointIndex is outside armDim");
    }
    const size_t yawStateIndex = baseStateDim + yawJointIndex;

    const WaistPhiPositiveBiasCost::vector3_t rotationDirection =
        rotationDirectionVector.head<3>();
    const WaistPhiPositiveBiasCost::vector3_t angleOffset =
        angleOffsetVector.head<3>();

    reportStream() << " #### Waist phi positive bias added: waistType=three_joint waistStateIndices=["
              << waistStateIndices[0] << ", " << waistStateIndices[1] << ", " << waistStateIndices[2]
              << "] rotationDirection=[" << rotationDirection.transpose()
              << "] angleOffset=[" << angleOffset.transpose()
              << "] yawStateIndex=" << yawStateIndex
              << " yawOffset=" << yawOffset
              << " lowerBoundRad=" << lowerBoundRad
              << " mu=" << mu << "\n";

    return std::make_unique<WaistPhiPositiveBiasCost>(
        *referenceManagerPtr_, waistStateIndices, rotationDirection,
        angleOffset, yawStateIndex, yawOffset, lowerBoundRad, mu);
}

std::unique_ptr<StateCost> WholeBodyInterface::getHomeJointReferenceConstraint(
    const std::string& taskFile) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    bool activate = false;
    loadData::loadPtreeValue(pt, activate, "homeJointReference.activate", false);
    if (!activate) {
        hasHomeJointReference_ = false;
        homeJointReferenceDefaultEnabled_ = false;
        return nullptr;
    }

    scalar_t mu = 1e-3;
    bool activateDefault = false;
    loadData::loadPtreeValue(pt, mu, "homeJointReference.mu", false);
    loadData::loadPtreeValue(pt, activateDefault, "homeJointReference.activateDefault", false);

    const size_t armDim = static_cast<size_t>(modelInfo_.armDim);
    const size_t baseStateDim =
        static_cast<size_t>(modelInfo_.stateDim) > armDim
            ? static_cast<size_t>(modelInfo_.stateDim) - armDim
            : 0;

    std::vector<size_t> excludeJointIndices;
    loadData::loadStdVector<size_t>(
        taskFile, "homeJointReference.excludeJointIndices", excludeJointIndices, false);
    std::unordered_set<size_t> excluded;
    for (const auto index : excludeJointIndices) {
        if (index >= armDim) {
            throw std::runtime_error(
                "[WholeBodyInterface] homeJointReference.excludeJointIndices contains index outside armDim");
        }
        excluded.insert(index);
    }

    std::vector<size_t> trackedArmIndices;
    trackedArmIndices.reserve(armDim);
    for (size_t i = 0; i < armDim; ++i) {
        if (excluded.find(i) == excluded.end()) {
            trackedArmIndices.push_back(i);
        }
    }
    if (trackedArmIndices.empty()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "homeJointReference.activate=true but all joints are excluded; skipped.");
        hasHomeJointReference_ = false;
        homeJointReferenceDefaultEnabled_ = false;
        return nullptr;
    }

    hasHomeJointReference_ = true;
    homeJointReferenceDefaultEnabled_ = activateDefault;
    referenceManagerPtr_->setHomeJointReferenceEnabled(activateDefault);

    reportStream() << " #### HOME joint reference added: tracked=" << trackedArmIndices.size()
              << "/" << armDim << " mu=" << mu
              << " default=" << (activateDefault ? "ON" : "OFF")
              << " exclude=";
    if (excludeJointIndices.empty()) {
        reportStream() << "(none)";
    } else {
        for (const auto index : excludeJointIndices) {
            reportStream() << " " << index;
        }
    }
    reportStream() << "\n";

    auto constraint = std::make_unique<HomeJointReferenceConstraint>(
        *referenceManagerPtr_, baseStateDim, std::move(trackedArmIndices));
    const size_t constraintDim = constraint->trackedJointCount();
    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(constraintDim);
    for (size_t i = 0; i < constraintDim; ++i) {
        penaltyArray[i] = std::make_unique<QuadraticPenalty>(mu);
    }
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

std::unique_ptr<StateCost> WholeBodyInterface::getJoint67CouplingConstraint(
    const std::string& taskFile, size_t defaultLeftArmStart, size_t defaultRightArmStart) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    const std::string prefix = "joint67Coupling";
    const size_t armDim = static_cast<size_t>(modelInfo_.armDim);
    // Arm joints are the last armDim state entries; a mobile base contributes the leading states.
    const size_t baseStateDim = static_cast<size_t>(modelInfo_.stateDim - modelInfo_.armDim);

    // The raw CCS boundary is the manufacturer's octagon. Log-sum-exp rounds all eight joins
    // inward, while safetyMarginDeg supplies an additional global clearance.
    Joint67CouplingConstraint::Config config;
    scalar_t penaltyWeight = 50.0;
    scalar_t penaltyActivationDeg = 3.0;
    loadData::loadPtreeValue(pt, config.j6LimitDeg, prefix + ".j6LimitDeg", true);
    loadData::loadPtreeValue(pt, config.j7LimitDeg, prefix + ".j7LimitDeg", true);
    loadData::loadPtreeValue(pt, config.diagonalLimitDeg, prefix + ".diagonalLimitDeg", true);
    loadData::loadPtreeValue(pt, config.diagonalSlope, prefix + ".slope", true);
    loadData::loadPtreeValue(pt, config.smoothTauDeg, prefix + ".smoothTauDeg", true);
    loadData::loadPtreeValue(pt, config.safetyMarginDeg, prefix + ".safetyMarginDeg", true);
    loadData::loadPtreeValue(pt, penaltyWeight, prefix + ".penaltyWeight", true);
    loadData::loadPtreeValue(pt, penaltyActivationDeg, prefix + ".penaltyActivationDeg", true);
    if (!std::isfinite(penaltyWeight) || penaltyWeight <= 0.0 ||
        !std::isfinite(penaltyActivationDeg) || penaltyActivationDeg < 0.0) {
        throw std::invalid_argument(
            "[WholeBodyInterface] invalid joint67Coupling penalty configuration");
    }

    // Indices in the .info file are arm-segment relative (they exclude the base states); the base
    // offset is added here so the same section works for a fixed base and a mobile base.
    auto loadArm = [&](const std::string& side, size_t defaultArmStart)
        -> std::optional<Joint67CouplingConstraint::ArmCoupling> {
        const std::string armPrefix = prefix + "." + side;
        if (!pt.get_child_optional(armPrefix)) {
            return std::nullopt;
        }
        size_t j6Index = defaultArmStart + 5;
        size_t j7Index = defaultArmStart + 6;
        loadData::loadPtreeValue(pt, j6Index, armPrefix + ".j6Index", false);
        loadData::loadPtreeValue(pt, j7Index, armPrefix + ".j7Index", false);
        if (j6Index >= armDim || j7Index >= armDim) {
            throw std::runtime_error(
                "[WholeBodyInterface] joint67Coupling." + side +
                " j6Index/j7Index is outside armDim");
        }
        Joint67CouplingConstraint::ArmCoupling arm;
        arm.j6StateIndex = baseStateDim + j6Index;
        arm.j7StateIndex = baseStateDim + j7Index;
        reportStream() << " ####   joint67Coupling." << side << ": J6/J7 state indices "
                  << arm.j6StateIndex << "/" << arm.j7StateIndex << '\n';
        return arm;
    };

    std::vector<Joint67CouplingConstraint::ArmCoupling> arms;
    if (auto left = loadArm("left", defaultLeftArmStart)) {
        arms.push_back(*left);
    }
    if (auto right = loadArm("right", defaultRightArmStart)) {
        arms.push_back(*right);
    }
    // Fallback: coupling always applies to the 6/7 axes of each 7-DOF arm.
    if (arms.empty()) {
        if (armDim >= 7) {
            arms.push_back(Joint67CouplingConstraint::ArmCoupling{
                baseStateDim + defaultLeftArmStart + 5, baseStateDim + defaultLeftArmStart + 6});
        }
        if (armDim >= 14) {
            arms.push_back(Joint67CouplingConstraint::ArmCoupling{
                baseStateDim + defaultRightArmStart + 5, baseStateDim + defaultRightArmStart + 6});
        }
    }
    if (arms.empty()) {
        throw std::runtime_error(
            "[WholeBodyInterface] joint67Coupling enabled but armDim too small "
            "for a 6/7-axis coupling layout");
    }

    reportStream() << " #### Joint67Coupling: J6=" << config.j6LimitDeg
              << "deg J7=" << config.j7LimitDeg
              << "deg diagonal=" << config.diagonalSlope << "*|J6|+|J7|<="
              << config.diagonalLimitDeg << "deg"
              << " smoothTau=" << config.smoothTauDeg << "deg"
              << " safetyMargin=" << config.safetyMarginDeg << "deg"
              << " penaltyWeight=" << penaltyWeight
              << " penaltyActivation=" << penaltyActivationDeg << "deg"
              << " arms=" << arms.size() << '\n';

    // Keep the state indices and degree-domain config for runtime checks.
    joint67CouplingArms_ = arms;
    joint67CouplingConfig_ = config;

    auto constraint = std::make_unique<Joint67CouplingConstraint>(
        std::move(arms), config, static_cast<size_t>(modelInfo_.stateDim));

    constexpr scalar_t kDegToRad = 3.14159265358979323846 / 180.0;
    auto penalty = std::make_unique<SquaredHingePenalty>(SquaredHingePenalty::Config{
        penaltyWeight, penaltyActivationDeg * kDegToRad});
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

std::vector<Joint67CouplingConstraint::CouplingStatus>
WholeBodyInterface::checkJoint67Coupling(const vector_t& state) const {
    constexpr scalar_t kRadToDeg = 180.0 / 3.14159265358979323846;
    std::vector<Joint67CouplingConstraint::CouplingStatus> statuses;
    if (joint67CouplingArms_.empty()) {
        return statuses;
    }
    if (static_cast<size_t>(state.size()) != static_cast<size_t>(modelInfo_.stateDim)) {
        throw std::out_of_range(
            "[WholeBodyInterface] checkJoint67Coupling state dimension mismatch");
    }
    statuses.reserve(joint67CouplingArms_.size());
    for (const auto& arm : joint67CouplingArms_) {
        const scalar_t j6Deg = state(static_cast<Eigen::Index>(arm.j6StateIndex)) * kRadToDeg;
        const scalar_t j7Deg = state(static_cast<Eigen::Index>(arm.j7StateIndex)) * kRadToDeg;
        statuses.push_back(Joint67CouplingConstraint::checkCouplingDeg(j6Deg, j7Deg, joint67CouplingConfig_));
    }
    return statuses;
}

void WholeBodyInterface::setHomeJointReferenceFromArmJoints(
    const std::vector<double>& armJoints) {
    if (!referenceManagerPtr_) {
        return;
    }
    const size_t armDim = static_cast<size_t>(modelInfo_.armDim);
    vector_t qHome = vector_t::Zero(static_cast<Eigen::Index>(armDim));
    const size_t n = std::min(armJoints.size(), armDim);
    for (size_t i = 0; i < n; ++i) {
        qHome(static_cast<Eigen::Index>(i)) = armJoints[i];
    }
    referenceManagerPtr_->setHomeJointReference(qHome);
}

//=============================================================================
// End-Effector Constraint (reuse from mobile_manipulator with modifications)
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getEndEffectorConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    scalar_t muPosition = 1.0;
    scalar_t muOrientation = 1.0;
    loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
    loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
    
    scalar_t leftMuPosition = muPosition;
    scalar_t leftMuOrientation = muOrientation;
    scalar_t rightMuPosition = muPosition;
    scalar_t rightMuOrientation = muOrientation;
    loadData::loadPtreeValue(pt, leftMuPosition, prefix + ".leftArm.muPosition", false);
    loadData::loadPtreeValue(pt, leftMuOrientation, prefix + ".leftArm.muOrientation", false);
    loadData::loadPtreeValue(pt, rightMuPosition, prefix + ".rightArm.muPosition", false);
    loadData::loadPtreeValue(pt, rightMuOrientation, prefix + ".rightArm.muOrientation", false);
    
    reportStream() << "\n #### " << prefix << " Settings: ";
    reportStream() << "\n #### =============================================================================\n";
    reportStream() << " #### Left  arm: muPosition=" << leftMuPosition << ", muOrientation=" << leftMuOrientation << std::endl;
    reportStream() << " #### Right arm: muPosition=" << rightMuPosition << ", muOrientation=" << rightMuOrientation << std::endl;
    reportStream() << " #### =============================================================================\n";
    
    std::unique_ptr<StateConstraint> constraint;
    
    if (usePreComputation) {
        MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo_);
        PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,
            {modelInfo_.eeFrame, modelInfo_.eeFrame1});
        constraint = std::make_unique<WholeBodyEndEffectorConstraint>(eeKinematics, *referenceManagerPtr_);
    } else {
        MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(modelInfo_);
        PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd,
            {modelInfo_.eeFrame, modelInfo_.eeFrame1},
            modelInfo_.stateDim, modelInfo_.inputDim,
            "end_effector_kinematics", libraryFolder, recompileLibraries, false);
        constraint = std::make_unique<WholeBodyEndEffectorConstraint>(eeKinematics, *referenceManagerPtr_);
    }
    
    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(12);
    std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(leftMuPosition); });
    std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(leftMuOrientation); });
    std::generate_n(penaltyArray.begin() + 6, 3, [&] { return std::make_unique<QuadraticPenalty>(rightMuPosition); });
    std::generate_n(penaltyArray.begin() + 9, 3, [&] { return std::make_unique<QuadraticPenalty>(rightMuOrientation); });
    
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

//=============================================================================
// Body Tracking Constraint
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getBodyTrackingConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    BodyConstraintMode constraintMode,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    scalar_t muPosition = constraintMode == BodyConstraintMode::TRACKING
        ? pt.get<scalar_t>("bodyRelative.muPosition", 1.0)
        : 1.0;
    scalar_t muOrientation = constraintMode == BodyConstraintMode::TRACKING
        ? pt.get<scalar_t>("bodyRelative.muOrientation", 1.0)
        : 1.0;
    loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", false);
    loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", false);

    scalar_t relativeXLowerBound = 0.0;
    scalar_t relativeXUpperBound = 0.0;
    if (constraintMode == BodyConstraintMode::RELATIVE) {
        const auto lowerBound = pt.get_optional<scalar_t>(prefix + ".xLowerBound");
        const auto upperBound = pt.get_optional<scalar_t>(prefix + ".xUpperBound");
        if (lowerBound.has_value() != upperBound.has_value()) {
            throw std::invalid_argument(
                "[WholeBodyInterface] bodyRelative.xLowerBound and xUpperBound must be configured together");
        }
        if (lowerBound && upperBound) {
            relativeXLowerBound = *lowerBound;
            relativeXUpperBound = *upperBound;
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyInterface"), "Missing bodyRelative x bounds; "
                         "using legacy [0, 0] behavior.");
        }
        if (relativeXLowerBound > relativeXUpperBound) {
            throw std::invalid_argument(
                "[WholeBodyInterface] bodyRelative.xLowerBound must not exceed xUpperBound");
        }
    }
    
    std::unique_ptr<StateConstraint> constraint;
    
    const size_t baseDim = modelInfo_.stateDim - modelInfo_.armDim;
    if (usePreComputation) {
        MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo_);
        PinocchioEndEffectorKinematics bodyKinematics(pinocchioInterface, pinocchioMapping,
            {modelInfo_.bodyFrame});
        constraint = std::make_unique<BodyTrackingConstraint>(
            bodyKinematics, *referenceManagerPtr_, modelInfo_.bodyFrame, constraintMode,
            relativeXLowerBound, relativeXUpperBound, baseDim);
    } else {
        MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(modelInfo_);
        PinocchioEndEffectorKinematicsCppAd bodyKinematics(pinocchioInterface, pinocchioMappingCppAd,
            {modelInfo_.bodyFrame},
            modelInfo_.stateDim, modelInfo_.inputDim,
            "body_kinematics", libraryFolder, recompileLibraries, false);
        constraint = std::make_unique<BodyTrackingConstraint>(
            bodyKinematics, *referenceManagerPtr_, modelInfo_.bodyFrame, constraintMode,
            relativeXLowerBound, relativeXUpperBound, baseDim);
    }
    
    const size_t constraintDim = constraintMode == BodyConstraintMode::RELATIVE ? 3 : 6;
    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(constraintDim);
    if (constraintMode == BodyConstraintMode::RELATIVE) {
        penaltyArray[0] = std::make_unique<QuadraticPenalty>(muPosition);
        penaltyArray[1] = std::make_unique<QuadraticPenalty>(muOrientation);
        penaltyArray[2] = std::make_unique<QuadraticPenalty>(muOrientation);
    } else {
        std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
        std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });
    }
    
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

//=============================================================================
// Head Tracking Constraint
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getHeadTrackingConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    scalar_t muPosition = 1.0;
    scalar_t muOrientation = 1.0;
    loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", false);
    loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", false);

    std::unique_ptr<StateConstraint> constraint;
    if (usePreComputation) {
        MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo_);
        PinocchioEndEffectorKinematics headKinematics(
            pinocchioInterface, pinocchioMapping, {modelInfo_.headFrame});
        constraint = std::make_unique<HeadTrackingConstraint>(
            headKinematics, *referenceManagerPtr_, modelInfo_.headFrame);
    } else {
        MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(modelInfo_);
        PinocchioEndEffectorKinematicsCppAd headKinematics(
            pinocchioInterface, pinocchioMappingCppAd, {modelInfo_.headFrame},
            modelInfo_.stateDim, modelInfo_.inputDim,
            "head_kinematics", libraryFolder, recompileLibraries, false);
        constraint = std::make_unique<HeadTrackingConstraint>(
            headKinematics, *referenceManagerPtr_, modelInfo_.headFrame);
    }

    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
    std::generate_n(penaltyArray.begin(), 3,
                    [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
    std::generate_n(penaltyArray.begin() + 3, 3,
                    [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

//=============================================================================
// Head Midpoint Gaze Constraint
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getHeadMidpointGazeConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    size_t headInputStart,
    size_t headInputDof,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    std::string cameraFrame;
    loadData::loadPtreeValue<std::string>(pt, cameraFrame, prefix + ".cameraFrame", true);

    HeadMidpointGazeConstraint::CameraIntrinsics intrinsics;
    loadData::loadPtreeValue(pt, intrinsics.fx, prefix + ".fx", true);
    loadData::loadPtreeValue(pt, intrinsics.fy, prefix + ".fy", true);
    loadData::loadPtreeValue(pt, intrinsics.cx, prefix + ".cx", true);
    loadData::loadPtreeValue(pt, intrinsics.cy, prefix + ".cy", true);
    loadData::loadPtreeValue(pt, intrinsics.width, prefix + ".width", true);
    loadData::loadPtreeValue(pt, intrinsics.height, prefix + ".height", true);
    loadData::loadPtreeValue(pt, intrinsics.minDepth, prefix + ".minDepth", true);
    loadData::loadPtreeValue(pt, intrinsics.horizontalDeadbandDeg, prefix + ".horizontalDeadbandDeg", true);
    loadData::loadPtreeValue(pt, intrinsics.verticalDeadbandDeg, prefix + ".verticalDeadbandDeg", true);
    scalar_t muAlignmentHorizontal = 5.0;
    scalar_t muAlignmentVertical = 5.0;
    scalar_t muFront = 5.0;
    scalar_t muHandsHorizon = 0.0;
    scalar_t muUpright = 0.0;
    loadData::loadPtreeValue(pt, muAlignmentHorizontal, prefix + ".muAlignmentHorizontal", true);
    loadData::loadPtreeValue(pt, muAlignmentVertical, prefix + ".muAlignmentVertical", true);
    loadData::loadPtreeValue(pt, muFront, prefix + ".muFront", true);
    loadData::loadPtreeValue(pt, muHandsHorizon, prefix + ".muHandsHorizon", false);
    loadData::loadPtreeValue(pt, muUpright, prefix + ".muUpright", false);
    const bool enableHandsHorizon = muHandsHorizon > 0.0;
    const bool enableUpright = muUpright > 0.0;
    if (enableHandsHorizon && enableUpright) {
        throw std::invalid_argument(
            "[WholeBodyInterface] headMidpointGaze.muHandsHorizon and muUpright are mutually exclusive");
    }
    if (enableUpright) {
        loadData::loadPtreeValue(pt, intrinsics.uprightDeadbandDeg, prefix + ".uprightDeadbandDeg", false);
    }

    reportStream() << "\n #### " << prefix << " Settings:";
    reportStream() << "\n #### =============================================================================\n";
    reportStream() << " #### cameraFrame=\"" << cameraFrame << "\" fx=" << intrinsics.fx << " fy=" << intrinsics.fy
              << " cx=" << intrinsics.cx << " cy=" << intrinsics.cy << " size=" << intrinsics.width << "x"
              << intrinsics.height << " muAlignmentHorizontal=" << muAlignmentHorizontal
              << " muAlignmentVertical=" << muAlignmentVertical << " muFront=" << muFront
              << " muHandsHorizon=" << muHandsHorizon << " muUpright=" << muUpright
              << " horizontalDeadbandDeg=" << intrinsics.horizontalDeadbandDeg
              << " verticalDeadbandDeg=" << intrinsics.verticalDeadbandDeg;
    if (enableUpright) {
        reportStream() << " uprightDeadbandDeg=" << intrinsics.uprightDeadbandDeg;
    }
    reportStream() << " minDepth=" << intrinsics.minDepth << " headInputStart=" << headInputStart
              << " headInputDof=" << headInputDof << "\n";
    reportStream() << " #### =============================================================================\n";

    std::unique_ptr<StateConstraint> constraint;
    if (usePreComputation) {
        MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo_);
        PinocchioEndEffectorKinematics gazeKinematics(
            pinocchioInterface, pinocchioMapping, {modelInfo_.eeFrame, modelInfo_.eeFrame1, cameraFrame});
        constraint = std::make_unique<HeadMidpointGazeConstraint>(
            gazeKinematics, pinocchioInterface, pinocchioMapping, *referenceManagerPtr_, intrinsics, cameraFrame,
            static_cast<size_t>(modelInfo_.stateDim), static_cast<size_t>(modelInfo_.inputDim),
            headInputStart, headInputDof, enableHandsHorizon, enableUpright);
    } else {
        MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(modelInfo_);
        constraint = std::make_unique<HeadMidpointGazeConstraint>(
            pinocchioInterface, pinocchioMappingCppAd, *referenceManagerPtr_, intrinsics,
            modelInfo_.eeFrame, modelInfo_.eeFrame1, cameraFrame,
            static_cast<size_t>(modelInfo_.stateDim), static_cast<size_t>(modelInfo_.inputDim),
            headInputStart, headInputDof, "head_midpoint_gaze_kinematics", libraryFolder,
            recompileLibraries, enableHandsHorizon, enableUpright);
    }

    const size_t penaltyDim = (enableHandsHorizon || enableUpright) ? 4 : 3;
    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(penaltyDim);
    penaltyArray[0] = std::make_unique<QuadraticPenalty>(muAlignmentHorizontal);
    penaltyArray[1] = std::make_unique<QuadraticPenalty>(muAlignmentVertical);
    penaltyArray[2] = std::make_unique<QuadraticPenalty>(muFront);
    if (enableHandsHorizon) {
        penaltyArray[3] = std::make_unique<QuadraticPenalty>(muHandsHorizon);
    } else if (enableUpright) {
        penaltyArray[3] = std::make_unique<QuadraticPenalty>(muUpright);
    }
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

//=============================================================================
// Bimanual Coupling Constraint
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getBimanualCouplingConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& prefix,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    scalar_t muPosition = 5.0;
    scalar_t muOrientation = 5.0;
    loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", false);
    loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", false);
    
    std::unique_ptr<StateConstraint> constraint;

    // Bimanual coupling uses Pinocchio getBodyId + oMf via MobileManipulatorPreComputation.
    if (!usePreComputation) {
        throw std::runtime_error(
            "[WholeBodyInterface] bimanualCoupling requires model_settings.usePreComputation=true "
            "(CppAd / no-precomputation path is unsupported).");
    }
    (void)libraryFolder;
    (void)recompileLibraries;
    MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo_);
    PinocchioEndEffectorKinematics dualArmKinematics(pinocchioInterface, pinocchioMapping,
        {modelInfo_.eeFrame, modelInfo_.eeFrame1});
    constraint = std::make_unique<BimanualCouplingConstraint>(
        dualArmKinematics, *referenceManagerPtr_);
    
    // 6 penalties for coupling constraint
    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
    std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
    std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });
    
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

//=============================================================================
// Joint Limit Constraint (reuse from mobile_manipulator)
//=============================================================================

std::unique_ptr<StateInputCost> WholeBodyInterface::getJointLimitSoftConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile) {
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    bool activateJointPositionLimit = true;
    loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);
    
    const int baseStateDim = modelInfo_.stateDim - modelInfo_.armDim;
    const int armStateDim = modelInfo_.armDim;
    const int baseInputDim = modelInfo_.inputDim - modelInfo_.armDim;
    const int armInputDim = modelInfo_.armDim;
    const auto& model = pinocchioInterface.getModel();
    
    std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
    
    if (activateJointPositionLimit) {
        scalar_t muPositionLimits = 1e-2;
        scalar_t deltaPositionLimits = 1e-3;
        
        const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
        const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);
        
        loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
        loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
        
        stateLimits.reserve(armStateDim);
        for (int i = 0; i < armStateDim; ++i) {
            StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = baseStateDim + i;
            boxConstraint.lowerBound = lowerBound(i);
            boxConstraint.upperBound = upperBound(i);
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
            stateLimits.push_back(std::move(boxConstraint));
        }
    }
    
    // Velocity limits
    std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
    {
        vector_t lowerBound = vector_t::Zero(modelInfo_.inputDim);
        vector_t upperBound = vector_t::Zero(modelInfo_.inputDim);
        scalar_t muVelocityLimits = 1e-2;
        scalar_t deltaVelocityLimits = 1e-3;
        
        if (baseInputDim > 0) {
            vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
            vector_t upperBoundBase = vector_t::Zero(baseInputDim);
            loadData::loadEigenMatrix(taskFile,
                "jointVelocityLimits.lowerBound.base." + modelTypeEnumToString(modelInfo_.manipulatorModelType),
                lowerBoundBase);
            loadData::loadEigenMatrix(taskFile,
                "jointVelocityLimits.upperBound.base." + modelTypeEnumToString(modelInfo_.manipulatorModelType),
                upperBoundBase);
            lowerBound.head(baseInputDim) = lowerBoundBase;
            upperBound.head(baseInputDim) = upperBoundBase;
        }
        
        vector_t lowerBoundArm = vector_t::Zero(armInputDim);
        vector_t upperBoundArm = vector_t::Zero(armInputDim);
        loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
        loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
        lowerBound.tail(armInputDim) = lowerBoundArm;
        upperBound.tail(armInputDim) = upperBoundArm;
        
        loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
        loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
        
        inputLimits.reserve(modelInfo_.inputDim);
        for (int i = 0; i < modelInfo_.inputDim; ++i) {
            StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = i;
            boxConstraint.lowerBound = lowerBound(i);
            boxConstraint.upperBound = upperBound(i);
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
            inputLimits.push_back(std::move(boxConstraint));
        }
    }
    
    auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
    boxConstraints->initializeOffset(0.0, vector_t::Zero(modelInfo_.stateDim),
                                      vector_t::Zero(modelInfo_.stateDim));
    return boxConstraints;
}

//=============================================================================
// Self-Collision Constraint (reuse from mobile_manipulator)
//=============================================================================

std::unique_ptr<StateCost> WholeBodyInterface::getSelfCollisionConstraint(
    const PinocchioInterface& pinocchioInterface,
    const std::string& taskFile,
    const std::string& urdfFile,
    const std::string& prefix,
    bool usePreComputation,
    const std::string& libraryFolder,
    bool recompileLibraries) {
    
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
    scalar_t mu = 1e-2;
    scalar_t delta = 1e-3;
    scalar_t minimumDistance = 0.0;
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    
    loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
    loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
    loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);

    scalar_t activationDistance = 5.0 * minimumDistance;
    loadData::loadPtreeValue(pt, activationDistance, prefix + ".activationDistance", false);

    selfCollisionEnabled_ = true;
    selfCollisionMinimumDistance_ = minimumDistance;
    selfCollisionActivationDistance_ = activationDistance;
    loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
    loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
    
    pinocchioGeometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(
        pinocchioInterface, urdfFile, collisionLinkPairs, collisionObjectPairs);
    
    reportStream() << "\n #### SelfCollision Settings: ";
    reportStream() << "\n #### =============================================================================\n";
    reportStream() << " #### minimumDistance: " << minimumDistance << " (minimum allowed distance)\n";
    reportStream() << " #### activationDistance: " << activationDistance << " (penalty only active when distance < this value)\n";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("WholeBodyInterface"), "SelfCollision: Testing for " << pinocchioGeometryInterfacePtr_->getNumCollisionPairs() 
              << " collision pairs");
    reportStream() << " #### =============================================================================\n";
    
    std::unique_ptr<StateConstraint> constraint;
    if (usePreComputation) {
        PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
        constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(
            MobileManipulatorPinocchioMapping(modelInfo_),
            std::move(geometryInterfaceCopy), minimumDistance);
    } else {
        PinocchioGeometryInterface geometryInterfaceCopy(*pinocchioGeometryInterfacePtr_);
        constraint = std::make_unique<SelfCollisionConstraintCppAd>(
            pinocchioInterface, MobileManipulatorPinocchioMapping(modelInfo_),
            std::move(geometryInterfaceCopy), minimumDistance,
            "self_collision", libraryFolder, recompileLibraries, false);
    }
    
    const scalar_t activationThreshold = activationDistance - minimumDistance;
    auto penalty = std::make_unique<ThresholdRelaxedBarrierPenalty>(
        ThresholdRelaxedBarrierPenalty::Config{mu, delta, activationThreshold});
    
    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

std::unique_ptr<PinocchioGeometryInterface> WholeBodyInterface::getPinocchioGeometryInterface() const {
    if (pinocchioGeometryInterfacePtr_) {
        return std::make_unique<PinocchioGeometryInterface>(*pinocchioGeometryInterfacePtr_);
    }
    return nullptr;
}

} // namespace ocs2::whole_body_mpc
