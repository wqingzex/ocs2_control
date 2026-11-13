/******************************************************************************
 * OCS2 Wheel Humanoid - Main Interface
 * 
 * Main interface for wheel-based humanoid robot MPC controller.
 * Inherits and extends mobile_manipulator functionality with:
 * - ModeSchedule support for task switching
 * - Bimanual coupling constraint
 * - Mode-aware body tracking
 ******************************************************************************/

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

// Reuse from mobile_manipulator
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/constraint/Joint67CouplingConstraint.h>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace ocs2::whole_body_mpc {

// Reuse ManipulatorModelInfo from mobile_manipulator
using mobile_manipulator::ManipulatorModelInfo;
using mobile_manipulator::ManipulatorModelType;

/**
 * @brief Extended model info for wheel humanoid
 */
struct WholeBodyModelInfo : public ManipulatorModelInfo {
    std::string bodyFrame;          // Body/torso frame name (from model_information.bodyFrame)
    std::string headFrame;          // Head tracking frame (from model_information.headFrame)
    // EE frames: use base class eeFrame (left) / eeFrame1 (right)
};

/**
 * Optional in-memory overrides after reading task.info.
 * Empty string means keep the value loaded from task.info.
 * Convention: eeFrame = left tip, eeFrame1 = right tip.
 * recompileLibraries: nullopt keeps model_settings.recompileLibraries from info.
 */
struct FrameOverrides {
    std::string baseFrame;
    std::string eeFrame;
    std::string eeFrame1;
    std::string bodyFrame;
    std::string headFrame;
    std::optional<bool> recompileLibraries;
};

/**
 * @brief Main interface for wheel humanoid MPC
 * 
 * This interface builds on mobile_manipulator and adds:
 * - ModeSchedule-based task switching
 * - BimanualCouplingConstraint for coordinated manipulation
 * - Mode-aware BodyTrackingConstraint
 */
class WholeBodyInterface {
public:
    /**
     * @brief Constructor
     * @param taskFile Path to task configuration file
     * @param libraryFolder Path for auto-generated CppAD libraries. WBC uses
     *        ``/tmp/ocs2_ros2/<robot>/whole_body_mpc/<hash>`` (override base with
     *        OCS2_PLANNING_URDF_DIR), not the description package share.
     * @param urdfFile Path to URDF file
     * @param frameOverrides Optional in-memory frame overrides (empty fields keep task.info values)
     * Body frame id: `model_information.bodyFrame` (unless overridden).
     * EE frames: `model_information.eeFrame` (left) / `model_information.eeFrame1` (right).
     */
    WholeBodyInterface(const std::string& taskFile,
                           const std::string& libraryFolder,
                           const std::string& urdfFile,
                           const FrameOverrides& frameOverrides = FrameOverrides{});
    
    ~WholeBodyInterface() = default;
    
    //=========================================================================
    // Accessors
    //=========================================================================
    
    const OptimalControlProblem& getOptimalControlProblem() const { return problem_; }
    OptimalControlProblem& getOptimalControlProblem() { return problem_; }
    
    const vector_t& getInitialState() const { return initialState_; }
    
    std::shared_ptr<SwitchedHumanoidReferenceManager> getReferenceManagerPtr() {
        return referenceManagerPtr_;
    }
    
    std::shared_ptr<const SwitchedHumanoidReferenceManager> getReferenceManagerPtr() const {
        return referenceManagerPtr_;
    }
    
    const ddp::Settings& ddpSettings() const { return ddpSettings_; }
    const mpc::Settings& mpcSettings() const { return mpcSettings_; }
    
    const RolloutBase& getRollout() const { return *rolloutPtr_; }
    const Initializer& getInitializer() const { return *initializerPtr_; }
    
    const PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_; }
    const WholeBodyModelInfo& getModelInfo() const { return modelInfo_; }

    /** Copy of collision geometry for visualization (nullptr if self-collision not built). */
    std::unique_ptr<PinocchioGeometryInterface> getPinocchioGeometryInterface() const;

    bool isSelfCollisionEnabled() const { return selfCollisionEnabled_; }
    scalar_t getSelfCollisionMinimumDistance() const { return selfCollisionMinimumDistance_; }
    scalar_t getSelfCollisionActivationDistance() const { return selfCollisionActivationDistance_; }

    //=========================================================================
    // Mode capability flags (derived from config at construction time)
    //=========================================================================

    struct ArmJointLayout {
        size_t leftStart{0};
        size_t leftDof{0};
        size_t rightStart{0};
        size_t rightDof{0};
        bool valid{false};
    };

    const ArmJointLayout& getArmJointLayout() const { return armJointLayout_; }

    bool hasMobileBase() const { return hasMobileBase_; }
    bool hasBodyRelativeConstraint() const { return hasBodyRelativeConstraint_; }
    /** True when WaistJointLockConstraint is active (LOCKED mode). */
    bool hasWaistLock() const { return hasWaistLock_; }
    /** True when HeadCouplingConstraint is active (HEAD_COUPLED mode). */
    bool hasHeadCoupling() const { return hasHeadCoupling_; }
    /** True when implicit head default lock is active (model has head DOF). */
    bool hasHeadDefaultLock() const { return hasHeadDefaultLock_; }
    /** True when CustomJointLockConstraint is active (CUSTOM_LOCKED mode). */
    bool hasCustomJointLock() const { return hasCustomJointLock_; }
    /** True when the head midpoint gaze soft constraint is registered. Independent of body mode / customJointLock.activate. */
    bool isHeadMidpointGazeEnabled() const { return headMidpointGazeEnabled_; }
    bool hasBimanualCoupling() const { return hasBimanualCoupling_; }
    /** True when the 6/7-axis (J6/J7) coupling soft constraint is registered. */
    bool hasJoint67Coupling() const { return hasJoint67Coupling_; }
    bool isBodyTrackingEeEnabled() const { return bodyTrackingEeEnabled_; }
    bool isHeadTrackingEeEnabled() const { return headTrackingEeEnabled_; }
    /** True when HOME joint reference soft constraint is available (switchable). */
    bool hasHomeJointReference() const { return hasHomeJointReference_; }
    bool homeJointReferenceDefaultEnabled() const { return homeJointReferenceDefaultEnabled_; }
    /** Copy controller HOME (arm-local) into the switchable joint reference. */
    void setHomeJointReferenceFromArmJoints(const std::vector<double>& armJoints);

    /**
     * Degree-domain 6/7-axis coupling check for the arm joints inside @p state
     * (same smooth envelope as the OCS2 soft constraint). Empty when disabled.
     * @param state Full OCS2 state (arm joints in the last armDim entries).
     */
    std::vector<mobile_manipulator::Joint67CouplingConstraint::CouplingStatus> checkJoint67Coupling(
        const vector_t& state) const;

private:
    //=========================================================================
    // Setup Functions
    //=========================================================================
    
    void setupOptimalControlProblem(const std::string& taskFile,
                                     const std::string& libraryFolder,
                                     bool usePreComputation,
                                     bool recompileLibraries);
    
    //=========================================================================
    // Cost Functions
    //=========================================================================
    
    std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);

    std::unique_ptr<StateCost> getWaistPhiPositiveBiasCost(const std::string& taskFile);
    
    //=========================================================================
    // Constraint Creation Functions
    //=========================================================================
    
    /**
     * @brief Create end-effector tracking constraint (reuses mobile_manipulator)
     */
    std::unique_ptr<StateCost> getEndEffectorConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);
    
    /**
     * @brief Create one mode-specific body constraint
     */
    std::unique_ptr<StateCost> getBodyTrackingConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix,
        BodyConstraintMode constraintMode,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);

    std::unique_ptr<StateCost> getHeadTrackingConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);

    /**
     * @brief Create head midpoint gaze soft constraint (active in HEAD_GAZE, see
     *        SwitchedHumanoidReferenceManager::isHeadMidpointGazeActive)
     * @param headInputStart/headInputDof Head input window, used only for the startup log line.
     */
    std::unique_ptr<StateCost> getHeadMidpointGazeConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix,
        size_t headInputStart,
        size_t headInputDof,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);

    /**
     * @brief Create bimanual coupling constraint
     */
    std::unique_ptr<StateCost> getBimanualCouplingConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& prefix,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);

    std::unique_ptr<StateCost> getHomeJointReferenceConstraint(const std::string& taskFile);

    /**
     * @brief Create the smooth 6/7-axis (J6/J7) coupling soft constraint for both arms
     *        (Marvin CCS coupled wrist range), using the `joint67Coupling` section.
     * @param defaultLeftArmStart/defaultRightArmStart Arm-segment start index of each arm
     *        (i.e. excluding the base state), used for the default J6/J7 indices.
     */
    std::unique_ptr<StateCost> getJoint67CouplingConstraint(const std::string& taskFile,
                                                            size_t defaultLeftArmStart,
                                                            size_t defaultRightArmStart);
    
    /**
     * @brief Create joint limit soft constraint (reuses mobile_manipulator)
     */
    std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile);
    
    /**
     * @brief Create self-collision constraint (reuses mobile_manipulator)
     */
    std::unique_ptr<StateCost> getSelfCollisionConstraint(
        const PinocchioInterface& pinocchioInterface,
        const std::string& taskFile,
        const std::string& urdfFile,
        const std::string& prefix,
        bool usePreComputation,
        const std::string& libraryFolder,
        bool recompileLibraries);
    
    //=========================================================================
    // Member Variables
    //=========================================================================
    
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    std::unique_ptr<PinocchioGeometryInterface> pinocchioGeometryInterfacePtr_;
    WholeBodyModelInfo modelInfo_;
    
    OptimalControlProblem problem_;
    std::shared_ptr<SwitchedHumanoidReferenceManager> referenceManagerPtr_;
    
    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> initializerPtr_;
    
    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    
    vector_t initialState_;
    
    // Store URDF path for self-collision
    std::string urdfFile_;

    bool selfCollisionEnabled_{false};
    scalar_t selfCollisionMinimumDistance_{0};
    scalar_t selfCollisionActivationDistance_{0};
    bool bodyTrackingEeEnabled_{false};
    bool headTrackingEeEnabled_{false};
    bool hasMobileBase_{false};
    bool hasBodyRelativeConstraint_{false};
    bool hasWaistLock_{false};
    bool hasHeadCoupling_{false};
    bool hasHeadDefaultLock_{false};
    bool hasCustomJointLock_{false};
    bool headMidpointGazeEnabled_{false};
    bool hasBimanualCoupling_{false};
    bool hasJoint67Coupling_{false};
    std::vector<mobile_manipulator::Joint67CouplingConstraint::ArmCoupling> joint67CouplingArms_;
    mobile_manipulator::Joint67CouplingConstraint::Config joint67CouplingConfig_;
    bool hasHomeJointReference_{false};
    bool homeJointReferenceDefaultEnabled_{false};
    ArmJointLayout armJointLayout_{};
};

} // namespace ocs2::whole_body_mpc
