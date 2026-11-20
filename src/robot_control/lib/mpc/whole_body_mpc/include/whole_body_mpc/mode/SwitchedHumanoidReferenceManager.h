/******************************************************************************
 * OCS2 Wheel Humanoid - Switched Reference Manager
 * 
 * Extends ReferenceManager to support mode-dependent behavior.
 * Provides convenient accessors for current mode and target interpolation.
 ******************************************************************************/

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>

#include <ocs2_core/Types.h>
#include <mpc_core/reference/BimanualCouplingSource.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "whole_body_mpc/mode/HumanoidModeSchedule.h"

namespace ocs2::whole_body_mpc {

/**
 * @brief Reference manager with mode-aware target trajectory handling
 * 
 * Target Trajectory Layout (stateTrajectory):
 * ============================================
 * Quaternion blocks are [qx, qy, qz, qw] (Eigen coeffs / geometry_msgs), NOT [qw, qx, qy, qz].
 *
 * 14D mode (bodyTrackingEE disabled, same as mobile_manipulator dual-arm):
 *   [0:3]   Left arm position (x, y, z)
 *   [3:7]   Left arm orientation quaternion (x, y, z, w)
 *   [7:10]  Right arm position (x, y, z)
 *   [10:14] Right arm orientation quaternion (x, y, z, w)
 * 
 * Optional pose blocks are appended in capability order: body, then head.
 * Body-only or head-only therefore uses 21D; body + head uses 28D.
 *
 * Body block (when enabled):
 *   [0:14]  Same as 14D
 *   [bodyOffset:bodyOffset+7] Body pose
 * Head block (when enabled):
 *   [headOffset:headOffset+7] Head pose
 * 
 * Coupling Parameters (captured via captureCouplingFromCurrentPose; optional in inputTrajectory):
 * ==============================================================================================
 * Stored in the left EE frame (not world-frame right-left):
 * [0:3]   Relative position in left EE frame
 * [3:7]   Relative orientation quaternion in left EE frame (x, y, z, w)
 * Runtime constraint anchors on the *measured* left EE (see BimanualCouplingConstraint).
 */
class SwitchedHumanoidReferenceManager : public ReferenceManager, public mpc_core::BimanualCouplingSource {
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;
    
    //=========================================================================
    // Target trajectory dimension constants
    //=========================================================================
    static constexpr int LEFT_ARM_POS_OFFSET = 0;
    static constexpr int LEFT_ARM_QUAT_OFFSET = 3;
    static constexpr int RIGHT_ARM_POS_OFFSET = 7;
    static constexpr int RIGHT_ARM_QUAT_OFFSET = 10;
    static constexpr int DUAL_ARM_TARGET_DIM = 14;
    static constexpr int POSE_TARGET_DIM = 7;
    
    // Coupling parameters (in inputTrajectory)
    static constexpr int COUPLING_POS_OFFSET = 0;
    static constexpr int COUPLING_QUAT_OFFSET = 3;
    static constexpr int COUPLING_PARAM_DIM = 7;
    
    //=========================================================================
    // Constructors
    //=========================================================================
    
    SwitchedHumanoidReferenceManager();
    
    explicit SwitchedHumanoidReferenceManager(
        TargetTrajectories initialTargetTrajectories,
        ModeSchedule initialModeSchedule = ModeSchedule(),
        bool bodyTargetEnabled = false,
        bool headTargetEnabled = false,
        bool headMidpointGazeEnabled = false);
    
    ~SwitchedHumanoidReferenceManager() override = default;
    
    //=========================================================================
    // Mode Query Interface
    //=========================================================================
    
    /**
     * @brief Get the full humanoid mode at given time
     */
    [[nodiscard]] HumanoidMode getHumanoidMode(scalar_t time) const {
        return HumanoidMode::decode(getModeSchedule().modeAtTime(time));
    }
    
    /**
     * @brief Get body constraint mode at given time
     */
    [[nodiscard]] BodyConstraintMode getBodyMode(scalar_t time) const {
        return getHumanoidMode(time).effectiveBodyMode();
    }
    
    /**
     * @brief Check if left arm is active at given time
     */
    [[nodiscard]] bool isHeadMidpointGazeEnabled() const { return headMidpointGazeEnabled_; }
    [[nodiscard]] bool isHeadMidpointGazeActive(scalar_t time) const {
        return headMidpointGazeEnabled_ && getHumanoidMode(time).headMode == HeadMode::GAZE;
    }
    void validateMode(size_t mode) const;
    void setModeSchedule(const ModeSchedule& schedule) override;
    void setModeSchedule(ModeSchedule&& schedule) override;

    [[nodiscard]] bool isHeadActive(scalar_t time) const {
        return getHumanoidMode(time).headMode == HeadMode::TRACKING;
    }

    [[nodiscard]] bool isLeftArmActive(scalar_t time) const {
        return getHumanoidMode(time).leftArmActive;
    }
    
    /**
     * @brief Check if right arm is active at given time
     */
    [[nodiscard]] bool isRightArmActive(scalar_t time) const {
        return getHumanoidMode(time).rightArmActive;
    }
    
    /**
     * @brief Check if bimanual coupling is enabled at given time
     */
    [[nodiscard]] bool isBimanualCoupled(scalar_t time) const override {
        return getHumanoidMode(time).bimanualMode == BimanualMode::COUPLED;
    }
    
    /**
     * @brief Check if mobile base is locked at given time
     */
    [[nodiscard]] bool isBaseLocked(scalar_t time) const {
        return getHumanoidMode(time).baseLocked;
    }

    /** True when BodyConstraintMode::LOCKED (waist joint velocities locked). */
    [[nodiscard]] bool isBodyLocked(scalar_t time) const {
        return getBodyMode(time) == BodyConstraintMode::LOCKED;
    }
    
    //=========================================================================
    // Target Interpolation Interface
    //=========================================================================
    
    /**
     * @brief Interpolate left arm target pose at given time
     * @return pair of (position, quaternion)
     */
    [[nodiscard]] std::pair<vector3_t, quaternion_t> 
    interpolateLeftArmPose(scalar_t time) const;
    
    /**
     * @brief Interpolate right arm target pose at given time
     * @return pair of (position, quaternion)
     */
    [[nodiscard]] std::pair<vector3_t, quaternion_t> 
    interpolateRightArmPose(scalar_t time) const;
    
    /**
     * @brief Interpolate body target pose at given time
     * @return pair of (position, quaternion)
     */
    [[nodiscard]] std::pair<vector3_t, quaternion_t> 
    interpolateBodyPose(scalar_t time) const;

    [[nodiscard]] std::pair<vector3_t, quaternion_t>
    interpolateHeadPose(scalar_t time) const;

    [[nodiscard]] bool isBodyTargetEnabled() const { return bodyTargetEnabled_; }
    [[nodiscard]] bool isHeadTargetEnabled() const { return headTargetEnabled_; }
    [[nodiscard]] int getBodyTargetOffset() const { return bodyTargetOffset_; }
    [[nodiscard]] int getHeadTargetOffset() const { return headTargetOffset_; }
    [[nodiscard]] int getTargetStateDim() const { return targetStateDim_; }
    
    /**
     * @brief Coupling offset in the left EE frame (from capture, else default).
     */
    [[nodiscard]] std::pair<vector3_t, quaternion_t> 
    getCouplingParameters(scalar_t time) const override;
    
    /**
     * @brief Set default coupling parameters for bimanual mode
     */
    void setDefaultCouplingOffset(const vector3_t& position, 
                                   const quaternion_t& orientation);
    
    /**
     * @brief Capture relative pose in the left EE frame for bimanual coupling.
     * Call when entering COUPLED; runtime constraint follows the measured left EE.
     */
    void captureCouplingFromCurrentPose(const vector3_t& leftPos, 
                                         const quaternion_t& leftQuat,
                                         const vector3_t& rightPos, 
                                         const quaternion_t& rightQuat);
    
    /**
     * @brief Check if coupling has been captured
     */
    [[nodiscard]] bool hasCapturedCoupling() const override { return couplingCaptured_; }
    
    /**
     * @brief Clear captured coupling (will use default again)
     */
    void clearCapturedCoupling() { couplingCaptured_ = false; }

    /**
     * Switchable HOME joint reference (not encoded in HumanoidMode).
     * q_home is arm-local, same order as controller joints / armDim.
     */
    void setHomeJointReferenceEnabled(bool enabled) {
        homeJointReferenceEnabled_.store(enabled, std::memory_order_relaxed);
    }

    [[nodiscard]] bool isHomeJointReferenceEnabled() const {
        return homeJointReferenceEnabled_.load(std::memory_order_relaxed);
    }

    void setHomeJointReference(const vector_t& armJoints) { homeJointReference_ = armJoints; }

    [[nodiscard]] const vector_t& getHomeJointReference() const { return homeJointReference_; }

    /**
     * Base velocity the operator asks for (diff base: [v, omega]; omni base: [vx, vy, omega]).
     * modifyReferences writes it into the desired base input of every solve, so the input cost pulls the base
     * towards it. A request older than the timeout (or none yet) counts as zero.
     */
    void setBaseInputDim(size_t dim);
    void setBaseVelocityReferenceTimeout(scalar_t seconds);
    void setBaseVelocityReference(const vector_t& velocity);

protected:
    /**
     * @brief Override to add custom reference modification logic
     */
    void modifyReferences(scalar_t initTime, scalar_t finalTime,
                          const vector_t& initState,
                          TargetTrajectories& targetTrajectories,
                          ModeSchedule& modeSchedule) override;

private:
    // Helper for quaternion interpolation
    [[nodiscard]] std::pair<vector3_t, quaternion_t> 
    interpolatePose(scalar_t time, int posOffset, int quatOffset) const;
    
    // Default coupling parameters (used if not captured)
    vector3_t defaultCouplingPosition_ = vector3_t(0.6, 0.0, 0.0);  // 60cm apart
    quaternion_t defaultCouplingOrientation_ = quaternion_t::Identity();
    
    // Captured coupling parameters (from current pose when switching to COUPLED)
    bool couplingCaptured_ = false;
    vector3_t capturedCouplingPosition_ = vector3_t::Zero();
    quaternion_t capturedCouplingOrientation_ = quaternion_t::Identity();

    std::atomic<bool> homeJointReferenceEnabled_{false};
    vector_t homeJointReference_;

    // The operator's base velocity: set from a ROS callback, read by the solver thread.
    mutable std::mutex baseVelocityMutex_;
    size_t baseInputDim_{0};  // 0 = fixed base
    vector_t baseVelocityReference_;
    std::chrono::steady_clock::time_point baseVelocityStamp_{};
    scalar_t baseVelocityTimeout_{0.2};
    [[nodiscard]] vector_t currentBaseVelocityReference() const;

    bool headMidpointGazeEnabled_{false};
    bool bodyTargetEnabled_{false};
    bool headTargetEnabled_{false};
    int bodyTargetOffset_{-1};
    int headTargetOffset_{-1};
    int targetStateDim_{DUAL_ARM_TARGET_DIM};
};

} // namespace ocs2::whole_body_mpc
