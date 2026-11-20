/******************************************************************************
 * OCS2 Wheel Humanoid - Mode Schedule Definition
 * 
 * Defines the mode system for wheel-based humanoid robots with dual arms.
 * Supports body constraint modes, bimanual coupling, and task switching.
 ******************************************************************************/

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <string>
#include <ostream>
#include <stdexcept>

namespace ocs2::whole_body_mpc {

//=============================================================================
// Body Constraint Mode
//=============================================================================

enum class BodyConstraintMode : uint8_t {
    FREE = 0,           // Body unconstrained (floating base free motion)
    RELATIVE = 1,       // Keep body balanced relative to base (current BodyRelativeConstraint)
    TRACKING = 2,       // Track body target pose from TargetTrajectories
    /** No Cartesian body tracking; waist joint velocities forced to zero (see waistLock in task file). */
    LOCKED = 3,
    /** Head-waist coupling: specific waist joint couples with head joint; others locked.
     *  Requires headCoupling.activate=true in task file (see HeadCouplingConstraint). */
    HEAD_COUPLED = 4,
    /** Arbitrary joint indices are velocity-locked by customJointLock.lockedJointIndices. */
    CUSTOM_LOCKED = 5
};

// Head task, selected independently from the stored body mode.
enum class HeadMode : uint8_t {
    DISABLED = 0,
    TRACKING = 1,
    GAZE = 2,
    FORWARD = 3
};

/**
 * String presets accepted by stringToMode():
 *
 *   HM_DUAL_BODY_FREE
 *       Body free, dual arms independent, mobile base.
 *   HM_DUAL_RELATIVE_MOBILE_BASE
 *       Body relative to the base, dual arms independent, mobile base.
 *   HM_DUAL_BODY_TRACKING
 *       Body and dual arms track targets, mobile base.
 *   HM_DUAL_RELATIVE_FIXED_BASE
 *       Fixed base, body relative constraint, dual arms independent.
 *   HM_DUAL_CUSTOM_LOCK
 *       Lock configured joint indices, mobile base.
 *   HM_DUAL_CUSTOM_LOCK_FIXED_BASE
 *       Fixed base, dual arms independent, lock configured joint indices.
 *   HM_DUAL_IDLE
 *       Body and base locked, dual arms independent.
 *   HM_LEFT_ARM / HM_RIGHT_ARM
 *       Control only the left or right arm, with body and base locked.
 *
 * Head mode commands accepted by applyPartialHumanoidModeString():
 *
 *   HEAD_DISABLE
 *       Disable head control and keep head joints locked.
 *   HEAD_TRACKING
 *       Track the configured head target trajectory.
 *   HEAD_GAZE
 *       Gaze at the midpoint of the two end effectors; requires the
 *       headMidpointGaze configuration.
 *   HEAD_FORWARD
 *       Enable the head-waist coupling constraint; requires headCoupling and
 *       restores the previous body mode when it exits.
 *
 * Body and head modes are independent and can be combined in a mode schedule.
 */

//=============================================================================
// Bimanual Coordination Mode
//=============================================================================

enum class BimanualMode : uint8_t {
    INDEPENDENT = 0,    // Arms track independent targets
    COUPLED = 1         // Arms maintain relative pose constraint (e.g., carrying object)
};

//=============================================================================
// Humanoid Mode Structure
//=============================================================================

/**
 * @brief Complete mode state for wheel humanoid robot
 * 
 * This struct captures the full operational mode of the robot:
 * - Body constraint mode (how the body/torso is controlled)
 * - Which arms are active
 * - How the arms coordinate (independent vs coupled)
 * - Whether the mobile base is locked (velocity = 0)
 */
struct HumanoidMode {
    BodyConstraintMode bodyMode = BodyConstraintMode::RELATIVE;
    bool leftArmActive = true;
    bool rightArmActive = true;
    BimanualMode bimanualMode = BimanualMode::INDEPENDENT;
    bool baseLocked = false;  // If true, base velocity is constrained to zero
    HeadMode headMode = HeadMode::DISABLED;
    
    /** Forward temporarily uses the complete legacy head/waist constraint set. */
    [[nodiscard]] constexpr BodyConstraintMode effectiveBodyMode() const {
        return headMode == HeadMode::FORWARD ? BodyConstraintMode::HEAD_COUPLED : bodyMode;
    }

    /** Bits 0-2: body; 3-4: arms; 5: bimanual; 6: base lock; 7-8: head mode. */
    [[nodiscard]] constexpr size_t encode() const {
        auto body = bodyMode;
        auto head = headMode;
        // Preserve old presets and commands, but keep new head/body state separate.
        if (body == BodyConstraintMode::HEAD_COUPLED) {
            body = BodyConstraintMode::FREE;
            head = HeadMode::FORWARD;
        }
        if (static_cast<size_t>(body) > 5 || static_cast<size_t>(head) > 3 ||
            static_cast<size_t>(bimanualMode) > 1) {
            throw std::invalid_argument("Invalid humanoid mode");
        }
        return static_cast<size_t>(body) |
               (leftArmActive ? 0x08 : 0) |
               (rightArmActive ? 0x10 : 0) |
               (static_cast<size_t>(bimanualMode) << 5) |
               (baseLocked ? 0x40 : 0) |
               (static_cast<size_t>(head) << 7);
    }

    static constexpr HumanoidMode decode(size_t mode) {
        if (mode > 511 || (mode & 0x07) > 5) {
            throw std::invalid_argument("Invalid humanoid mode bits");
        }
        HumanoidMode m;
        m.bodyMode = static_cast<BodyConstraintMode>(mode & 0x07);
        m.leftArmActive = (mode & 0x08) != 0;
        m.rightArmActive = (mode & 0x10) != 0;
        m.bimanualMode = static_cast<BimanualMode>((mode >> 5) & 0x01);
        m.baseLocked = (mode & 0x40) != 0;
        m.headMode = static_cast<HeadMode>((mode >> 7) & 0x03);
        if (m.bodyMode == BodyConstraintMode::HEAD_COUPLED) {
            if (m.headMode != HeadMode::DISABLED) {
                throw std::invalid_argument("Legacy HEAD_COUPLED cannot include another head mode");
            }
            m.bodyMode = BodyConstraintMode::FREE;
            m.headMode = HeadMode::FORWARD;
        }
        return m;
    }

    constexpr bool operator==(const HumanoidMode& other) const {
        return bodyMode == other.bodyMode &&
               leftArmActive == other.leftArmActive &&
               rightArmActive == other.rightArmActive &&
               bimanualMode == other.bimanualMode &&
               baseLocked == other.baseLocked &&
               headMode == other.headMode;
    }
};

//=============================================================================
// Predefined Mode Constants
//=============================================================================

namespace Modes {
    // Single arm: base locked + body locked + independent (no coupling)
    inline constexpr size_t LEFT_ARM_ONLY =
        HumanoidMode{BodyConstraintMode::LOCKED, true, false, BimanualMode::INDEPENDENT, true}.encode();

    inline constexpr size_t RIGHT_ARM_ONLY =
        HumanoidMode{BodyConstraintMode::LOCKED, false, true, BimanualMode::INDEPENDENT, true}.encode();
    
    // Dual arm independent (each arm tracks its own target, base mobile)
    inline constexpr size_t DUAL_INDEPENDENT = 
        HumanoidMode{BodyConstraintMode::RELATIVE, true, true, BimanualMode::INDEPENDENT, false}.encode();
    
    // Dual arm coupled (arms maintain relative pose, base mobile)
    inline constexpr size_t DUAL_COUPLED = 
        HumanoidMode{BodyConstraintMode::RELATIVE, true, true, BimanualMode::COUPLED, false}.encode();
    
    // Whole body tracking (body + dual arms all track targets, base mobile)
    inline constexpr size_t WHOLE_BODY = 
        HumanoidMode{BodyConstraintMode::TRACKING, true, true, BimanualMode::INDEPENDENT, false}.encode();
    
    // Whole body with coupled arms (base mobile)
    inline constexpr size_t WHOLE_BODY_COUPLED = 
        HumanoidMode{BodyConstraintMode::TRACKING, true, true, BimanualMode::COUPLED, false}.encode();
        
    // Body free, dual arms independent (base mobile); string preset: HM_DUAL_BODY_FREE
    inline constexpr size_t BODY_FREE_DUAL = 
        HumanoidMode{BodyConstraintMode::FREE, true, true, BimanualMode::INDEPENDENT, false}.encode();

    // Dual arm independent, body locked (no torso motion), base mobile
    inline constexpr size_t DUAL_INDEPENDENT_LOCKED =
        HumanoidMode{BodyConstraintMode::LOCKED, true, true, BimanualMode::INDEPENDENT, false}.encode();

    // Dual arm coupled, body locked
    inline constexpr size_t DUAL_COUPLED_LOCKED =
        HumanoidMode{BodyConstraintMode::LOCKED, true, true, BimanualMode::COUPLED, false}.encode();

    /** Default idle: body lock + base lock + dual independent (string preset: HM_DUAL_IDLE). */
    inline constexpr size_t FIXED_DUAL_INDEPENDENT_LOCKED =
        HumanoidMode{BodyConstraintMode::LOCKED, true, true, BimanualMode::INDEPENDENT, true}.encode();

    // Custom joint-index lock: base mobile
    inline constexpr size_t DUAL_CUSTOM_LOCK =
        HumanoidMode{BodyConstraintMode::CUSTOM_LOCKED, true, true, BimanualMode::INDEPENDENT, false}.encode();

    // Custom joint-index lock: base locked
    inline constexpr size_t FIXED_DUAL_CUSTOM_LOCK =
        HumanoidMode{BodyConstraintMode::CUSTOM_LOCKED, true, true, BimanualMode::INDEPENDENT, true}.encode();
    
    //=========================================================================
    // Base locked variants (stationary base, only arms move)
    //=========================================================================
    
    // Dual arm independent with locked base
    inline constexpr size_t FIXED_DUAL_INDEPENDENT = 
        HumanoidMode{BodyConstraintMode::RELATIVE, true, true, BimanualMode::INDEPENDENT, true}.encode();
    
    // Dual arm coupled with locked base
    inline constexpr size_t FIXED_DUAL_COUPLED = 
        HumanoidMode{BodyConstraintMode::RELATIVE, true, true, BimanualMode::COUPLED, true}.encode();
    
    // Same as LEFT_ARM_ONLY / RIGHT_ARM_ONLY (string presets: HM_LEFT_ARM / HM_RIGHT_ARM)
    inline constexpr size_t FIXED_LEFT_ARM = LEFT_ARM_ONLY;

    inline constexpr size_t FIXED_RIGHT_ARM = RIGHT_ARM_ONLY;
}

//=============================================================================
// Utility Functions
//=============================================================================

/**
 * @brief Convert mode to human-readable string
 */
std::string modeToString(size_t mode);

/**
 * @brief Body/arms/base preset: HM_* names only; head defaults to DISABLED. Topic PRESET: preserves current head.
 */
size_t stringToMode(const std::string& name);

/**
 * @brief Normalize topic command: trim whitespace, replace spaces with underscores (e.g. "BASE UNLOCK" -> "BASE_UNLOCK").
 */
std::string normalizeHumanoidModeCommandName(std::string s);

/**
 * @brief Partial update (topic default, no prefix):
 * BASE_UNLOCK, BASE_LOCK;
 * BODY_FREE, BODY_RELATIVE, BODY_TRACKING, BODY_LOCK, BODY_UNLOCK,
 * BODY_CUSTOM_LOCK;
 * HEAD_DISABLE, HEAD_TRACKING, HEAD_GAZE, HEAD_FORWARD;
 * ARMS_COUPLED, ARMS_INDEPENDENT;
 * LEFT_ARM_ENABLE, LEFT_ARM_DISABLE, RIGHT_ARM_ENABLE, RIGHT_ARM_DISABLE.
 * HOME_JOINT_ON / HOME_JOINT_OFF are handled by the WBC controller, not here.
 */
size_t applyPartialHumanoidModeString(size_t currentMode, const std::string& name);

/**
 * @brief Resolve topic line after normalizeHumanoidModeCommandName:
 * - Default: partial (applyPartialHumanoidModeString).
 * - PRESET: + HM_* replaces body/arms/base and preserves the current head mode.
 */
size_t resolveHumanoidModeCommand(size_t currentMode, const std::string& normalizedName);

/**
 * @brief Convert BodyConstraintMode to string
 */
std::string bodyModeToString(BodyConstraintMode mode);
std::string headModeToString(HeadMode mode);

/**
 * @brief Convert BimanualMode to string
 */
std::string bimanualModeToString(BimanualMode mode);

/**
 * @brief Stream output operator for HumanoidMode
 */
std::ostream& operator<<(std::ostream& os, const HumanoidMode& mode);

/**
 * @brief Load initial mode schedule from config file
 */
ModeSchedule loadHumanoidModeSchedule(const std::string& filename, 
                                       const std::string& topicName, 
                                       bool verbose = false);

} // namespace ocs2::whole_body_mpc
