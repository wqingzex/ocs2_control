/******************************************************************************
 * OCS2 Wheel Humanoid - Switched Reference Manager Implementation
 ******************************************************************************/

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"
#include <rclcpp/logging.hpp>

#include <iostream>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2::whole_body_mpc {

//=============================================================================
// Constructors
//=============================================================================

SwitchedHumanoidReferenceManager::SwitchedHumanoidReferenceManager()
    : ReferenceManager(TargetTrajectories(), ModeSchedule({}, {Modes::FIXED_DUAL_INDEPENDENT_LOCKED})) {
}

SwitchedHumanoidReferenceManager::SwitchedHumanoidReferenceManager(
    TargetTrajectories initialTargetTrajectories,
    ModeSchedule initialModeSchedule,
    bool bodyTargetEnabled,
    bool headTargetEnabled,
    bool headMidpointGazeEnabled)
    : ReferenceManager(std::move(initialTargetTrajectories), std::move(initialModeSchedule)),
      headMidpointGazeEnabled_(headMidpointGazeEnabled),
      bodyTargetEnabled_(bodyTargetEnabled),
      headTargetEnabled_(headTargetEnabled) {
    for (const auto mode : getModeSchedule().modeSequence) validateMode(mode);
    int offset = DUAL_ARM_TARGET_DIM;
    if (bodyTargetEnabled_) {
        bodyTargetOffset_ = offset;
        offset += POSE_TARGET_DIM;
    }
    if (headTargetEnabled_) {
        headTargetOffset_ = offset;
        offset += POSE_TARGET_DIM;
    }
    targetStateDim_ = offset;
}

void SwitchedHumanoidReferenceManager::validateMode(size_t mode) const {
    const auto hm = HumanoidMode::decode(mode);
    if (hm.headMode == HeadMode::GAZE && !headMidpointGazeEnabled_) {
        throw std::invalid_argument("Head midpoint gaze is not configured");
    }
}

void SwitchedHumanoidReferenceManager::setModeSchedule(const ModeSchedule& schedule) {
    for (const auto mode : schedule.modeSequence) validateMode(mode);
    ReferenceManager::setModeSchedule(schedule);
}

void SwitchedHumanoidReferenceManager::setModeSchedule(ModeSchedule&& schedule) {
    for (const auto mode : schedule.modeSequence) validateMode(mode);
    ReferenceManager::setModeSchedule(std::move(schedule));
}

//=============================================================================
// Target Interpolation
//=============================================================================

std::pair<SwitchedHumanoidReferenceManager::vector3_t, 
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::interpolatePose(scalar_t time, 
                                                   int posOffset, 
                                                   int quatOffset) const {
    const auto& targetTrajectories = getTargetTrajectories();
    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;
    
    if (stateTrajectory.empty()) {
        return {vector3_t::Zero(), quaternion_t::Identity()};
    }
    
    vector3_t position;
    quaternion_t orientation;
    
    if (stateTrajectory.size() > 1) {
        // Linear interpolation for position, slerp for quaternion
        int index;
        scalar_t alpha;
        std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);
        
        const auto& lhs = stateTrajectory[index];
        const auto& rhs = stateTrajectory[index + 1];
        
        // Position interpolation
        position = alpha * lhs.segment<3>(posOffset) + 
                   (1.0 - alpha) * rhs.segment<3>(posOffset);
        
        // Quaternion storage is [qx, qy, qz, qw] (Eigen coeffs / geometry_msgs), matching PoseBasedReferenceManager.
        const auto quatLhs = lhs.segment<4>(quatOffset);
        const auto quatRhs = rhs.segment<4>(quatOffset);
        const quaternion_t q_lhs(quatLhs(3), quatLhs(0), quatLhs(1), quatLhs(2));
        const quaternion_t q_rhs(quatRhs(3), quatRhs(0), quatRhs(1), quatRhs(2));
        orientation = q_lhs.slerp(1.0 - alpha, q_rhs);
    } else {
        // Single point
        position = stateTrajectory.front().segment<3>(posOffset);
        const auto quat = stateTrajectory.front().segment<4>(quatOffset);
        orientation = quaternion_t(quat(3), quat(0), quat(1), quat(2));
    }
    
    return {position, orientation};
}

std::pair<SwitchedHumanoidReferenceManager::vector3_t, 
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::interpolateLeftArmPose(scalar_t time) const {
    return interpolatePose(time, LEFT_ARM_POS_OFFSET, LEFT_ARM_QUAT_OFFSET);
}

std::pair<SwitchedHumanoidReferenceManager::vector3_t, 
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::interpolateRightArmPose(scalar_t time) const {
    return interpolatePose(time, RIGHT_ARM_POS_OFFSET, RIGHT_ARM_QUAT_OFFSET);
}

std::pair<SwitchedHumanoidReferenceManager::vector3_t, 
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::interpolateBodyPose(scalar_t time) const {
    const auto& stateTrajectory = getTargetTrajectories().stateTrajectory;
    if (!bodyTargetEnabled_ || stateTrajectory.empty() ||
        stateTrajectory.front().size() < bodyTargetOffset_ + POSE_TARGET_DIM) {
        return {vector3_t::Zero(), quaternion_t::Identity()};
    }
    return interpolatePose(time, bodyTargetOffset_, bodyTargetOffset_ + 3);
}

std::pair<SwitchedHumanoidReferenceManager::vector3_t,
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::interpolateHeadPose(scalar_t time) const {
    const auto& stateTrajectory = getTargetTrajectories().stateTrajectory;
    if (!headTargetEnabled_ || stateTrajectory.empty() ||
        stateTrajectory.front().size() < headTargetOffset_ + POSE_TARGET_DIM) {
        return {vector3_t::Zero(), quaternion_t::Identity()};
    }
    return interpolatePose(time, headTargetOffset_, headTargetOffset_ + 3);
}

//=============================================================================
// Coupling Parameters
//=============================================================================

std::pair<SwitchedHumanoidReferenceManager::vector3_t, 
          SwitchedHumanoidReferenceManager::quaternion_t>
SwitchedHumanoidReferenceManager::getCouplingParameters(scalar_t time) const {
    // Priority 1: Check if inputTrajectory has explicit coupling parameters (from TargetNode)
    const auto& targetTrajectories = getTargetTrajectories();
    const auto& inputTrajectory = targetTrajectories.inputTrajectory;
    
    if (!inputTrajectory.empty() && 
        inputTrajectory.front().size() >= COUPLING_PARAM_DIM) {
        
        // Check if coupling parameters are valid (not all zeros)
        const auto& firstInput = inputTrajectory.front();
        scalar_t quatNormSq = firstInput.segment<4>(COUPLING_QUAT_OFFSET).squaredNorm();
        
        // Valid quaternion should have norm close to 1
        if (quatNormSq > 0.5) {
            const auto& timeTrajectory = targetTrajectories.timeTrajectory;
            
            if (inputTrajectory.size() > 1) {
                int index;
                scalar_t alpha;
                std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);
                
                const auto& lhs = inputTrajectory[index];
                const auto& rhs = inputTrajectory[index + 1];
                
                vector3_t position = alpha * lhs.segment<3>(COUPLING_POS_OFFSET) + 
                                     (1.0 - alpha) * rhs.segment<3>(COUPLING_POS_OFFSET);
                
                const quaternion_t q_lhs(lhs.segment<4>(COUPLING_QUAT_OFFSET));
                const quaternion_t q_rhs(rhs.segment<4>(COUPLING_QUAT_OFFSET));
                quaternion_t orientation = q_lhs.slerp(1.0 - alpha, q_rhs);
                
                return {position, orientation};
            } else {
                vector3_t position = inputTrajectory.front().segment<3>(COUPLING_POS_OFFSET);
                quaternion_t orientation(inputTrajectory.front().segment<4>(COUPLING_QUAT_OFFSET));
                return {position, orientation};
            }
        }
    }
    
    // Priority 2: Use captured coupling from captureCouplingFromCurrentPose()
    if (couplingCaptured_) {
        return {capturedCouplingPosition_, capturedCouplingOrientation_};
    }
    
    // Priority 3: Use default coupling parameters
    return {defaultCouplingPosition_, defaultCouplingOrientation_};
}

void SwitchedHumanoidReferenceManager::setDefaultCouplingOffset(
    const vector3_t& position, 
    const quaternion_t& orientation) {
    defaultCouplingPosition_ = position;
    defaultCouplingOrientation_ = orientation;
}

void SwitchedHumanoidReferenceManager::captureCouplingFromCurrentPose(
    const vector3_t& leftPos, 
    const quaternion_t& leftQuat,
    const vector3_t& rightPos, 
    const quaternion_t& rightQuat) {
    
    // Compute relative position in LEFT ARM's local frame
    // This way, when left arm moves, the relative position in local frame stays constant
    // and right arm will follow
    vector3_t worldRelPos = rightPos - leftPos;
    capturedCouplingPosition_ = leftQuat.inverse() * worldRelPos;  // Transform to left arm's frame
    
    // Compute relative orientation: inv(left) * right
    capturedCouplingOrientation_ = leftQuat.inverse() * rightQuat;
    
    // Ensure quaternion is in positive hemisphere
    if (capturedCouplingOrientation_.w() < 0) {
        capturedCouplingOrientation_.coeffs() = -capturedCouplingOrientation_.coeffs();
    }
    
    couplingCaptured_ = true;
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SwitchedHumanoidReferenceManager"), "Captured coupling (in left arm's local frame):\n"
              << "  Relative position: " << capturedCouplingPosition_.transpose() << "\n"
              << "  Relative orientation (w,x,y,z): " 
              << capturedCouplingOrientation_.w() << ", "
              << capturedCouplingOrientation_.x() << ", "
              << capturedCouplingOrientation_.y() << ", "
              << capturedCouplingOrientation_.z());
}

//=============================================================================
// Reference Modification
//=============================================================================

void SwitchedHumanoidReferenceManager::modifyReferences(
    scalar_t initTime, scalar_t finalTime,
    const vector_t& initState,
    TargetTrajectories& targetTrajectories,
    ModeSchedule& modeSchedule) {
    
    for (const auto mode : modeSchedule.modeSequence) validateMode(mode);

    // The operator's base velocity is the desired base input of every solve, whichever code rebuilt the targets.
    // Only the base entries change; the coupling parameters in the following entries stay as they are.
    if (baseInputDim_ > 0) {
        const vector_t reference = currentBaseVelocityReference();
        for (auto& input : targetTrajectories.inputTrajectory) {
            if (input.size() < reference.size()) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("SwitchedHumanoidReferenceManager"),
                                   "Desired input (" << input.size() << ") is shorter than the base input ("
                                                     << reference.size() << "): base velocity reference not applied");
                break;
            }
            input.head(reference.size()) = reference;
        }
    }

    if (!targetTrajectories.empty()) {
        for (const auto& state : targetTrajectories.stateTrajectory) {
            if (state.size() != targetStateDim_) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("SwitchedHumanoidReferenceManager"), "Warning: "
                          << "Target state dimension (" << state.size()
                          << ") does not match enabled layout (" << targetStateDim_ << ")");
                break;
            }
        }
    }
}

void SwitchedHumanoidReferenceManager::setBaseInputDim(size_t dim) {
    std::lock_guard<std::mutex> lock(baseVelocityMutex_);
    baseInputDim_ = dim;
}

void SwitchedHumanoidReferenceManager::setBaseVelocityReferenceTimeout(scalar_t seconds) {
    std::lock_guard<std::mutex> lock(baseVelocityMutex_);
    baseVelocityTimeout_ = seconds;
}

void SwitchedHumanoidReferenceManager::setBaseVelocityReference(const vector_t& velocity) {
    std::lock_guard<std::mutex> lock(baseVelocityMutex_);
    baseVelocityReference_ = velocity;
    baseVelocityStamp_ = std::chrono::steady_clock::now();
}

vector_t SwitchedHumanoidReferenceManager::currentBaseVelocityReference() const {
    std::lock_guard<std::mutex> lock(baseVelocityMutex_);
    const auto age = std::chrono::duration<scalar_t>(std::chrono::steady_clock::now() - baseVelocityStamp_).count();
    if (baseVelocityReference_.size() == static_cast<Eigen::Index>(baseInputDim_) && age <= baseVelocityTimeout_) {
        return baseVelocityReference_;
    }
    return vector_t::Zero(static_cast<Eigen::Index>(baseInputDim_));
}

} // namespace ocs2::whole_body_mpc
