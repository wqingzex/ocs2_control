//
#include "whole_body_mpc/WholeBodyMpcTask.h"
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <mpc_core/rt/ThreadScheduling.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include <trajectory_manager/TrajectoryRecorder.h>
#include <exception>
#include <filesystem>
#include <chrono>
#include <mutex>
#include <optional>

namespace ocs2::whole_body_mpc {

namespace {

constexpr const char* kCppadCacheSchema = "cppad-cache-v1";

void fnv1aUpdate(std::uint64_t& hash, const char* data, std::streamsize size)
{
    for (std::streamsize i = 0; i < size; ++i) {
        hash ^= static_cast<unsigned char>(data[i]);
        hash *= 1099511628211ull;
    }
}

std::string fnv1aFile(const std::string& path)
{
    std::uint64_t hash = 14695981039346656037ull;
    fnv1aUpdate(hash, kCppadCacheSchema, static_cast<std::streamsize>(std::char_traits<char>::length(kCppadCacheSchema)));
    std::ifstream input(path, std::ios::binary);
    char buffer[4096];
    while (input) {
        input.read(buffer, sizeof buffer);
        fnv1aUpdate(hash, buffer, input.gcount());
    }
    std::ostringstream out;
    out << std::hex << hash;
    return out.str();
}

std::string cppadCacheBase()
{
    if (const char* override_dir = std::getenv("OCS2_PLANNING_URDF_DIR");
        override_dir != nullptr && override_dir[0] != '\0') {
        std::string base(override_dir);
        while (!base.empty() && base.back() == '/') {
            base.pop_back();
        }
        return base;
    }
    return "/tmp";
}

}  // namespace

std::string cppadLibraryFolder(const std::string& robot_name, const std::string& subdir,
                                  const std::string& task_file, const std::string& urdf_file)
{
    const std::string hash = fnv1aFile(task_file) + fnv1aFile(urdf_file);
    return cppadCacheBase() + "/ocs2_ros2/" + robot_name + "/" + subdir + "/" + hash;
}

namespace {
double wrapToPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

FrameOverrides loadInfoFrameDefaults(const std::string& task_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(task_file, pt);
    FrameOverrides frames;
    loadData::loadPtreeValue<std::string>(pt, frames.baseFrame, "model_information.baseFrame", false);
    loadData::loadPtreeValue<std::string>(pt, frames.eeFrame, "model_information.eeFrame", false);
    loadData::loadPtreeValue<std::string>(pt, frames.eeFrame1, "model_information.eeFrame1", false);
    if (const auto configuredBodyFrame = pt.get_optional<std::string>("model_information.bodyFrame")) {
        frames.bodyFrame = *configuredBodyFrame;
    } else if (const auto legacyBodyFrame = pt.get_optional<std::string>("bodyRelative.bodyLinkName")) {
        frames.bodyFrame = *legacyBodyFrame;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("WholeBodyMpcTask"), "Deprecated config key bodyRelative.bodyLinkName; "
                     "use model_information.bodyFrame instead.");
    }
    if (const auto configuredHeadFrame = pt.get_optional<std::string>("model_information.headFrame")) {
        frames.headFrame = *configuredHeadFrame;
    }
    bool recompile_libraries = true;
    loadData::loadPtreeValue(pt, recompile_libraries, "model_settings.recompileLibraries", false);
    frames.recompileLibraries = recompile_libraries;
    return frames;
}

bool WholeBodyMpcTask::isHumanoidModeStructurallySupported(size_t mode) const {
    const auto hm = HumanoidMode::decode(mode);
    if (!hm.baseLocked && !interface_->hasMobileBase()) {
        return false;
    }
    if (hm.headMode == HeadMode::TRACKING && !interface_->isHeadTrackingEeEnabled()) return false;
    if (hm.headMode == HeadMode::GAZE && !interface_->isHeadMidpointGazeEnabled()) return false;
    if (hm.headMode == HeadMode::FORWARD && !interface_->hasHeadCoupling()) return false;
    switch (hm.bodyMode) {
        case BodyConstraintMode::RELATIVE:
            if (!interface_->hasBodyRelativeConstraint()) return false;
            break;
        case BodyConstraintMode::TRACKING:
            if (!interface_->isBodyTrackingEeEnabled()) return false;
            break;
        case BodyConstraintMode::LOCKED:
            if (!interface_->hasWaistLock()) return false;
            break;
        case BodyConstraintMode::HEAD_COUPLED:
            if (!interface_->hasHeadCoupling()) return false;
            break;
        case BodyConstraintMode::CUSTOM_LOCKED:
            if (!interface_->hasCustomJointLock()) return false;
            break;
        default:
            break;
    }
    if (hm.bimanualMode == BimanualMode::COUPLED && !interface_->hasBimanualCoupling()) {
        return false;
    }
    return true;
}

bool WholeBodyMpcTask::isHumanoidModeSupported(size_t mode) const {
    if (!isHumanoidModeStructurallySupported(mode)) {
        return false;
    }
    // Unlocked-base planning also needs confirmed external world->base TF.
    const auto hm = HumanoidMode::decode(mode);
    return hm.baseLocked || isMobileBasePlanningEnabled();
}

void WholeBodyMpcTask::setupHumanoidModeSubscription(const std::string& topic) {
    humanoid_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg) { humanoidModeCallback(msg); });
    RCLCPP_INFO(node_->get_logger(),
                "Humanoid mode topic: %s — commands are queued and applied every control cycle in any FSM state "
                "(HOLD/MOVEJ/OCS2). partial: BASE_*, BODY_*, HEAD_*, ARMS_*, LEFT/RIGHT_ARM_*; body/arms/base preset (keeps head): PRESET:HM_*",
                topic.c_str());
}

void WholeBodyMpcTask::humanoidModeCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string name = normalizeHumanoidModeCommandName(msg->data);
    if (name.empty()) {
        return;
    }
    size_t queue_size = 0;
    {
        std::lock_guard lock(pending_humanoid_mode_mutex_);
        if (pending_humanoid_mode_cmds_.size() >= kMaxPendingHumanoidModeCmds) {
            RCLCPP_WARN(node_->get_logger(),
                        "Humanoid mode command queue full (%zu); dropping oldest. cmd=%s",
                        kMaxPendingHumanoidModeCmds, name.c_str());
            pending_humanoid_mode_cmds_.pop_front();
        }
        pending_humanoid_mode_cmds_.push_back(name);
        queue_size = pending_humanoid_mode_cmds_.size();
    }
    RCLCPP_INFO(node_->get_logger(), "Humanoid mode command queued: %s (pending=%zu)", name.c_str(), queue_size);
}

void WholeBodyMpcTask::setupTfBuffer(const std::string& topic) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // Subscribe to tf_topic and feed Buffer (no TransformListener on /tf) so chained
    // lookups world_frame_->base_frame_ (model_information.baseFrame) work.
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        topic, rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile(),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tfMsgCallback(msg); });
    RCLCPP_INFO(node_->get_logger(), "Base pose from TF buffer (topic: %s, lookup: %s->%s)",
                topic.c_str(), world_frame_.c_str(), base_frame_.c_str());
}

void WholeBodyMpcTask::tfMsgCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& t : msg->transforms) {
        try {
            tf_buffer_->setTransform(t, "isaac_tf_authority", false /*is_static*/);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "setTransform failed: %s", ex.what());
        }
    }
}

void WholeBodyMpcTask::lookupBasePoseFromBuffer() {
    (void)tryLookupMobileBasePoseFromTf();
}

bool WholeBodyMpcTask::tryLookupMobileBasePoseFromTf() {
    if (!tf_buffer_) {
        return false;
    }
    try {
        const auto tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
        const auto& p = tf.transform.translation;
        const auto& o = tf.transform.rotation;
        tf2::Quaternion q(o.x, o.y, o.z, o.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        base_x_ = p.x;
        base_y_ = p.y;
        // Continuize yaw across ±π so MPC observation does not jump by ~2π.
        base_yaw_ = base_yaw_ + wrapToPi(yaw - base_yaw_);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                              "lookupTransform %s->%s failed: %s",
                              world_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

bool WholeBodyMpcTask::isMobileBasePlanningEnabled() const {
    return interface_ && interface_->hasMobileBase() && mobile_base_tf_available_;
}

void WholeBodyMpcTask::beginMobileBaseTfProbe() {
    world_tf_publish_done_ = false;
    mobile_base_tf_probe_active_ = true;
    mobile_base_tf_decided_ = false;
    mobile_base_tf_available_ = false;
    mobile_base_tf_probe_start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                "Probing external TF %s->%s for up to %.1fs "
                "(mobile: unlock planning; any model: skip identity world TF if found)",
                world_frame_.c_str(), base_frame_.c_str(), mobile_base_tf_timeout_sec_);
    updateMobileBaseTfProbe();
}

void WholeBodyMpcTask::finalizeMobileBaseTfProbe(bool available) {
    mobile_base_tf_probe_active_ = false;
    mobile_base_tf_decided_ = true;
    mobile_base_tf_available_ = available;
    if (available) {
        world_tf_publish_done_ = true;  // external TF owns world->base; do not publish ours
        if (interface_->hasMobileBase()) {
            RCLCPP_INFO(node_->get_logger(),
                        "External TF %s->%s available: unlocked-base planning enabled",
                        world_frame_.c_str(), base_frame_.c_str());
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "External TF %s->%s available: fixed-base will not publish identity world TF",
                        world_frame_.c_str(), base_frame_.c_str());
        }
    } else {
        if (interface_->hasMobileBase()) {
            base_x_ = 0.0;
            base_y_ = 0.0;
            base_yaw_ = 0.0;
            // Persist locked default so later FSM resets cannot unlock without TF.
            {
                auto hm = HumanoidMode::decode(fsm_default_humanoid_mode_);
                if (!hm.baseLocked) {
                    hm.baseLocked = true;
                    fsm_default_humanoid_mode_ = hm.encode();
                }
            }
            RCLCPP_WARN(node_->get_logger(),
                        "No external TF %s->%s within %.1fs: unlocked-base planning disabled "
                        "(model still has mobile base DOF; base stays locked at origin). "
                        "Publishing identity world TF once for visualization.",
                        world_frame_.c_str(), base_frame_.c_str(), mobile_base_tf_timeout_sec_);
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "No external TF %s->%s within %.1fs: publishing identity world TF once for visualization",
                        world_frame_.c_str(), base_frame_.c_str(), mobile_base_tf_timeout_sec_);
        }
        publishIdentityWorldTfOnce(node_->now());
    }
    // Install the startup default once (base lock included). Later rounds reset on OCS2 exit.
    resetHumanoidModeToFsmDefault();
    publishCapabilities();
}

void WholeBodyMpcTask::updateMobileBaseTfProbe() {
    if (!mobile_base_tf_probe_active_ || mobile_base_tf_decided_) {
        return;
    }
    if (tryLookupMobileBasePoseFromTf()) {
        finalizeMobileBaseTfProbe(true);
        return;
    }
    const double elapsed = (node_->now() - mobile_base_tf_probe_start_).seconds();
    if (elapsed >= mobile_base_tf_timeout_sec_) {
        finalizeMobileBaseTfProbe(false);
    }
}

void WholeBodyMpcTask::publishIdentityWorldTfOnce(const rclcpp::Time& /*time*/) {
    if (!static_tf_broadcaster_ || world_tf_publish_done_) {
        return;
    }
    geometry_msgs::msg::TransformStamped tf_msg;
    // Static TF: stamp 0; latched on /tf_static so RViz/TF buffers do not expire it.
    tf_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_msg);
    world_tf_publish_done_ = true;
}

void WholeBodyMpcTask::computeCouplingCapturePoses(const vector_t& q, vector_t& left_pose7,
                                                   vector_t& right_pose7) const {
    // PinocchioEndEffectorKinematics resolves EE names with getBodyId (not getFrameId). Capture must
    // use the same frames or the relative vector is expressed in a different orientation than the constraint.
    left_pose7 = vector_t::Zero(7);
    right_pose7 = vector_t::Zero(7);
    left_pose7(6) = 1.0;
    right_pose7(6) = 1.0;

    const auto& minfo = interface_->getModelInfo();
    const auto& model = interface_->getPinocchioInterface().getModel();
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    auto fill = [&](const std::string& name, vector_t& pose7) {
        if (name.empty()) {
            return;
        }
        const auto frameId = model.getBodyId(name);
        const auto& M = data.oMf[frameId];
        pose7.head<3>() = M.translation();
        const Eigen::Quaterniond quat(M.rotation());
        pose7(3) = quat.x();
        pose7(4) = quat.y();
        pose7(5) = quat.z();
        pose7(6) = quat.w();
    };
    fill(minfo.eeFrame, left_pose7);
    fill(minfo.eeFrame1, right_pose7);
}

vector_t WholeBodyMpcTask::computeFramePose(const vector_t& q, const std::string& frame) const {
    const auto& model = interface_->getPinocchioInterface().getModel();
    if (frame.empty() || !model.existBodyName(frame)) {
        throw std::invalid_argument("WBC frame not found in Pinocchio model: " + frame);
    }
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const auto& placement = data.oMf[model.getBodyId(frame)];
    const Eigen::Quaterniond quat(placement.rotation());
    vector_t pose(7);
    pose << placement.translation().x(), placement.translation().y(), placement.translation().z(),
        quat.x(), quat.y(), quat.z(), quat.w();
    return pose;
}

void WholeBodyMpcTask::captureBimanualCouplingFromState(const vector_t& q) {
    vector_t left;
    vector_t right;
    computeCouplingCapturePoses(q, left, right);

    Eigen::Matrix<scalar_t, 3, 1> leftPos = left.head<3>();
    Eigen::Matrix<scalar_t, 3, 1> rightPos = right.head<3>();
    Eigen::Quaternion<scalar_t> leftQuat(left(6), left(3), left(4), left(5));
    Eigen::Quaternion<scalar_t> rightQuat(right(6), right(3), right(4), right(5));
    if (leftQuat.norm() < static_cast<scalar_t>(1e-9)) {
        leftQuat = Eigen::Quaternion<scalar_t>::Identity();
    } else {
        leftQuat.normalize();
    }
    if (rightQuat.norm() < static_cast<scalar_t>(1e-9)) {
        rightQuat = Eigen::Quaternion<scalar_t>::Identity();
    } else {
        rightQuat.normalize();
    }
    interface_->getReferenceManagerPtr()->captureCouplingFromCurrentPose(leftPos, leftQuat, rightPos, rightQuat);
}

void WholeBodyMpcTask::refreshCoupledRightMarkerFromMeasurement() {
    if (!pose_reference_manager_) {
        return;
    }
    const size_t mode = pose_reference_manager_->getModeSchedule().modeAtTime(observation_.time);
    if (HumanoidMode::decode(mode).bimanualMode != BimanualMode::COUPLED) {
        return;
    }
    vector_t left;
    vector_t right;
    computeCouplingCapturePoses(observation_.state, left, right);
    (void)right;
    pose_reference_manager_->setCoupledRightMarkerFromLeftPose(left, true);
}

WholeBodyMpcTask::ArmPolicyTransitionHold& WholeBodyMpcTask::armPolicyHold(ArmSide side) {
    return side == ArmSide::LEFT ? left_arm_policy_hold_ : right_arm_policy_hold_;
}

const WholeBodyMpcTask::ArmPolicyTransitionHold& WholeBodyMpcTask::armPolicyHold(ArmSide side) const {
    return side == ArmSide::LEFT ? left_arm_policy_hold_ : right_arm_policy_hold_;
}

void WholeBodyMpcTask::publishExternalEeCommandBlockFromHolds() {
    // Any hold (disable wait, disabled, enable wait) keeps this arm's EE reference frozen.
    // External topics are accepted again only after the hold returns to IDLE.
    block_left_external_ee_.store(
        left_arm_policy_hold_.commandHoldActive(), std::memory_order_release);
    block_right_external_ee_.store(
        right_arm_policy_hold_.commandHoldActive(), std::memory_order_release);
}

void WholeBodyMpcTask::clearArmPolicyTransitions() {
    left_arm_policy_hold_ = ArmPolicyTransitionHold{};
    right_arm_policy_hold_ = ArmPolicyTransitionHold{};
    publishExternalEeCommandBlockFromHolds();
}

void WholeBodyMpcTask::seedArmPolicyHoldsFromCurrentMode() {
    clearArmPolicyTransitions();
    if (!pose_reference_manager_ || !interface_) {
        return;
    }
    const auto& layout = interface_->getArmJointLayout();
    if (!layout.valid) {
        RCLCPP_WARN(node_->get_logger(),
                    "seedArmPolicyHoldsFromCurrentMode: arm joint layout invalid; holds left IDLE");
        return;
    }
    const size_t mode = pose_reference_manager_->getModeSchedule().modeAtTime(node_->now().seconds());
    const auto hm = HumanoidMode::decode(mode);
    const auto& command = ctrl_interfaces_.last_sent_joint_positions_;

    const auto seedInactive = [&](ArmSide side, bool active) {
        if (active) {
            return;
        }
        const char* sideName = side == ArmSide::LEFT ? "LEFT" : "RIGHT";
        const size_t start = side == ArmSide::LEFT ? layout.leftStart : layout.rightStart;
        const size_t dof = side == ArmSide::LEFT ? layout.leftDof : layout.rightDof;
        std::vector<double> held(dof);
        for (size_t local = 0; local < dof; ++local) {
            const size_t hardwareIndex = start + local;
            if (hardwareIndex >= command.size() || !std::isfinite(command[hardwareIndex])) {
                RCLCPP_WARN(node_->get_logger(),
                            "seedArmPolicyHoldsFromCurrentMode(%s): last_sent_joint_positions_[%zu] "
                            "out of bounds or non-finite; hold left IDLE",
                            sideName, hardwareIndex);
                return;
            }
            held[local] = command[hardwareIndex];
        }
        auto& hold = armPolicyHold(side);
        hold.phase = ArmHoldPhase::DISABLED_HOLD;
        hold.heldCommandPositions = std::move(held);
        hold.requiredPolicyGeneration = active_policy_generation_;
        hold.consecutiveReadyCycles = 0;
        hold.startedAt = node_->now();
        RCLCPP_INFO(node_->get_logger(),
                    "Arm policy hold: side=%s seeded DISABLED_HOLD from current mode %s",
                    sideName, modeToString(mode).c_str());
    };

    seedInactive(ArmSide::LEFT, hm.leftArmActive);
    seedInactive(ArmSide::RIGHT, hm.rightArmActive);
    publishExternalEeCommandBlockFromHolds();
}

vector_t WholeBodyMpcTask::buildStateWithArmHeldCommand(
    ArmSide side, const vector_t& measuredState) const {
    const auto& layout = interface_->getArmJointLayout();
    const auto& hold = armPolicyHold(side);
    if (!layout.valid) {
        return measuredState;
    }
    const size_t start = side == ArmSide::LEFT ? layout.leftStart : layout.rightStart;
    const size_t dof = side == ArmSide::LEFT ? layout.leftDof : layout.rightDof;
    if (hold.heldCommandPositions.size() != dof) {
        return measuredState;
    }

    vector_t state = measuredState;
    // Same state offset convention as evaluatePolicy(): mobile base prepends [x, y, yaw].
    const size_t stateOffset = interface_->hasMobileBase() ? 3 : 0;
    for (size_t local = 0; local < dof; ++local) {
        const size_t stateIndex = stateOffset + start + local;
        if (stateIndex >= static_cast<size_t>(state.size())) {
            return measuredState;
        }
        state(static_cast<Eigen::Index>(stateIndex)) = hold.heldCommandPositions[local];
    }
    return state;
}

bool WholeBodyMpcTask::beginArmPolicyTransition(ArmSide side, bool requestedActive) {
    const char* sideName = side == ArmSide::LEFT ? "LEFT" : "RIGHT";
    auto& hold = armPolicyHold(side);
    const auto& layout = interface_->getArmJointLayout();
    if (!layout.valid) {
        RCLCPP_WARN(node_->get_logger(),
                    "beginArmPolicyTransition(%s): arm joint layout invalid, rejecting transition",
                    sideName);
        return false;
    }
    const size_t start = side == ArmSide::LEFT ? layout.leftStart : layout.rightStart;
    const size_t dof = side == ArmSide::LEFT ? layout.leftDof : layout.rightDof;

    if (!requestedActive) {
        // Disable: capture last-sent command positions verbatim (never measured/deformed state),
        // so the arm freezes exactly where the hardware was last commanded.
        const auto& command = ctrl_interfaces_.last_sent_joint_positions_;
        std::vector<double> held(dof);
        for (size_t local = 0; local < dof; ++local) {
            const size_t hardwareIndex = start + local;
            if (hardwareIndex >= command.size() || !std::isfinite(command[hardwareIndex])) {
                RCLCPP_WARN(node_->get_logger(),
                            "beginArmPolicyTransition(%s, disable): last_sent_joint_positions_[%zu] "
                            "out of bounds or non-finite, rejecting transition",
                            sideName, hardwareIndex);
                return false;
            }
            held[local] = command[hardwareIndex];
        }
        hold.phase = ArmHoldPhase::WAITING_FOR_DISABLE_POLICY;
        hold.requiredPolicyGeneration = active_policy_generation_ + 1;
        hold.heldCommandPositions = std::move(held);
        hold.consecutiveReadyCycles = 0;
        hold.startedAt = node_->now();
        publishExternalEeCommandBlockFromHolds();
        RCLCPP_INFO(node_->get_logger(),
                    "Arm policy hold: side=%s begin DISABLE, waiting for policy generation %llu",
                    sideName, static_cast<unsigned long long>(hold.requiredPolicyGeneration));
        return true;
    }

    // Enable: must resume from a completed disable hold with a matching held-command size. Never
    // recapture from measured/deformed state — reuse the exact positions the disable hold captured.
    if (hold.phase != ArmHoldPhase::DISABLED_HOLD || hold.heldCommandPositions.size() != dof) {
        RCLCPP_WARN(node_->get_logger(),
                    "beginArmPolicyTransition(%s, enable): not in DISABLED_HOLD or size mismatch "
                    "(phase=%d, heldSize=%zu, dof=%zu), rejecting transition",
                    sideName, static_cast<int>(hold.phase), hold.heldCommandPositions.size(), dof);
        return false;
    }

    hold.phase = ArmHoldPhase::WAITING_FOR_ENABLE_POLICY;
    hold.requiredPolicyGeneration = active_policy_generation_ + 1;
    hold.consecutiveReadyCycles = 0;
    hold.startedAt = node_->now();
    // Block external EE topics before writing the resume pose, so a callback cannot
    // replace that pose while the joints are still frozen.
    publishExternalEeCommandBlockFromHolds();

    try {
        // EE target: this arm keeps the held joints; every other joint uses the
        // command being sent now (last_sent), not the measured pose.
        vector_t reference = observation_.state;
        const size_t jointOffset = interface_->hasMobileBase() ? 3 : 0;
        const auto& command = ctrl_interfaces_.last_sent_joint_positions_;
        for (size_t i = 0; i < joint_names_.size() && i < command.size(); ++i) {
            const size_t stateIndex = jointOffset + i;
            if (stateIndex >= static_cast<size_t>(reference.size()) || !std::isfinite(command[i])) {
                continue;
            }
            reference(static_cast<Eigen::Index>(stateIndex)) = command[i];
        }
        const vector_t targetState = buildStateWithArmHeldCommand(side, reference);
        vector_t leftEe;
        vector_t rightEe;
        computeCouplingCapturePoses(targetState, leftEe, rightEe);
        if (side == ArmSide::LEFT) {
            pose_reference_manager_->resetLeftEndEffectorTarget(leftEe);
        } else {
            pose_reference_manager_->resetRightEndEffectorTarget(rightEe);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Arm policy hold: side=%s begin ENABLE (resume from held command), waiting for "
                    "policy generation %llu",
                    sideName, static_cast<unsigned long long>(hold.requiredPolicyGeneration));
        return true;
    } catch (const std::exception& e) {
        hold.phase = ArmHoldPhase::DISABLED_HOLD;
        publishExternalEeCommandBlockFromHolds();
        RCLCPP_WARN(node_->get_logger(), "beginArmPolicyTransition(%s, enable) failed: %s", sideName,
                    e.what());
        return false;
    }
}

void WholeBodyMpcTask::updateArmPolicyTransitionReadiness(
    size_t plannedMode, const vector_t& futureState, const vector_t& futureInput) {
    const auto& layout = interface_->getArmJointLayout();
    const auto plannedHm = HumanoidMode::decode(plannedMode);
    const size_t stateOffset = interface_->hasMobileBase() ? 3 : 0;
    const size_t inputOffset = interface_->hasMobileBase() ? base_input_dim_ : 0;
    bool enableReleased = false;

    const auto processSide = [&](ArmSide side) {
        auto& hold = armPolicyHold(side);
        if (hold.phase == ArmHoldPhase::IDLE) {
            return;
        }
        const char* sideName = side == ArmSide::LEFT ? "LEFT" : "RIGHT";
        if (!layout.valid) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                  "updateArmPolicyTransitionReadiness(%s): arm joint layout invalid, "
                                  "cannot evaluate readiness (holding last command)",
                                  sideName);
            return;
        }
        const size_t start = side == ArmSide::LEFT ? layout.leftStart : layout.rightStart;
        const size_t dof = side == ArmSide::LEFT ? layout.leftDof : layout.rightDof;
        const bool plannedActive = side == ArmSide::LEFT ? plannedHm.leftArmActive : plannedHm.rightArmActive;
        const bool generationReady = active_policy_generation_ >= hold.requiredPolicyGeneration;
        const double waitedSec = (node_->now() - hold.startedAt).seconds();

        if (hold.phase == ArmHoldPhase::WAITING_FOR_DISABLE_POLICY) {
            // Only confirm that a plan reflecting the disabled arm has become active; the held
            // command itself must not be cleared here (DISABLED_HOLD keeps overriding it).
            if (generationReady && !plannedActive) {
                hold.phase = ArmHoldPhase::DISABLED_HOLD;
                RCLCPP_INFO(node_->get_logger(),
                            "Arm policy hold: side=%s DISABLE confirmed at generation %llu -> DISABLED_HOLD",
                            sideName, static_cast<unsigned long long>(active_policy_generation_));
                return;
            }
            if (waitedSec > 0.5) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Arm policy hold: side=%s still WAITING_FOR_DISABLE_POLICY after %.2fs "
                                      "(generation=%llu/%llu, plannedActive=%d)",
                                      sideName, waitedSec,
                                      static_cast<unsigned long long>(active_policy_generation_),
                                      static_cast<unsigned long long>(hold.requiredPolicyGeneration),
                                      plannedActive ? 1 : 0);
            }
            return;
        }

        if (hold.phase == ArmHoldPhase::DISABLED_HOLD) {
            // No phase change here; applyArmTransitionHoldToCommands() keeps overriding the command.
            return;
        }

        if (hold.phase == ArmHoldPhase::WAITING_FOR_ENABLE_POLICY) {
            if (hold.heldCommandPositions.size() != dof) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Arm policy hold: side=%s held command size mismatch (%zu vs dof %zu); "
                                      "resetting ready count",
                                      sideName, hold.heldCommandPositions.size(), dof);
                hold.consecutiveReadyCycles = 0;
                return;
            }
            double maxPositionError = 0.0;
            double maxVelocity = 0.0;
            bool continuityOk = true;
            for (size_t local = 0; local < dof; ++local) {
                const size_t stateIndex = stateOffset + start + local;
                const size_t inputIndex = inputOffset + start + local;
                if (stateIndex >= static_cast<size_t>(futureState.size()) ||
                    inputIndex >= static_cast<size_t>(futureInput.size())) {
                    continuityOk = false;
                    break;
                }
                const double stateVal = futureState[static_cast<Eigen::Index>(stateIndex)];
                const double inputVal = futureInput[static_cast<Eigen::Index>(inputIndex)];
                if (!std::isfinite(stateVal) || !std::isfinite(inputVal)) {
                    continuityOk = false;
                    break;
                }
                maxPositionError = std::max(maxPositionError, std::abs(stateVal - hold.heldCommandPositions[local]));
                maxVelocity = std::max(maxVelocity, std::abs(inputVal));
            }

            // Release on joint velocity, not on future-state position. The held joints still sit
            // at heldCommandPositions this cycle; a small commanded velocity is what keeps the
            // first visible command continuous. A pose that merely stays close while the policy
            // runs off at ~0.6 rad/s is not continuous and must not release.
            const bool velocityReady = continuityOk && maxVelocity <= kArmPolicyReleaseVelocityTolerance;
            // maxPositionError is logged for visibility but no longer gates the release.

            if (generationReady && plannedActive && velocityReady) {
                ++hold.consecutiveReadyCycles;
                RCLCPP_INFO(node_->get_logger(),
                            "Arm policy hold: side=%s ENABLE readiness ok, generation=%llu "
                            "maxPosErr=%.5f maxVel=%.5f readyCycles=%zu/%zu",
                            sideName, static_cast<unsigned long long>(active_policy_generation_),
                            maxPositionError, maxVelocity, hold.consecutiveReadyCycles,
                            kArmPolicyReleaseConsecutiveCycles);
                if (hold.consecutiveReadyCycles >= kArmPolicyReleaseConsecutiveCycles) {
                    RCLCPP_INFO(node_->get_logger(),
                                "Arm policy hold: side=%s ENABLE released -> IDLE (policy fully resumed)",
                                sideName);
                    hold = ArmPolicyTransitionHold{};
                    enableReleased = true;
                }
                return;
            }

            hold.consecutiveReadyCycles = 0;
            if (waitedSec > 0.5) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                      "Arm policy hold: side=%s still WAITING_FOR_ENABLE_POLICY after %.2fs "
                                      "(generationReady=%d plannedActive=%d maxPosErr=%.5f maxVel=%.5f)",
                                      sideName, waitedSec, generationReady ? 1 : 0, plannedActive ? 1 : 0,
                                      maxPositionError, maxVelocity);
            }
        }
    };

    processSide(ArmSide::LEFT);
    processSide(ArmSide::RIGHT);
    // Open this arm's EE topics before publishing ENABLED, so the first command after
    // release is accepted.
    publishExternalEeCommandBlockFromHolds();
    // current_state keeps the arm DISABLED while the enable hold is active. Flip it only
    // after the hold actually returns to IDLE.
    if (enableReleased && pose_reference_manager_) {
        const size_t mode = pose_reference_manager_->getModeSchedule().modeAtTime(observation_.time);
        publishCurrentHumanoidState(mode);
        RCLCPP_INFO(node_->get_logger(),
                    "Humanoid current_state updated after arm ENABLE release: %s",
                    modeToString(mode).c_str());
    }
}

void WholeBodyMpcTask::applyArmTransitionHoldToCommands() {
    const auto& layout = interface_->getArmJointLayout();
    if (!layout.valid) {
        return;
    }
    const bool isMix = ctrl_interfaces_.control_mode_ == robot_controller_core::ControlMode::MIX;
    // cached_last_action_ is captured from the raw, un-held future_state earlier in evaluatePolicy()
    // (before this override runs). Pin it to the held command too, otherwise the next cycle's
    // observation (when the cached-observation path is taken) silently drifts away from the frozen
    // command instead of reflecting what was actually sent to hardware.
    const size_t stateOffset = interface_->hasMobileBase() ? 3 : 0;

    const auto applySide = [&](ArmSide side) {
        const auto& hold = armPolicyHold(side);
        const size_t start = side == ArmSide::LEFT ? layout.leftStart : layout.rightStart;
        const size_t dof = side == ArmSide::LEFT ? layout.leftDof : layout.rightDof;
        if (!hold.commandHoldActive() || hold.heldCommandPositions.size() != dof) {
            return;
        }
        for (size_t local = 0; local < dof; ++local) {
            const size_t hardwareIndex = start + local;
            if (hardwareIndex >= joint_names_.size()) {
                continue;
            }
            // fa_w2 POSITION mode: position only, no effort write. MIX mode: additionally zero the
            // velocity command for held joints; static torque/effort is left as written by the
            // normal output path above.
            ctrl_interfaces_.setJointPositionCommand(hardwareIndex, hold.heldCommandPositions[local]);
            if (isMix && hardwareIndex < ctrl_interfaces_.joint_velocity_command_interface_.size()) {
                ctrl_interfaces_.setJointVelocityCommand(hardwareIndex, 0.0);
            }
            const size_t stateIndex = stateOffset + hardwareIndex;
            if (stateIndex < static_cast<size_t>(cached_last_action_.size())) {
                cached_last_action_(static_cast<Eigen::Index>(stateIndex)) = hold.heldCommandPositions[local];
            }
        }
    };

    applySide(ArmSide::LEFT);
    applySide(ArmSide::RIGHT);
}

void WholeBodyMpcTask::applyPendingHumanoidModeIfAny() {
    std::deque<std::string> cmds;
    {
        std::lock_guard<std::mutex> lock(pending_humanoid_mode_mutex_);
        cmds.swap(pending_humanoid_mode_cmds_);
    }
    if (cmds.empty()) {
        return;
    }

    // ModeSchedule is double-buffered; getModeSchedule() returns the active value until the next
    // MPC preSolverRun. Chain locally so queued cmds in the same cycle compose correctly.
    size_t currentMode = pose_reference_manager_->getModeSchedule().modeAtTime(observation_.time);
    bool any_applied = false;
    for (const auto& cmd : cmds) {
        if (cmd.empty()) {
            continue;
        }

        ArmPolicyTransitionHold left_snapshot;
        ArmPolicyTransitionHold right_snapshot;
        bool snapshot_taken = false;

        try {
            if (handleHomeJointReferenceCommand(cmd, currentMode)) {
                continue;
            }
            size_t mode = resolveHumanoidModeCommand(currentMode, cmd);
            if (!isHumanoidModeSupported(mode)) {
                RCLCPP_WARN(node_->get_logger(),
                            "Mode rejected (unsupported by config): %s — "
                            "mobileBase=%s basePlan=%s bodyRelative=%s waistLock=%s bimanual=%s bodyTracking=%s headTracking=%s. cmd=%s",
                            modeToString(mode).c_str(),
                            interface_->hasMobileBase() ? "Y" : "N",
                            isMobileBasePlanningEnabled() ? "Y" : "N",
                            interface_->hasBodyRelativeConstraint() ? "Y" : "N",
                            interface_->hasWaistLock() ? "Y" : "N",
                            interface_->hasBimanualCoupling() ? "Y" : "N",
                            interface_->isBodyTrackingEeEnabled() ? "Y" : "N",
                            interface_->isHeadTrackingEeEnabled() ? "Y" : "N",
                            cmd.c_str());
                continue;
            }
            if (mode == currentMode) {
                RCLCPP_INFO(node_->get_logger(),
                            "Humanoid mode unchanged after cmd=%s (still %s)", cmd.c_str(),
                            modeToString(mode).c_str());
                continue;
            }

            const auto current_hm = HumanoidMode::decode(currentMode);
            auto hm = HumanoidMode::decode(mode);

            // Arbitration: active<->inactive transitions are only allowed from the hold phase that
            // marks the arm as fully settled in the opposite state (IDLE for disable, DISABLED_HOLD
            // for re-enable). Any WAITING_* phase rejects both directions for that arm.
            const auto canChange = [this](ArmSide side, bool currentActive, bool requestedActive) {
                const auto phase = armPolicyHold(side).phase;
                if (currentActive && !requestedActive) {
                    return phase == ArmHoldPhase::IDLE;
                }
                if (!currentActive && requestedActive) {
                    return phase == ArmHoldPhase::DISABLED_HOLD;
                }
                return true;
            };

            const bool leftChanges = current_hm.leftArmActive != hm.leftArmActive;
            const bool rightChanges = current_hm.rightArmActive != hm.rightArmActive;
            const bool leftAllowed =
                !leftChanges || canChange(ArmSide::LEFT, current_hm.leftArmActive, hm.leftArmActive);
            const bool rightAllowed =
                !rightChanges || canChange(ArmSide::RIGHT, current_hm.rightArmActive, hm.rightArmActive);
            // A command touching both arms is rejected in full if either arm cannot change right now.
            if (!leftAllowed || !rightAllowed) {
                RCLCPP_WARN(node_->get_logger(),
                            "Mode rejected (arm hold transition in progress): cmd=%s %s -> %s "
                            "(leftPhase=%d allowed=%d, rightPhase=%d allowed=%d)",
                            cmd.c_str(), modeToString(currentMode).c_str(),
                            modeToString(mode).c_str(),
                            static_cast<int>(armPolicyHold(ArmSide::LEFT).phase), leftAllowed ? 1 : 0,
                            static_cast<int>(armPolicyHold(ArmSide::RIGHT).phase), rightAllowed ? 1 : 0);
                continue;
            }

            left_snapshot = left_arm_policy_hold_;
            right_snapshot = right_arm_policy_hold_;
            snapshot_taken = true;

            bool transition_ok = true;
            if (leftChanges) {
                transition_ok = beginArmPolicyTransition(ArmSide::LEFT, hm.leftArmActive);
            }
            if (transition_ok && rightChanges) {
                transition_ok = beginArmPolicyTransition(ArmSide::RIGHT, hm.rightArmActive);
            }
            if (!transition_ok) {
                left_arm_policy_hold_ = left_snapshot;
                right_arm_policy_hold_ = right_snapshot;
                publishExternalEeCommandBlockFromHolds();
                RCLCPP_WARN(node_->get_logger(),
                            "Mode rejected (arm hold transition begin failed): cmd=%s", cmd.c_str());
                continue;
            }

            const vector_t q = observation_.state;
            // Use getBodyId EE poses for MPC targets so left soft-constraint matches coupling frames.
            vector_t left_ee;
            vector_t right_ee;
            computeCouplingCapturePoses(q, left_ee, right_ee);
            vector_t body = visualizer_->computeBodyFramePose(q);

            const bool entering_coupled =
                hm.bimanualMode == BimanualMode::COUPLED &&
                current_hm.bimanualMode != BimanualMode::COUPLED;
            const bool leaving_coupled =
                current_hm.bimanualMode == BimanualMode::COUPLED &&
                hm.bimanualMode != BimanualMode::COUPLED;
            const bool switched_to_body_tracking =
                (current_hm.effectiveBodyMode() != BodyConstraintMode::TRACKING) &&
                (hm.effectiveBodyMode() == BodyConstraintMode::TRACKING);
            const bool switched_to_head_tracking =
                current_hm.headMode != HeadMode::TRACKING &&
                hm.headMode == HeadMode::TRACKING;
            const bool already_coupled = hm.bimanualMode == BimanualMode::COUPLED;
            // When an arm changed active state this cmd, beginArmPolicyTransition() already reset
            // that arm's single-arm target (held command on disable, resumed pose on enable) — avoid
            // clobbering it with a dual-arm reset built from measured state below.
            const bool arm_hold_handled_reset = leftChanges || rightChanges;

            // Brake once on unlocked→locked: cmd_vel publish stops when baseLocked, but
            // many sims/HW latch the last non-zero Twist until a zero is sent. BASE_LOCK
            // and presets that lock the base all go through this path. StateOCS2 exit
            // still calls stopBase() separately.
            if (!current_hm.baseLocked && hm.baseLocked) {
                stopBase();
            }

            if (entering_coupled) {
                pose_reference_manager_->setBodyPoseReference(body);
                if (!arm_hold_handled_reset) {
                    pose_reference_manager_->setCurrentEndEffectorPoses(left_ee, right_ee, true);
                }
                captureBimanualCouplingFromState(q);
                pose_reference_manager_->markLeftCouplingCommand();
            }

            if (switched_to_head_tracking) {
                pose_reference_manager_->setHeadPoseTargetOnly(
                    computeFramePose(q, head_frame_), true);
            }

            pose_reference_manager_->setModeSchedule(ocs2::ModeSchedule({}, {mode}));

            if (leaving_coupled) {
                // Coupling drives the right EE from the left EE constraint, not right_target_state_.
                // Re-latch both targets to measured EE so decoupling cannot command a jump
                // to a stale marker / cache pose.
                pose_reference_manager_->setCurrentEndEffectorPoses(left_ee, right_ee, true);
            }

            if (switched_to_body_tracking) {
                if (!arm_hold_handled_reset) {
                    pose_reference_manager_->setCurrentEndEffectorPoses(left_ee, right_ee, false);
                }
                pose_reference_manager_->setBodyPoseTargetOnly(body, true);
                if (already_coupled) {
                    captureBimanualCouplingFromState(q);
                }
            }
            pose_reference_manager_->publishCurrentTargetsFromCache();
            publishCurrentHumanoidState(mode);
            RCLCPP_INFO(node_->get_logger(), "Humanoid mode applied: cmd=%s | %s -> %s", cmd.c_str(),
                        modeToString(currentMode).c_str(),
                        modeToString(mode).c_str());
            const bool enablePending =
                (leftChanges && hm.leftArmActive &&
                 armPolicyHold(ArmSide::LEFT).phase == ArmHoldPhase::WAITING_FOR_ENABLE_POLICY) ||
                (rightChanges && hm.rightArmActive &&
                 armPolicyHold(ArmSide::RIGHT).phase == ArmHoldPhase::WAITING_FOR_ENABLE_POLICY);
            if (enablePending) {
                RCLCPP_INFO(node_->get_logger(),
                            "Arm enable pending: current_state arm bit stays DISABLED until policy hold releases");
            }
            currentMode = mode;
            any_applied = true;
        } catch (const std::exception& e) {
            if (snapshot_taken) {
                left_arm_policy_hold_ = left_snapshot;
                right_arm_policy_hold_ = right_snapshot;
                publishExternalEeCommandBlockFromHolds();
            }
            RCLCPP_WARN(node_->get_logger(), "apply humanoid mode failed (cmd=%s): %s", cmd.c_str(), e.what());
        }
    }
    // Flush buffered ModeSchedule/TargetTrajectories once per batch. Prefer once over per-cmd to
    // reduce races with the MPC thread (BufferedValue get vs updateFromBuffer).
    if (any_applied) {
        pose_reference_manager_->preSolverRun(observation_.time, observation_.time, observation_.state);
    }
}

void WholeBodyMpcTask::resetHumanoidModeToFsmDefault() {
    if (!pose_reference_manager_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(pending_humanoid_mode_mutex_);
        pending_humanoid_mode_cmds_.clear();
    }
    // FSM default has both arms active; drop any stale hold transition bookkeeping.
    clearArmPolicyTransitions();
    interface_->getReferenceManagerPtr()->clearCapturedCoupling();
    size_t mode = fsm_default_humanoid_mode_;
    {
        auto hm = HumanoidMode::decode(mode);
        bool mutated = false;
        // Until / without external world TF, never leave the base unlocked.
        if (!isMobileBasePlanningEnabled()) {
            hm.baseLocked = true;
            mutated = true;
        }
        if (mutated) {
            mode = hm.encode();
        }
    }
    // BufferedValue: setModeSchedule only fills the buffer; flush so getModeSchedule() matches
    // (otherwise a later identical BODY_* preset looks "unchanged" while buffer still holds default).
    pose_reference_manager_->setModeSchedule(ocs2::ModeSchedule({}, {mode}));
    pose_reference_manager_->preSolverRun(observation_.time, observation_.time, observation_.state);
    publishCapabilities();
    publishCurrentHumanoidState(mode);
    RCLCPP_INFO(node_->get_logger(), "FSM: humanoid mode reset to %s (coupling cleared)",
                modeToString(mode).c_str());
}

rcl_interfaces::msg::SetParametersResult WholeBodyMpcTask::on_parameter_change(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Apply dir before enabled so a same-batch update uses the new directory.
    std::optional<bool> traj_record_enabled_update;
    for (const auto& param : parameters) {
        if (param.get_name() == "hardware_latency") {
            hardware_latency_ = param.as_double();
            RCLCPP_INFO(node_->get_logger(), "Updated hardware_latency to: %f", hardware_latency_);
        } else if (param.get_name() == "traj_record_dir") {
            traj_record_dir_ = param.as_string();
        } else if (param.get_name() == "traj_record_enabled") {
            traj_record_enabled_update = param.as_bool();
        } else if (param.get_name() == "rt_timing_enabled") {
            rt_timing_.enabled.store(param.as_bool(), std::memory_order_relaxed);
            RCLCPP_INFO(node_->get_logger(),
                        "rt_timing_enabled=%s. Remote-desktop: HOLD 20s / OCS2 idle 20s / "
                        "drag markers 20s / HOLD. Collect [rt_timing] and Overrun detected.",
                        param.as_bool() ? "true" : "false");
        }
    }
    if (traj_record_enabled_update.has_value()) {
        trajectory_manager::TrajectoryRecorder::instance()
            .setEnabled(*traj_record_enabled_update, traj_record_dir_);
        RCLCPP_INFO(node_->get_logger(), "traj_record_enabled=%d, dir=%s",
                    *traj_record_enabled_update, traj_record_dir_.c_str());
    }
    return result;
}

void WholeBodyMpcTask::setupInterface(const std::string& task_file, const std::string& lib_folder,
                                      const std::string& urdf_file,
                                      const FrameOverrides& frame_overrides) {
    interface_ = std::make_shared<WholeBodyInterface>(
        task_file, lib_folder, urdf_file, frame_overrides);

    // Used only when no external world TF: publish identity static world->base_frame once for RViz.
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

    if (interface_->hasMobileBase()) {
        RCLCPP_INFO(node_->get_logger(), "Mobile base model: static TF broadcaster ready for optional identity world->%s",
                    interface_->getModelInfo().baseFrame.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "Fixed base model: static TF broadcaster ready for optional identity world->%s",
                    interface_->getModelInfo().baseFrame.c_str());
    }
}

void WholeBodyMpcTask::loadBaseCmdVelocityLimits(const std::string& task_file) {
    const auto dim = static_cast<Eigen::Index>(base_input_dim_);
    base_cmd_lower_ = vector_t::Constant(dim, -1.0);
    base_cmd_upper_ = vector_t::Constant(dim, 1.0);
    if (dim < 2) {
        return;
    }
    const auto& minfo = interface_->getModelInfo();
    const std::string model_key =
        mobile_manipulator::modelTypeEnumToString(minfo.manipulatorModelType);
    const std::string lower_key = "jointVelocityLimits.lowerBound.base." + model_key;
    const std::string upper_key = "jointVelocityLimits.upperBound.base." + model_key;
    try {
        vector_t lower = vector_t::Zero(dim);
        vector_t upper = vector_t::Zero(dim);
        loadData::loadEigenMatrix(task_file, lower_key, lower);
        loadData::loadEigenMatrix(task_file, upper_key, upper);
        base_cmd_lower_ = lower;
        base_cmd_upper_ = upper;
        if (dim == 3) {
            RCLCPP_INFO(node_->get_logger(),
                        "cmd_vel clamp from task.info %s: vx[%.3f,%.3f] vy[%.3f,%.3f] w[%.3f,%.3f]",
                        model_key.c_str(),
                        base_cmd_lower_(0), base_cmd_upper_(0),
                        base_cmd_lower_(1), base_cmd_upper_(1),
                        base_cmd_lower_(2), base_cmd_upper_(2));
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "cmd_vel clamp from task.info %s: v[%.3f,%.3f] w[%.3f,%.3f]",
                        model_key.c_str(),
                        base_cmd_lower_(0), base_cmd_upper_(0),
                        base_cmd_lower_(1), base_cmd_upper_(1));
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to load %s / %s (%s); falling back to ±1.0 cmd_vel clamp",
                    lower_key.c_str(), upper_key.c_str(), ex.what());
    }
}

void WholeBodyMpcTask::setupPublisher() {
    mpc_observation_publisher_ =
        node_->create_publisher<ocs2_msgs::msg::MpcObservation>(robot_name_ + "_mpc_observation", 1);
    // Relative → /<ns>/fsm_command, /<ns>/cmd_vel
    // Relative with controller name → /<ns>/<controller>/whole_body_capabilities|current_state
    const std::string controller_name = std::string(node_->get_name());
    fsm_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("fsm_command", 10);
    capability_pub_ = node_->create_publisher<robot_control_msgs::msg::WholeBodyCapability>(
        controller_name + "/whole_body_capabilities", rclcpp::QoS(1).transient_local());
    current_state_pub_ = node_->create_publisher<robot_control_msgs::msg::WholeBodyCurrentState>(
        controller_name + "/current_state", rclcpp::QoS(1).transient_local());
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void WholeBodyMpcTask::updateObservation(const rclcpp::Time& time) {
    if (!mpc_mrt_interface_) {
        return;
    }
    observation_.time = time.seconds();

    // Fixed and mobile: progress activate-time world TF probe (skip identity publish if external TF exists).
    updateMobileBaseTfProbe();

    if (interface_->hasMobileBase()) {
        // WheelBasedMobileManipulator 状态向量：[x, y, yaw, q0, q1, ..., qN]
        // 前3维仅在外部 world TF 可用时由 TF 更新；否则固定在原点（禁止底盘规划）。
        if (isMobileBasePlanningEnabled()) {
            lookupBasePoseFromBuffer();
        }
        observation_.state[0] = base_x_;
        observation_.state[1] = base_y_;
        observation_.state[2] = base_yaw_;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            observation_.state[3 + i] = ctrl_interfaces_.measuredJointPosition(i);
        }
    } else {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            observation_.state[i] = ctrl_interfaces_.measuredJointPosition(i);
        }
    }

    observation_.input = vector_t::Zero(interface_->getModelInfo().inputDim);

    if (pose_reference_manager_) {
        pose_reference_manager_->setCurrentObservation(observation_);
        applyPendingHumanoidModeIfAny();
    }

    // RT: one FK for control-path body pose; PoseStamped publish is throttled.
    if (visualizer_) {
        mpc_core::OCS2PinocchioVisualizer::EndEffectorPoses poses;
        {
            auto fk_scope = rt_timing_.scope(&rt_timing_.ee_fk_us);
            poses = visualizer_->computeEndEffectorPoses(observation_.state);
        }
        if (pose_reference_manager_ && bodyTrackingEeEnabled_) {
            const size_t current_mode = pose_reference_manager_->getModeSchedule().modeAtTime(observation_.time);
            const auto hm = HumanoidMode::decode(current_mode);
            if (hm.bodyMode != BodyConstraintMode::TRACKING) {
                pose_reference_manager_->setBodyPoseReference(poses.body);
            }
        }
        {
            auto pub_scope = rt_timing_.scope(&rt_timing_.ee_pub_us);
            visualizer_->publishEndEffectorPoses(time, poses);
        }
    }
}

void WholeBodyMpcTask::evaluatePolicy(const rclcpp::Time& time) {
    auto eval_scope = rt_timing_.scope(&rt_timing_.eval_us);
    if (!mpc_mrt_interface_) {
        RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
        return;
    }
    {
        auto upd_scope = rt_timing_.scope(&rt_timing_.eval_upd_us);
        if (mpc_mrt_interface_->updatePolicy()) {
            policy_active_ = true;
            ++active_policy_generation_;
        }
    }
    // Keep the planner on the last commanded joints. last_sent is updated by HOLD
    // and by the previous OCS2 command, so re-entry does not snap to measured q.
    // Base x/y/yaw stay on the TF values written by updateObservation().
    if (cached_ob_state_) {
        const size_t jointOffset = interface_->hasMobileBase() ? 3 : 0;
        const auto& command = ctrl_interfaces_.last_sent_joint_positions_;
        for (size_t i = 0; i < joint_names_.size() && i < command.size(); ++i) {
            const size_t stateIndex = jointOffset + i;
            if (stateIndex >= static_cast<size_t>(observation_.state.size()) || !std::isfinite(command[i])) {
                continue;
            }
            observation_.state(static_cast<Eigen::Index>(stateIndex)) = command[i];
        }
    }
    const SystemObservation obs = observation_;

    mpc_mrt_interface_->setCurrentObservation(obs);
    const bool publish_dds = shouldPublishDds(time);
    {
        auto pub_scope = rt_timing_.scope(&rt_timing_.eval_obs_pub_us);
        if (publish_dds) {
            mpc_observation_publisher_->publish(ros_msg_conversions::createObservationMsg(obs));
        }
    }

    if (!policy_active_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Policy not ready (no active MRT policy yet), holding");
        holdLastSentPositions();
        applyArmTransitionHoldToCommands();
        return;
    }

    size_t planned_mode = 0;
    vector_t future_state;
    vector_t future_input;
    {
        auto mrt_scope = rt_timing_.scope(&rt_timing_.eval_mrt_us);
        mpc_mrt_interface_->evaluatePolicy(time.seconds(), obs.state, optimized_state_, optimized_input_, planned_mode);
        const auto& policy = mpc_mrt_interface_->getPolicy();
        const double future_time = obs.time + future_time_offset_ / ctrl_interfaces_.frequency_;
        future_state = LinearInterpolation::interpolate(future_time, policy.timeTrajectory_, policy.stateTrajectory_);
        future_input = LinearInterpolation::interpolate(future_time, policy.timeTrajectory_, policy.inputTrajectory_);
        pending_viz_has_pred_ = true;
        pending_viz_pred_time_ = future_time;
        pending_viz_pred_state_ = future_state;
    }

    updateArmPolicyTransitionReadiness(planned_mode, future_state, future_input);

    const size_t state_joint_offset = interface_->hasMobileBase() ? 3 : 0;
    const size_t input_joint_offset = interface_->hasMobileBase() ? base_input_dim_ : 0;

    auto cmd_scope = rt_timing_.scope(&rt_timing_.eval_cmd_us);
    if (cmd_vel_publisher_ && isMobileBasePlanningEnabled() &&
        !HumanoidMode::decode(planned_mode).baseLocked &&
        future_input.size() >= static_cast<Eigen::Index>(base_input_dim_) &&
        base_input_dim_ >= 2 && publish_dds)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        if (base_input_dim_ == 3 && base_cmd_lower_.size() >= 3 && base_cmd_upper_.size() >= 3) {
            cmd_vel_msg.linear.x = std::clamp(future_input(0), base_cmd_lower_(0), base_cmd_upper_(0));
            cmd_vel_msg.linear.y = std::clamp(future_input(1), base_cmd_lower_(1), base_cmd_upper_(1));
            cmd_vel_msg.angular.z = std::clamp(future_input(2), base_cmd_lower_(2), base_cmd_upper_(2));
        } else if (base_cmd_lower_.size() >= 2 && base_cmd_upper_.size() >= 2) {
            cmd_vel_msg.linear.x = std::clamp(future_input(0), base_cmd_lower_(0), base_cmd_upper_(0));
            cmd_vel_msg.angular.z = std::clamp(future_input(1), base_cmd_lower_(1), base_cmd_upper_(1));
        } else {
            cmd_vel_msg.linear.x = future_input(0);
            cmd_vel_msg.angular.z = future_input(base_input_dim_ == 3 ? 2 : 1);
            if (base_input_dim_ == 3) {
                cmd_vel_msg.linear.y = future_input(1);
            }
        }
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    if (ctrl_interfaces_.control_mode_ == robot_controller_core::ControlMode::POSITION) {
        for (size_t i = 0; i < joint_names_.size() && state_joint_offset + i < static_cast<size_t>(future_state.size()); ++i) {
            ctrl_interfaces_.setJointPositionCommand(i, future_state(state_joint_offset + i));
        }
        cached_last_action_ = future_state;
        last_execute_time_ = time;
    } else if (ctrl_interfaces_.control_mode_ == robot_controller_core::ControlMode::MIX) {
        vector_t static_torques = calculateStaticTorques();
        for (size_t i = 0; i < joint_names_.size() && (state_joint_offset + i) < static_cast<size_t>(static_torques.size()); ++i) {
            ctrl_interfaces_.setJointForceCommand(i, static_torques(state_joint_offset + i));
        }
        for (size_t i = 0; i < joint_names_.size() && (state_joint_offset + i) < static_cast<size_t>(future_state.size()); ++i) {
            ctrl_interfaces_.setJointPositionCommand(i, future_state(state_joint_offset + i));
        }
        for (size_t i = 0; i < joint_names_.size() && (input_joint_offset + i) < static_cast<size_t>(future_input.size()); ++i) {
            ctrl_interfaces_.setJointVelocityCommand(i, future_input(input_joint_offset + i));
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown control output mode");
    }

    applyArmTransitionHoldToCommands();
}

void WholeBodyMpcTask::resetMpc() {
    // Drop in-flight WAITING_* bookkeeping, then match holds to the mode that survived this reset.
    // Inactive arms become DISABLED_HOLD so a later ENABLE can resume, and the freeze stays on.
    seedArmPolicyHoldsFromCurrentMode();
    TargetTrajectories target_trajectories;
    {
        observation_.time = node_->now().seconds();
        if (interface_->hasMobileBase()) {
            observation_.state[0] = base_x_;
            observation_.state[1] = base_y_;
            observation_.state[2] = base_yaw_;
            for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i) {
                observation_.state[3 + i] = ctrl_interfaces_.last_sent_joint_positions_[i];
            }
        } else {
            for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i) {
                observation_.state[i] = ctrl_interfaces_.last_sent_joint_positions_[i];
            }
        }

        vector_t left_initial_ee_state;
        vector_t right_initial_ee_state;
        vector_t body_initial_pose;
        vector_t head_initial_pose = vector_t::Zero(7);
        head_initial_pose(6) = 1.0;
        if (visualizer_) {
            const auto poses = visualizer_->computeEndEffectorPoses(observation_.state);
            left_initial_ee_state = poses.left;
            right_initial_ee_state = poses.right;
            body_initial_pose = poses.body;
        } else {
            left_initial_ee_state = vector_t::Zero(7);
            right_initial_ee_state = vector_t::Zero(7);
            body_initial_pose = vector_t::Zero(7);
            body_initial_pose(6) = 1.0;
        }
        if (headTrackingEeEnabled_) {
            head_initial_pose = computeFramePose(observation_.state, head_frame_);
        }
        RCLCPP_INFO(node_->get_logger(), "Initial Target Trajectory - Time: %.3f", observation_.time);
        if (pose_reference_manager_) {
            pose_reference_manager_->setCurrentObservation(observation_);
            pose_reference_manager_->setBodyPoseReference(body_initial_pose);
            pose_reference_manager_->setHeadPoseReference(head_initial_pose);
            pose_reference_manager_->setCurrentEndEffectorPoses(left_initial_ee_state, right_initial_ee_state, false);
            // Explicit enter publish: markers subscribe to *_current_target (incl. body).
            pose_reference_manager_->publishCurrentTargetsFromCache();
        }
        vector_t dual_arm_target = ocs2::mpc_core::PoseBasedReferenceManager::assembleWholeBodyTargetState(
            left_initial_ee_state, right_initial_ee_state, body_initial_pose, head_initial_pose,
            bodyTrackingEeEnabled_, headTrackingEeEnabled_);
        target_trajectories =
            TargetTrajectories({observation_.time}, {dual_arm_target}, {observation_.input});
    }

    mpc_mrt_interface_->reset();
    policy_active_ = false;
    collision_active_.store(false, std::memory_order_release);
    mpc_mrt_interface_->setCurrentObservation(observation_);
    mpc_mrt_interface_->resetMpcNode(target_trajectories);
    // Do not block for initialPolicyReceived() here — that would overrun the RT update loop.
    RCLCPP_INFO(node_->get_logger(),
                "MPC reset complete; waiting for initial policy on MPC thread");
}

void WholeBodyMpcTask::runVisualizationOnce(const VisualizationSnapshot& snap) {
    const auto& obs = snap.observation;
    if (!visualizer_ || obs.state.size() == 0) {
        return;
    }

    // One FCL per viz tick on command-overlay q: updates markers + collision_active_ for RT.
    const vector_t& collision_q =
        snap.collision_state.size() > 0 ? snap.collision_state : obs.state;
    bool active = false;
    if (visualizer_->publishSelfCollisionVisualization(collision_q)) {
        const scalar_t thr = interface_->getSelfCollisionMinimumDistance();
        active = visualizer_->getLastMinDistance() <= thr;
    }
    collision_active_.store(active, std::memory_order_release);

    if (snap.has_policy_traj && snap.stateTrajectory && !snap.stateTrajectory->empty()) {
        visualizer_->updateEndEffectorTrajectoryFromStates(*snap.stateTrajectory);
        visualizer_->publishEndEffectorTrajectory(node_->now());
    }

    auto& rec = trajectory_manager::TrajectoryRecorder::instance();
    if (rec.enabled()) {
        const double t = obs.time;
        const vector_t lp = visualizer_->computeEndEffectorPose(obs.state);
        trajectory_manager::TrajSample ls;
        ls.stamp_sec = t;
        ls.position = {lp(0), lp(1), lp(2)};
        ls.quat_xyzw = {lp(3), lp(4), lp(5), lp(6)};
        rec.appendReal("left", ls);
        const vector_t rp = visualizer_->computeRightEndEffectorPose(obs.state);
        trajectory_manager::TrajSample rs;
        rs.stamp_sec = t;
        rs.position = {rp(0), rp(1), rp(2)};
        rs.quat_xyzw = {rp(3), rp(4), rp(5), rp(6)};
        rec.appendReal("right", rs);

        if (snap.has_pred && snap.pred_state.size() > 0) {
            trajectory_manager::TrajSample lps;
            lps.stamp_sec = snap.pred_time;
            const vector_t lpe = visualizer_->computeEndEffectorPose(snap.pred_state);
            lps.position = {lpe(0), lpe(1), lpe(2)};
            lps.quat_xyzw = {lpe(3), lpe(4), lpe(5), lpe(6)};
            rec.appendPred("left", lps);
            trajectory_manager::TrajSample rps;
            rps.stamp_sec = snap.pred_time;
            const vector_t rpe = visualizer_->computeRightEndEffectorPose(snap.pred_state);
            rps.position = {rpe(0), rpe(1), rpe(2)};
            rps.quat_xyzw = {rpe(3), rpe(4), rpe(5), rpe(6)};
            rec.appendPred("right", rps);
        }
    }
}

vector_t WholeBodyMpcTask::calculateStaticTorques() const {
    const Eigen::VectorXd q = observation_.state;
    const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
    if (!rnea_data_) {
        rnea_data_.emplace(pinocchio_model);
    }
    return pinocchio::rnea(pinocchio_model, *rnea_data_, q, Eigen::VectorXd::Zero(pinocchio_model.nv),
                           Eigen::VectorXd::Zero(pinocchio_model.nv));
}

vector_t WholeBodyMpcTask::buildCollisionState() const {
    vector_t q = observation_.state;
    const auto& cmd = ctrl_interfaces_.last_sent_joint_positions_;
    if (interface_ && interface_->hasMobileBase()) {
        for (size_t i = 0; i < joint_names_.size() && i < cmd.size() && (3 + i) < static_cast<size_t>(q.size()); ++i) {
            q[static_cast<Eigen::Index>(3 + i)] = cmd[i];
        }
    } else {
        for (size_t i = 0; i < joint_names_.size() && i < cmd.size() && i < static_cast<size_t>(q.size()); ++i) {
            q[static_cast<Eigen::Index>(i)] = cmd[i];
        }
    }
    return q;
}

void WholeBodyMpcTask::publishCapabilities () const
{
    if (!capability_pub_ || !interface_) {
        return;
    }

    robot_control_msgs::msg::WholeBodyCapability msg;

    msg.has_mobile_base               = isMobileBasePlanningEnabled();
    msg.has_body_relative_constraint  = interface_->hasBodyRelativeConstraint();
    msg.has_waist_lock                = interface_->hasWaistLock();
    msg.has_head_coupling             = interface_->hasHeadCoupling();
    msg.has_custom_joint_lock         = interface_->hasCustomJointLock();
    msg.has_bimanual_coupling         = interface_->hasBimanualCoupling();
    msg.body_tracking_ee_enabled      = interface_->isBodyTrackingEeEnabled();
    msg.head_tracking_ee_enabled      = interface_->isHeadTrackingEeEnabled();
    msg.head_midpoint_gaze_enabled    = interface_->isHeadMidpointGazeEnabled();
    msg.has_home_joint_reference      = interface_->hasHomeJointReference();

    capability_pub_->publish(msg);

    if (!capabilities_info_logged_) {
        RCLCPP_INFO(node_->get_logger(),
            "[WBC Capability] basePlan=%d (modelMobile=%d) rel=%d waist=%d headCoupled=%d customLock=%d bimanual=%d bodyTrack=%d headTrack=%d homeJoint=%d",
            msg.has_mobile_base,
            interface_->hasMobileBase() ? 1 : 0,
            msg.has_body_relative_constraint,
            msg.has_waist_lock,
            msg.has_head_coupling,
            msg.has_custom_joint_lock,
            msg.has_bimanual_coupling,
            msg.body_tracking_ee_enabled,
            msg.head_tracking_ee_enabled,
            msg.has_home_joint_reference);
        capabilities_info_logged_ = true;
    }
}

void WholeBodyMpcTask::publishCurrentHumanoidState(size_t mode) const
{
    if (!current_state_pub_) {
        return;
    }

    const auto hm = HumanoidMode::decode(mode);

    robot_control_msgs::msg::WholeBodyCurrentState msg;

    // base
    msg.base_state = hm.baseLocked
        ? robot_control_msgs::msg::WholeBodyCurrentState::BASE_LOCKED
        : robot_control_msgs::msg::WholeBodyCurrentState::BASE_UNLOCKED;

    // body
    switch (hm.bodyMode) {
    case BodyConstraintMode::FREE:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_FREE;
        break;
    case BodyConstraintMode::RELATIVE:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_VERTICAL;
        break;
    case BodyConstraintMode::TRACKING:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_TRACKING;
        break;
    case BodyConstraintMode::LOCKED:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_LOCKED;
        break;
    case BodyConstraintMode::CUSTOM_LOCKED:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_CUSTOM_LOCKED;
        break;
    default:
        msg.body_state = robot_control_msgs::msg::WholeBodyCurrentState::BODY_FREE;
        break;
    }

    // bimanual
    msg.bimanual_state =
        (hm.bimanualMode == BimanualMode::COUPLED)
            ? robot_control_msgs::msg::WholeBodyCurrentState::BIMANUAL_COUPLED
            : robot_control_msgs::msg::WholeBodyCurrentState::BIMANUAL_INDEPENDENT;

    msg.head_state = static_cast<uint8_t>(hm.headMode);
    msg.head_midpoint_gaze_active = hm.headMode == HeadMode::GAZE &&
        interface_->isHeadMidpointGazeEnabled();

    // Arm bits follow the mode schedule only after the command hold is gone. An accepted
    // LEFT/RIGHT_ARM_ENABLE flips the schedule immediately, but the joints stay frozen in
    // WAITING_FOR_ENABLE_POLICY until the policy is continuous. Report that arm DISABLED
    // until the hold returns to IDLE; disable still reports DISABLED as soon as the schedule
    // says the arm is inactive.
    const auto reportedArmState = [this](bool modeActive, ArmSide side) {
        const bool released = modeActive && armPolicyHold(side).phase == ArmHoldPhase::IDLE;
        return released
            ? robot_control_msgs::msg::WholeBodyCurrentState::ARM_ENABLED
            : robot_control_msgs::msg::WholeBodyCurrentState::ARM_DISABLED;
    };
    msg.left_arm_state = reportedArmState(hm.leftArmActive, ArmSide::LEFT);
    msg.right_arm_state = reportedArmState(hm.rightArmActive, ArmSide::RIGHT);

    msg.home_joint_reference_enabled =
        interface_ && interface_->hasHomeJointReference() &&
        interface_->getReferenceManagerPtr() &&
        interface_->getReferenceManagerPtr()->isHomeJointReferenceEnabled();

    current_state_pub_->publish(msg);
}

void WholeBodyMpcTask::setHomeJointReferenceJoints(const std::vector<double>& home_joints) {
    if (!interface_ || home_joints.empty()) {
        return;
    }
    try {
        interface_->setHomeJointReferenceFromArmJoints(home_joints);
        const bool enabled = interface_->getReferenceManagerPtr() &&
                             interface_->getReferenceManagerPtr()->isHomeJointReferenceEnabled();
        RCLCPP_INFO(node_->get_logger(),
                    "HOME joint reference set from controller HOME (%zu joints), enabled=%s",
                    home_joints.size(), enabled ? "true" : "false");
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to set HOME joint reference: %s", e.what());
    }
}

bool WholeBodyMpcTask::handleHomeJointReferenceCommand(const std::string& cmd, size_t currentMode) {
    if (cmd != "HOME_JOINT_ON" && cmd != "HOME_JOINT_OFF") {
        return false;
    }
    if (!interface_ || !interface_->hasHomeJointReference() || !interface_->getReferenceManagerPtr()) {
        RCLCPP_WARN(node_->get_logger(),
                    "HOME joint reference command ignored (not enabled in task.info): %s", cmd.c_str());
        return true;
    }
    const bool enabled = (cmd == "HOME_JOINT_ON");
    interface_->getReferenceManagerPtr()->setHomeJointReferenceEnabled(enabled);
    publishCurrentHumanoidState(currentMode);
    RCLCPP_INFO(node_->get_logger(), "HOME joint reference %s", enabled ? "ON" : "OFF");
    return true;
}

void WholeBodyMpcTask::stopBase() {
    // 仅在曾允许底盘规划时发一次零速刹停；无 TF 时本就不发 cmd_vel，避免刷零。
    if (cmd_vel_publisher_ && isMobileBasePlanningEnabled()) {
        geometry_msgs::msg::Twist zero;
        cmd_vel_publisher_->publish(zero);
    }
}

void WholeBodyMpcTask::publishWorldTF(const rclcpp::Time& time) {
    // External TF present: never publish. Otherwise identity once for RViz (skip during probe).
    if (mobile_base_tf_available_) {
        world_tf_publish_done_ = true;
        return;
    }
    if (mobile_base_tf_probe_active_ || !mobile_base_tf_decided_) {
        return;
    }
    publishIdentityWorldTfOnce(time);
}
} // namespace ocs2::whole_body_mpc
