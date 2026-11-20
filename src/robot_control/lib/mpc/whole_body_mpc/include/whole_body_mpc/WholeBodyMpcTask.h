//
// WBC: WholeBodyInterface + MPC (mirrors arm_controller CtrlComponent).
//
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <atomic>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mpc_core/MpcRuntimeBase.h>
#include <mpc_core/reference/PoseBasedReferenceManager.h>
#include <mpc_core/rt/RtCycleTiming.h>
#include <mpc_core/visualization/OCS2PinocchioVisualizer.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <whole_body_mpc/WholeBodyInterface.h>
#include <whole_body_mpc/mode/HumanoidModeSchedule.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <robot_controller_core/CtrlInterfaces.h>
#include <trajectory_manager/TrajectoryRecorder.h>
#include <robot_control_msgs/msg/whole_body_capability.hpp>
#include <robot_control_msgs/msg/whole_body_current_state.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <pinocchio/multibody/data.hpp>

namespace ocs2::whole_body_mpc {

using CtrlInterfaces = robot_controller_core::CtrlInterfaces;

/** Read model frame defaults from task.info (eeFrame=left, eeFrame1=right). */
FrameOverrides loadInfoFrameDefaults(const std::string& task_file);

/**
 * CppAD cache directory, same layout idea as planning URDF:
 * ``${OCS2_PLANNING_URDF_DIR:-/tmp}/ocs2_ros2/<robot>/<subdir>/<content-hash>``.
 * The hash covers the task file and planning URDF, so a model change gets a new
 * empty directory and compiles once. Reboot clears /tmp.
 */
std::string cppadLibraryFolder(const std::string& robot_name, const std::string& subdir,
                                  const std::string& task_file, const std::string& urdf_file);

class WholeBodyMpcTask : public mpc_core::MpcRuntimeBase {
public:
    template<typename AutoDeclareFunc>
    explicit WholeBodyMpcTask(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, CtrlInterfaces& ctrl_interfaces,
                              AutoDeclareFunc auto_declare)
        : MpcRuntimeBase(node, ctrl_interfaces) {
        cached_ob_state_ = auto_declare("cached_ob_state", true);
        joint_speed_threshold_ = auto_declare("joint_speed_threshold", 0.1);
        hardware_latency_ = auto_declare("hardware_latency", 0.2);
        // Declare dir before enabled so both can be applied together from yaml/callback.
        traj_record_dir_ = auto_declare("traj_record_dir", std::string("/tmp/traj_record"));
        const bool traj_record_enabled = auto_declare("traj_record_enabled", false);
        // Sync initial yaml/launch value (parameter callbacks do not fire on declare).
        trajectory_manager::TrajectoryRecorder::instance()
            .setEnabled(traj_record_enabled, traj_record_dir_);

        declareRuntimeParameters(auto_declare);

        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                return on_parameter_change(parameters);
            });

        robot_name_ = auto_declare("robot_name", std::string("fa_w2"));
        planning_urdf_variant_ = auto_declare("planning_urdf_variant", std::string(""));
        planning_urdf_path_ = auto_declare("planning_urdf_path", std::string(""));
        future_time_offset_ = auto_declare("future_time_offset", 1.0);
        const std::string info_file_name = auto_declare("info_file_name", std::string("whole_body_fixed_eef_l6"));
        /// Subdirectory name under the /tmp CppAD cache. Not a path inside the description share.
        const std::string cppad_library_subdir = auto_declare("cppad_library_subdir", std::string("whole_body_mpc"));
        joint_names_ = node_->get_parameter("joints").as_string_array();
        // Package holding config/ocs2/<info_file_name>.info. Empty = <robot_name>_description.
        const std::string info_package = auto_declare("info_package", std::string(""));
        const std::string robot_pkg = info_package.empty() ? robot_name_ + "_description" : info_package;
        const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

        const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
        const std::string urdf_file = resolvePlanningUrdfPath();
        const std::string lib_folder =
            cppadLibraryFolder(robot_name_, cppad_library_subdir, task_file, urdf_file);
        RCLCPP_INFO(node_->get_logger(), "CppAD library folder: %s", lib_folder.c_str());

        const auto info_frames = loadInfoFrameDefaults(task_file);
        const std::string base_frame_param =
            auto_declare("base_frame", info_frames.baseFrame);
        const std::string left_ee_frame =
            auto_declare("left_ee_frame", info_frames.eeFrame);
        const std::string right_ee_frame =
            auto_declare("right_ee_frame", info_frames.eeFrame1);
        const std::string body_frame_param =
            auto_declare("body_frame", info_frames.bodyFrame);
        const std::string head_frame_param =
            auto_declare("head_frame", info_frames.headFrame);
        const bool recompile_libraries = auto_declare(
            "recompile_libraries", info_frames.recompileLibraries.value_or(true));
        FrameOverrides frame_overrides;
        frame_overrides.baseFrame = base_frame_param;
        frame_overrides.eeFrame = left_ee_frame;
        frame_overrides.eeFrame1 = right_ee_frame;
        frame_overrides.bodyFrame = body_frame_param;
        frame_overrides.headFrame = head_frame_param;
        frame_overrides.recompileLibraries = recompile_libraries;
        RCLCPP_INFO(node_->get_logger(), "recompile_libraries=%s",
                    recompile_libraries ? "true" : "false");

        setupInterface(task_file, lib_folder, urdf_file, frame_overrides);
        if (interface_ && interface_->hasHomeJointReference() &&
            interface_->homeJointReferenceDefaultEnabled()) {
            interface_->getReferenceManagerPtr()->setHomeJointReferenceEnabled(true);
        }

        // WheelBasedMobileManipulator 的 stateDim = 3(base x,y,yaw) + armDim，
        // 但 joint_names_ 只包含实际关节，不含虚拟底盘自由度，因此应与 armDim 比较。
        const size_t expected_joints = interface_->getModelInfo().armDim;
        if (joint_names_.size() != expected_joints) {
            RCLCPP_FATAL(node_->get_logger(),
                         "Parameter 'joints' size (%zu) must equal OCS2 armDim (%zu) (Pinocchio nq after removeJoints).",
                         joint_names_.size(), expected_joints);
            throw std::invalid_argument("WholeBodyMpcTask: joints count does not match OCS2 arm dimension");
        }

        const auto& minfo = interface_->getModelInfo();
        if (minfo.eeFrame.empty() || minfo.eeFrame1.empty() || minfo.eeFrame1 == minfo.eeFrame) {
            throw std::invalid_argument(
                "WholeBodyMpcTask requires dual-arm model_information.eeFrame and eeFrame1 "
                "(distinct, non-empty)");
        }
        // Diff base: inputDim - armDim = 2 (v, omega); omni base: 3 (vx, vy, omega).
        base_input_dim_ = static_cast<size_t>(minfo.inputDim - minfo.armDim);
        if (interface_->hasMobileBase()) {
            RCLCPP_INFO(node_->get_logger(),
                        "Mobile base input dim: %zu (%s)",
                        base_input_dim_,
                        base_input_dim_ == 3 ? "omni vx,vy,omega" : "diff v,omega");
            loadBaseCmdVelocityLimits(task_file);
            setupBaseVelocityTarget(auto_declare("base_velocity_target_timeout_sec", 0.2));
        }

        setupPublisher();

        {
            mpc_core::OCS2VisualizerConfig viz_cfg;
            viz_cfg.urdf_file = urdf_file;
            viz_cfg.dual_arm = true;
            viz_cfg.base_frame = interface_->hasMobileBase() ? "world" : minfo.baseFrame;
            viz_cfg.left_ee_frame = minfo.eeFrame;
            viz_cfg.right_ee_frame = minfo.eeFrame1;
            viz_cfg.body_frame = minfo.bodyFrame;
            viz_cfg.head_frame = minfo.headFrame;
            if (auto geo = interface_->getPinocchioGeometryInterface()) {
                viz_cfg.pinocchio_geometry = std::move(*geo);
                viz_cfg.self_collision_activation_distance = interface_->getSelfCollisionActivationDistance();
            }
            visualizer_ = std::make_unique<ocs2::mpc_core::OCS2PinocchioVisualizer>(
                node_, interface_->getPinocchioInterface(), std::move(viz_cfg));
        }
        visualizer_->initialize();
        visualizer_->setPosePublishPeriod(dds_publish_period_sec_);
        RCLCPP_INFO(node_->get_logger(), "Future time offset: %.2f seconds", future_time_offset_);

        auto_declare("movel_trajectory_duration", 2.0);
        auto_declare("movel_duration", 2.0);
        auto_declare("movel_sample_interval", 0.04);
        auto_declare("movel_max_linear_velocity", 0.3);
        auto_declare("movel_max_linear_acceleration", 1.0);
        auto_declare("movel_max_linear_jerk", 2.0);
        auto_declare("movel_max_angular_velocity", 1.0);
        auto_declare("movel_max_angular_acceleration", 2.0);
        auto_declare("movel_max_angular_jerk", 4.0);
        auto_declare("movel_auto_extend_duration", true);

        mpc_core::OCS2ReferenceTargetContext ref_ctx;
        ref_ctx.dual_arm = true;
        // 轮式底盘模式下 OCS2 EE 坐标均在 world 坐标系，须与 publishCurrentTargets 的 frame_id 一致
        ref_ctx.base_frame = interface_->hasMobileBase() ? "world" : minfo.baseFrame;
        ref_ctx.input_dim = minfo.inputDim;
        bodyTrackingEeEnabled_ = interface_->isBodyTrackingEeEnabled();
        headTrackingEeEnabled_ = interface_->isHeadTrackingEeEnabled();
        head_frame_ = minfo.headFrame;
        ref_ctx.body_target_enabled = bodyTrackingEeEnabled_;
        ref_ctx.head_target_enabled = headTrackingEeEnabled_;
        if (bodyTrackingEeEnabled_) {
            ref_ctx.body_pose_from_current_state = auto_declare("body_pose_from_current_state", true);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Reference target layout: dim=%d body=%s head=%s",
                    ref_ctx.wholeBodyTargetStateDim(),
                    bodyTrackingEeEnabled_ ? "enabled" : "disabled",
                    headTrackingEeEnabled_ ? "enabled" : "disabled");
        pose_reference_manager_ = std::make_shared<ocs2::mpc_core::PoseBasedReferenceManager>(
            robot_name_, interface_->getReferenceManagerPtr(), ref_ctx);
        // Drop this arm's EE topics while it is disabled or its enable hold has not released.
        // The resume pose written by reset*EndEffectorTarget is not an external command.
        pose_reference_manager_->setArmExternalCommandPredicate([this](bool is_left) {
            const auto& blocked = is_left ? block_left_external_ee_ : block_right_external_ee_;
            return !blocked.load(std::memory_order_acquire);
        });
        pose_reference_manager_->subscribe(node_);

        const std::string humanoid_mode_topic = auto_declare("humanoid_mode_topic", std::string("mode_command"));
        setupHumanoidModeSubscription(humanoid_mode_topic);

        // Mobile-base pose from external TF only. Probe world->baseFrame at activate
        // (unlock planning if mobile; skip identity TF publish if external TF exists).
        world_frame_ = auto_declare("world_frame", std::string("world"));
        base_frame_ = minfo.baseFrame;
        if (base_frame_.empty()) {
            throw std::invalid_argument("WholeBodyMpcTask: model_information.baseFrame is empty");
        }
        mobile_base_tf_timeout_sec_ = auto_declare("mobile_base_tf_timeout_sec", 2.0);
        // Relative "tf" → /<ns>/tf under multi-robot namespaces (Isaac dual G1D).
        // Absolute "/tf" misses /g1d/tf|/g1d_01/tf and disables unlocked-base planning.
        const std::string tf_topic = auto_declare("tf_topic", std::string("tf"));
        setupTfBuffer(tf_topic);
        if (interface_->hasMobileBase()) {
            RCLCPP_INFO(node_->get_logger(),
                        "Mobile base model: unlocked-base planning requires %s->%s on TF topic %s "
                        "(timeout %.1fs at activate).",
                        world_frame_.c_str(), base_frame_.c_str(), tf_topic.c_str(),
                        mobile_base_tf_timeout_sec_);
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "Fixed base model: probe %s->%s on %s at activate; skip identity TF if external TF exists.",
                        world_frame_.c_str(), base_frame_.c_str(), tf_topic.c_str());
        }

        {
            const auto& initial_schedule = interface_->getReferenceManagerPtr()->getModeSchedule();
            if (initial_schedule.modeSequence.empty()) {
                throw std::invalid_argument("WholeBodyMpcTask: initialModeSchedule has no modes");
            }
            fsm_default_humanoid_mode_ = initial_schedule.modeSequence.front();

            const std::string fsm_default_mode_str =
                auto_declare("fsm_default_humanoid_mode", std::string(""));
            if (!fsm_default_mode_str.empty()) {
                try {
                    auto default_mode = HumanoidMode::decode(fsm_default_humanoid_mode_);
                    auto override_mode = HumanoidMode::decode(
                        stringToMode(fsm_default_mode_str));
                    override_mode.headMode = default_mode.headMode;
                    fsm_default_humanoid_mode_ = override_mode.encode();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(node_->get_logger(),
                                "Invalid fsm_default_humanoid_mode '%s'; keeping .info default (%s): %s",
                                fsm_default_mode_str.c_str(),
                                modeToString(fsm_default_humanoid_mode_).c_str(), e.what());
                }
            }

            const std::string default_head_mode = auto_declare("fsm_default_head_mode", std::string(""));
            if (!default_head_mode.empty()) {
                try {
                    fsm_default_humanoid_mode_ = applyPartialHumanoidModeString(
                        fsm_default_humanoid_mode_, default_head_mode);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(node_->get_logger(),
                                "Invalid fsm_default_head_mode '%s'; keeping .info default: %s",
                                default_head_mode.c_str(), e.what());
                }
            }
            // Construct-time: model capabilities only (TF probe runs on activate).
            if (!isHumanoidModeStructurallySupported(fsm_default_humanoid_mode_)) {
                auto hm = HumanoidMode::decode(fsm_default_humanoid_mode_);
                const auto original_mode = fsm_default_humanoid_mode_;
                if (!hm.baseLocked && !interface_->hasMobileBase()) {
                    hm.baseLocked = true;
                }
                if (hm.bodyMode == BodyConstraintMode::TRACKING && !interface_->isBodyTrackingEeEnabled()) {
                    hm.bodyMode = BodyConstraintMode::FREE;
                }
                if (hm.headMode == HeadMode::TRACKING && !interface_->isHeadTrackingEeEnabled()) {
                    hm.headMode = HeadMode::DISABLED;
                }
                if (hm.headMode == HeadMode::GAZE && !interface_->isHeadMidpointGazeEnabled()) {
                    hm.headMode = HeadMode::DISABLED;
                }
                if (hm.bodyMode == BodyConstraintMode::RELATIVE && !interface_->hasBodyRelativeConstraint()) {
                    hm.bodyMode = BodyConstraintMode::FREE;
                }
                if (hm.bodyMode == BodyConstraintMode::LOCKED && !interface_->hasWaistLock()) {
                    hm.bodyMode = BodyConstraintMode::FREE;
                }
                if (hm.headMode == HeadMode::FORWARD && !interface_->hasHeadCoupling()) {
                    hm.headMode = HeadMode::DISABLED;
                }
                if (hm.bodyMode == BodyConstraintMode::CUSTOM_LOCKED && !interface_->hasCustomJointLock()) {
                    hm.bodyMode = BodyConstraintMode::FREE;
                }
                if (hm.bimanualMode == BimanualMode::COUPLED && !interface_->hasBimanualCoupling()) {
                    hm.bimanualMode = BimanualMode::INDEPENDENT;
                }
                fsm_default_humanoid_mode_ = hm.encode();
                RCLCPP_WARN(node_->get_logger(),
                            "fsm_default_humanoid_mode '%s' (%s) requires unsupported capabilities; degraded to %s",
                            fsm_default_mode_str.c_str(),
                            modeToString(original_mode).c_str(),
                            modeToString(fsm_default_humanoid_mode_).c_str());
            }

        }

        prepareThreadIsolation();
        mpc_ = std::make_unique<GaussNewtonDDP_MPC>(interface_->mpcSettings(), interface_->ddpSettings(), interface_->getRollout(),
                                                      interface_->getOptimalControlProblem(), interface_->getInitializer());
        ocs2::ThreadPool::setLaunchWorkerCpuAffinity({});

        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

        mpc_->getSolverPtr()->setReferenceManager(pose_reference_manager_);

        observation_.state = interface_->getInitialState();
        observation_.input = vector_t::Zero(minfo.inputDim);
        observation_.time = 0.0;

        if (interface_->hasMobileBase()) {
            base_x_ = observation_.state(0);
            base_y_ = observation_.state(1);
            base_yaw_ = observation_.state(2);
        }

        last_execute_time_ = node_->now();
        cached_last_action_ = observation_.state;
    }

    ~WholeBodyMpcTask() override { stopVisualizationThread(); }

    void updateObservation(const rclcpp::Time& time) override;
    void evaluatePolicy(const rclcpp::Time& time) override;
    void resetMpc() override;
    [[nodiscard]] const pinocchio::Model* planningModel() const override
    {
        return interface_ ? &interface_->getPinocchioInterface().getModel() : nullptr;
    }
    [[nodiscard]] bool selfCollisionEnabled() const override { return interface_->isSelfCollisionEnabled(); }
    [[nodiscard]] double selfCollisionMinimumDistance() const override
    {
        return interface_->getSelfCollisionMinimumDistance();
    }
    /** OCS2 state left: stop the base and put the humanoid mode back to the FSM default. */
    void onOCS2Exit() override
    {
        stopBase();
        resetHumanoidModeToFsmDefault();
    }
    vector_t calculateStaticTorques() const;

    void publishCapabilities() const;
    void publishCurrentHumanoidState(size_t mode) const;
    /** Startup: copy controller HOME joints as the switchable joint reference. */
    void setHomeJointReferenceJoints(const std::vector<double>& home_joints);
    /** FSM leaving OCS2 / controller activation: clear coupling and set ModeSchedule to fsm_default_humanoid_mode. */
    void resetHumanoidModeToFsmDefault();
    /** No external world TF: publish identity static world->base once for RViz; with external TF: no-op. */
    void publishWorldTF(const rclcpp::Time& time);
    /** Start activate-time window probing external world->base TF (planning gate + identity-TF skip). */
    void beginMobileBaseTfProbe();
    /** Model has mobile base AND external world TF was confirmed in the startup probe window. */
    [[nodiscard]] bool isMobileBasePlanningEnabled() const;
    /** Publish zero base_velocity_command once to stop the base (only if mobile planning/TF is active).
     *  Called on OCS2 exit and on unlocked→locked mode transitions (BASE_LOCK / lock presets). */
    void stopBase();

    std::shared_ptr<WholeBodyInterface> interface_;

private:
    /** EE pose using the same getBodyId frames as PinocchioEndEffectorKinematics / coupling. */
    void computeCouplingCapturePoses(const vector_t& q, vector_t& left_pose7, vector_t& right_pose7) const;
    [[nodiscard]] vector_t computeFramePose(const vector_t& q, const std::string& frame) const;
    bool handleHomeJointReferenceCommand(const std::string& cmd, size_t currentMode);
    void captureBimanualCouplingFromState(const vector_t& q);
    /** Under COUPLED: refresh right marker from measured left so RViz matches the constraint. */
    void refreshCoupledRightMarkerFromMeasurement();
    void setupInterface(const std::string& task_file, const std::string& lib_folder, const std::string& urdf_file,
                        const FrameOverrides& frame_overrides = FrameOverrides{});
    /** Load base_velocity_command hard clamps from task.info jointVelocityLimits.*.base.<modelType>. */
    void loadBaseCmdVelocityLimits(const std::string& task_file);
    void setupPublisher();
    /** The operator's base velocity (XrInputHandler): becomes the desired base input of the MPC. */
    void setupBaseVelocityTarget(double timeout_sec);
    void setupHumanoidModeSubscription(const std::string& topic);
    void humanoidModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void setupTfBuffer(const std::string& topic);
    void tfMsgCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void lookupBasePoseFromBuffer();
    [[nodiscard]] bool tryLookupMobileBasePoseFromTf();
    void finalizeMobileBaseTfProbe(bool available);
    void updateMobileBaseTfProbe();
    void publishIdentityWorldTfOnce(const rclcpp::Time& time);

    enum class ArmSide { LEFT, RIGHT };
    enum class ArmHoldPhase {
        IDLE,
        WAITING_FOR_DISABLE_POLICY,
        DISABLED_HOLD,
        WAITING_FOR_ENABLE_POLICY,
    };

    struct ArmPolicyTransitionHold {
        ArmHoldPhase phase{ArmHoldPhase::IDLE};
        uint64_t requiredPolicyGeneration{0};
        std::vector<double> heldCommandPositions;
        size_t consecutiveReadyCycles{0};
        rclcpp::Time startedAt{0, 0, RCL_ROS_TIME};
        [[nodiscard]] bool commandHoldActive() const {
            return phase != ArmHoldPhase::IDLE;
        }
    };

    // Enable release is a continuity check, not a "trajectory stays at the frozen pose" check.
    // Position is written from a future lookahead point, so even a continuous path reads as a
    // position offset there. The joint command starts at the frozen position and then moves with
    // the policy's joint velocity; keeping that velocity small for a few consecutive cycles is
    // what actually guarantees a continuous handover. A position-goal target on a stationary arm
    // holds a ~0.06 rad/s MPC velocity floor, so 0.1 rad/s releases intentional motion but still
    // rejects a discontinuous first command.
    static constexpr double kArmPolicyReleaseVelocityTolerance = 0.1;
    static constexpr size_t kArmPolicyReleaseConsecutiveCycles = 5;

    bool beginArmPolicyTransition(ArmSide side, bool requestedActive);
    [[nodiscard]] vector_t buildStateWithArmHeldCommand(
        ArmSide side, const vector_t& measuredState) const;
    void updateArmPolicyTransitionReadiness(
        size_t plannedMode, const vector_t& futureState, const vector_t& futureInput);
    void applyArmTransitionHoldToCommands();
    void clearArmPolicyTransitions();
    /** Mirror a non-IDLE arm hold into the atomics read by EE topic callbacks. */
    void publishExternalEeCommandBlockFromHolds();
    /** After an MPC reset, rebuild holds from the current mode. Inactive arms become DISABLED_HOLD. */
    void seedArmPolicyHoldsFromCurrentMode();
    ArmPolicyTransitionHold& armPolicyHold(ArmSide side);
    const ArmPolicyTransitionHold& armPolicyHold(ArmSide side) const;

    /** Applies queued mode_command strings in order; runs in updateObservation (any FSM state including HOLD). */
    void applyPendingHumanoidModeIfAny();
    /** Model capabilities only (no external-TF gate). Used at construct-time. */
    bool isHumanoidModeStructurallySupported(size_t mode) const;
    /** Structural support + unlocked-base requires confirmed external world TF. */
    bool isHumanoidModeSupported(size_t mode) const;

    rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr humanoid_mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_velocity_target_sub_;
    rclcpp::Publisher<robot_control_msgs::msg::WholeBodyCapability>::SharedPtr capability_pub_;
    mutable bool capabilities_info_logged_{false};
    rclcpp::Publisher<robot_control_msgs::msg::WholeBodyCurrentState>::SharedPtr current_state_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /** Topic callback enqueues commands; updateObservation applies them in order (HOLD/OCS2/any FSM). */
    std::mutex pending_humanoid_mode_mutex_;
    std::deque<std::string> pending_humanoid_mode_cmds_;
    static constexpr size_t kMaxPendingHumanoidModeCmds = 32;

    void runVisualizationOnce(const VisualizationSnapshot& snap) override;
    /** Observation + last_sent_joint_positions_ overlay (same as former checkSelfCollisionOnCommand). */
    vector_t buildCollisionState() const override;

    /** Mobile-base input size: 2 (diff v,omega) or 3 (omni vx,vy,omega); 0 if fixed base. */
    size_t base_input_dim_{0};
    /** Hard clamps for published base_velocity_command (from task.info jointVelocityLimits base bounds). */
    vector_t base_cmd_lower_;
    vector_t base_cmd_upper_;
    bool bodyTrackingEeEnabled_{false};
    bool headTrackingEeEnabled_{false};
    std::string head_frame_;
    double future_time_offset_{1.0};
    bool cached_ob_state_{true};
    double joint_speed_threshold_{0.1};
    rclcpp::Time last_execute_time_;
    vector_t cached_last_action_;
    double hardware_latency_{0.2};
    std::string traj_record_dir_{"/tmp/traj_record"};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& parameters);

    size_t fsm_default_humanoid_mode_{};

    std::string world_frame_;
    /// From task info `model_information.baseFrame` (TF lookup / identity publish child).
    std::string base_frame_;
    double mobile_base_tf_timeout_sec_{2.0};
    /// Probe window after beginMobileBaseTfProbe(); until decided, unlocked-base modes are rejected.
    bool mobile_base_tf_probe_active_{false};
    bool mobile_base_tf_decided_{false};
    bool mobile_base_tf_available_{false};
    rclcpp::Time mobile_base_tf_probe_start_{0, 0, RCL_ROS_TIME};
    bool world_tf_publish_done_{false};

    // 底盘位姿：仅在控制线程由外部 TF lookup 更新（TF 回调只写 tf_buffer_）。
    double base_x_{0.0};
    double base_y_{0.0};
    double base_yaw_{0.0};
    // 无外部 world TF 时发一次 identity static TF，供可视化（避免动态 TF 超时过期）。
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    ArmPolicyTransitionHold left_arm_policy_hold_;
    ArmPolicyTransitionHold right_arm_policy_hold_;
    /** Topic callbacks reject this arm's EE command while its hold is not IDLE. */
    std::atomic<bool> block_left_external_ee_{false};
    std::atomic<bool> block_right_external_ee_{false};
    uint64_t active_policy_generation_{0};

    mutable std::optional<pinocchio::Data> rnea_data_;
};

} // namespace ocs2::whole_body_mpc
