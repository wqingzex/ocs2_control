//
// MPC task for one or two arms (OCS2 mobile manipulator).
//
#pragma once


#include <cstdint>
#include <cstdlib>

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>
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
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>

#include <robot_controller_core/CtrlInterfaces.h>
#include <trajectory_manager/TrajectoryRecorder.h>
#include <pinocchio/multibody/data.hpp>

namespace ocs2::arm_mpc
{
    using mobile_manipulator::FrameOverrides;
    using mobile_manipulator::MobileManipulatorInterface;

    // Use CtrlInterfaces from robot_controller_core
    using CtrlInterfaces = robot_controller_core::CtrlInterfaces;

    /**
     * CppAD cache, same layout as planning URDF and WBC:
     * ``${OCS2_PLANNING_URDF_DIR:-/tmp}/ocs2_ros2/<robot>/ocs2/<content-hash>``.
     * Hash covers the task file and planning URDF. An explicit ocs2_library_folder
     * parameter still overrides this.
     */
    std::string defaultOCS2LibraryFolder(const std::string& robot_name, const std::string& task_file,
                                         const std::string& urdf_file);

    /** Read model frame defaults from task.info (eeFrame=left, eeFrame1=right). */
    FrameOverrides loadInfoFrameDefaults(const std::string& task_file);

    class ArmMpcTask : public mpc_core::MpcRuntimeBase
    {
    public:
        template <typename AutoDeclareFunc>
        explicit ArmMpcTask(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces,
                               AutoDeclareFunc auto_declare)
            : MpcRuntimeBase(node, ctrl_interfaces)
        {
            cached_ob_state_ = auto_declare("cached_ob_state", true);
            joint_speed_threshold_ = auto_declare("joint_speed_threshold", 0.1);
            hardware_latency_ = auto_declare("hardware_latency", 0.2);
            // Declare dir before enabled so both can be applied together from yaml/callback.
            traj_record_dir_ = auto_declare("traj_record_dir", std::string("/tmp/traj_record"));
            const bool traj_record_enabled = auto_declare("traj_record_enabled", false);
            robot_controller_core::TrajectoryRecorder::instance()
                .setEnabled(traj_record_enabled, traj_record_dir_);

            declareRuntimeParameters(auto_declare);

            parameter_callback_handle_ = node_->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter>& parameters) {
                    return on_parameter_change(parameters);
                });

            robot_name_ = auto_declare("robot_name", std::string("cr5"));
            planning_urdf_variant_ = auto_declare("planning_urdf_variant", std::string(""));
            planning_urdf_path_ = auto_declare("planning_urdf_path", std::string(""));
            future_time_offset_ = auto_declare("future_time_offset", 1.0);
            const std::string info_file_name = auto_declare("info_file_name", std::string("task"));
            joint_names_ = node_->get_parameter("joints").as_string_array();
            const std::string robot_pkg = robot_name_ + "_description";
            const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

            const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
            const std::string urdf_file = resolvePlanningUrdfPath();
            const std::string lib_folder = auto_declare(
                "ocs2_library_folder", defaultOCS2LibraryFolder(robot_name_, task_file, urdf_file));
            RCLCPP_INFO(node_->get_logger(), "CppAD library folder: %s", lib_folder.c_str());

            // Frame / model_settings params: YAML overrides task.info; applied in-memory into Interface.
            const auto info_frames = loadInfoFrameDefaults(task_file);
            const std::string base_frame =
                auto_declare("base_frame", info_frames.baseFrame);
            const std::string left_ee_frame =
                auto_declare("left_ee_frame", info_frames.eeFrame);
            const std::string right_ee_frame =
                auto_declare("right_ee_frame", info_frames.eeFrame1);
            const bool recompile_libraries = auto_declare(
                "recompile_libraries", info_frames.recompileLibraries.value_or(true));
            FrameOverrides frame_overrides;
            frame_overrides.baseFrame = base_frame;
            frame_overrides.eeFrame = left_ee_frame;
            frame_overrides.eeFrame1 = right_ee_frame;
            frame_overrides.recompileLibraries = recompile_libraries;
            RCLCPP_INFO(node_->get_logger(), "recompile_libraries=%s",
                        recompile_libraries ? "true" : "false");

            setupInterface(task_file, lib_folder, urdf_file, frame_overrides);

            dual_arm_mode_ = interface_->dual_arm_;

            setupPublisher();

            {
                ocs2::mpc_core::OCS2VisualizerConfig viz_cfg;
                viz_cfg.urdf_file = urdf_file;
                viz_cfg.dual_arm = interface_->dual_arm_;
                const auto& minfo = interface_->getManipulatorModelInfo();
                viz_cfg.base_frame = minfo.baseFrame;
                viz_cfg.left_ee_frame = minfo.eeFrame;
                viz_cfg.right_ee_frame = minfo.eeFrame1;
                if (auto geo = interface_->getPinocchioGeometryInterface())
                {
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
            ref_ctx.dual_arm = interface_->dual_arm_;
            ref_ctx.base_frame = interface_->getManipulatorModelInfo().baseFrame;
            ref_ctx.input_dim = interface_->getManipulatorModelInfo().inputDim;
            pose_reference_manager_ = std::make_shared<ocs2::mpc_core::PoseBasedReferenceManager>(
                robot_name_, interface_->getReferenceManagerPtr(), ref_ctx);
            pose_reference_manager_->subscribe(node_);

            prepareThreadIsolation();
            mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
                interface_->mpcSettings(),
                interface_->ddpSettings(),
                interface_->getRollout(),
                interface_->getOptimalControlProblem(),
                interface_->getInitializer());
            ocs2::ThreadPool::setLaunchWorkerCpuAffinity({});

            mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

            mpc_->getSolverPtr()->setReferenceManager(pose_reference_manager_);

            observation_.state = interface_->getInitialState();
            observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
            observation_.time = 0.0;

            // Initialize cached state
            last_execute_time_ = node_->now();
            cached_last_action_ = observation_.state;
        }

        ~ArmMpcTask() override { stopVisualizationThread(); }

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

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        // OCS2 interface (public access)
        std::shared_ptr<MobileManipulatorInterface> interface_;

        std::string getUrdfFilePath() const { return urdf_file_; }
        std::string getTaskFilePath() const { return task_file_; }

    protected:
        void runVisualizationOnce(const VisualizationSnapshot& snap) override;
        vector_t buildCollisionState() const override;

    private:
        void setupInterface(const std::string& task_file,
                            const std::string& lib_folder,
                            const std::string& urdf_file,
                            const FrameOverrides& frame_overrides = FrameOverrides{});
        void setupPublisher();

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;

        // Configuration
        bool dual_arm_mode_;
        double future_time_offset_; // Future time offset
        /// cached MPC observation state (which is different from real observation on purpose)
        bool cached_ob_state_;
        double joint_speed_threshold_;
        rclcpp::Time last_execute_time_;
        vector_t cached_last_action_;
        double hardware_latency_;
        std::string traj_record_dir_{"/tmp/traj_record"};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rcl_interfaces::msg::SetParametersResult on_parameter_change(
            const std::vector<rclcpp::Parameter>& parameters);

        std::string urdf_file_;
        std::string task_file_;

        mutable std::optional<pinocchio::Data> rnea_data_;
    };
}
