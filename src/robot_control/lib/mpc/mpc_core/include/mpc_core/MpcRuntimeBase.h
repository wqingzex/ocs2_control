//
// Runtime machinery shared by the OCS2 control components (arm and whole-body):
// RT-cycle timing, CPU / thread scheduling, the MPC advance wrapper, the self-collision and trajectory
// visualization worker thread, and the small publish / hold helpers the FSM states call.
//
// Domain-specific parts (observation layout, policy evaluation, MPC reset, self-collision limits, what the
// visualization thread draws) stay in the derived component and are reached through the virtuals below.
//
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <mpc_core/reference/PoseBasedReferenceManager.h>
#include <mpc_core/rt/RtCycleTiming.h>
#include <mpc_core/visualization/OCS2PinocchioVisualizer.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>

#include <robot_controller_core/CtrlInterfaces.h>

namespace ocs2::mpc_core
{
    class MpcRuntimeBase
    {
    public:
        MpcRuntimeBase(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                       robot_controller_core::CtrlInterfaces& ctrl_interfaces)
            : node_(node), ctrl_interfaces_(ctrl_interfaces)
        {
        }

        virtual ~MpcRuntimeBase() = default;

        // ---- domain-specific, implemented by the arm / whole-body component ----

        /** RT: read joint states into observation_ and refresh the end-effector poses. */
        virtual void updateObservation(const rclcpp::Time& time) = 0;
        /** RT: take the latest MPC policy and write the joint command. */
        virtual void evaluatePolicy(const rclcpp::Time& time) = 0;
        /** Reset observation/targets and the MPC node. Does NOT block for the first policy (RT-safe). */
        virtual void resetMpc() = 0;
        /** Planning model of the OCS2 interface; nullptr when the interface could not be built. */
        [[nodiscard]] virtual const pinocchio::Model* planningModel() const = 0;
        [[nodiscard]] virtual bool selfCollisionEnabled() const = 0;
        /** Distance [m] at or below which the OCS2 state hands over to HOLD. */
        [[nodiscard]] virtual double selfCollisionMinimumDistance() const = 0;
        /** Called once the OCS2 state has stopped its MPC thread and cleared the trajectory visualization. */
        virtual void onOCS2Exit() {}

        /** True once the first MPC policy after resetMpc() is available. */
        [[nodiscard]] bool initialPolicyReceived() const;
        void advanceMpc() const;
        /** MPC request period; used to warn only when a solve overruns that period. */
        void setMpcPeriodSec(double period_sec);
        /** Hold joints at last commanded positions (used while waiting for initial policy). */
        void holdLastSentPositions() const;
        /** Publish the cached current targets (call on OCS2 enter for marker sync). */
        void publishCachedCurrentTargets() const;
        /** Publish an FSM command (e.g. 2 = HOLD) to all controllers. */
        void publishFsmCommand(int32_t command) const;

        /**
         * Self-collision markers + trajectory recording at MPC cadence on a dedicated thread
         * (not the RT update). FCL judgment runs on the viz thread; RT only reads collision_active_.
         * Start on controller activate, stop on deactivate. Request updates via
         * maybeRequestVisualizationUpdate() from the common control path (all FSM states).
         */
        void startVisualizationThread(int thread_sleep_ms, double visualization_period_sec);
        /** Blocking stop: clear running + join (destructor / deactivate). */
        void stopVisualizationThread();
        void requestVisualizationUpdate();
        /** Throttle requestVisualizationUpdate to visualization_period_sec (MPC rate). */
        void maybeRequestVisualizationUpdate(const rclcpp::Time& time);
        void clearTrajectoryVisualization() const;

        void beginRtCycle() { rt_timing_.beginCycle(); }
        void endRtCycle(const std::string& fsm) { rt_timing_.endCycle(fsm, node_->get_logger()); }
        RtCycleTiming::Scope rtScopeObs() { return rt_timing_.scope(&rt_timing_.obs_us); }
        RtCycleTiming::Scope rtScopeViz() { return rt_timing_.scope(&rt_timing_.viz_us); }
        void applyWorkerThreadScheduling(const char* thread_name) const;
        /** Pin the controller_manager update thread away from the MPC core (once). */
        void applyRtLoopSchedulingOnce();

        /**
         * RT-safe: the viz thread already compared the FCL min distance to the configured threshold.
         * Update/FSM only consume this flag (no threshold re-judgment).
         */
        bool isSelfCollisionActive() const
        {
            return collision_active_.load(std::memory_order_acquire);
        }

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode() const { return node_; }

        // MPC components
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        /** True after the first successful updatePolicy() swap; cleared in resetMpc(). */
        bool policy_active_{false};
        std::shared_ptr<PoseBasedReferenceManager> pose_reference_manager_;

        // Observation state
        SystemObservation observation_;
        vector_t optimized_state_;
        vector_t optimized_input_;

    protected:
        /** Snapshot handed from the RT loop to the visualization thread. */
        struct VisualizationSnapshot
        {
            SystemObservation observation;
            /** Command-overlay q for FCL + collision markers (obs base + last_sent joints). */
            vector_t collision_state;
            bool has_pred{false};
            double pred_time{0.0};
            vector_t pred_state;
            bool has_policy_traj{false};
            std::shared_ptr<const vector_array_t> stateTrajectory;
        };

        /** Runs on the viz thread for every snapshot: self-collision, trajectory, recording. */
        virtual void runVisualizationOnce(const VisualizationSnapshot& snap) = 0;
        /** Joint configuration used for the collision check (observation overlaid with the last command). */
        virtual vector_t buildCollisionState() const = 0;

        /** Declares and reads rt_timing_enabled, thread_priority, cpu_affinity, dds_publish_hz. */
        template <typename AutoDeclareFunc>
        void declareRuntimeParameters(AutoDeclareFunc& auto_declare)
        {
            rt_timing_.enabled.store(auto_declare("rt_timing_enabled", false), std::memory_order_relaxed);
            thread_priority_ = auto_declare("thread_priority", 0);
            {
                const auto affinity_i64 = auto_declare("cpu_affinity", std::vector<int64_t>{});
                cpu_affinity_.clear();
                cpu_affinity_.reserve(affinity_i64.size());
                for (const auto cpu : affinity_i64)
                {
                    cpu_affinity_.push_back(static_cast<int>(cpu));
                }
            }
            dds_publish_hz_ = auto_declare("dds_publish_hz", 50.0);
            dds_publish_period_sec_ = dds_publish_hz_ > 0.0 ? 1.0 / dds_publish_hz_ : 0.0;
        }

        /** Requires a launch-injected xacro planning URDF (planning_urdf_path). */
        std::string resolvePlanningUrdfPath() const;
        /** Split isolcpus: RT / MPC / DDP+viz, and pin the DDP ThreadPool before the MPC is constructed. */
        void prepareThreadIsolation();
        [[nodiscard]] bool shouldPublishDds(const rclcpp::Time& time);

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        robot_controller_core::CtrlInterfaces& ctrl_interfaces_;
        std::vector<std::string> joint_names_;

        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;
        std::unique_ptr<OCS2PinocchioVisualizer> visualizer_;

        // Configuration
        std::string robot_name_;
        std::string planning_urdf_variant_; // must be "xacro"
        std::string planning_urdf_path_; // xacro-generated URDF cache path

        // Visualization worker (MPC-rate, off the RT update path)
        std::atomic_bool collision_active_{false};
        /** RT-only staging for pred samples; folded into the snapshot in requestVisualizationUpdate(). */
        bool pending_viz_has_pred_{false};
        double pending_viz_pred_time_{0.0};
        vector_t pending_viz_pred_state_;

        RtCycleTiming rt_timing_;
        int thread_priority_{0};
        std::vector<int> cpu_affinity_;
        double dds_publish_hz_{50.0};
        double dds_publish_period_sec_{0.02};

    private:
        void visualizationThreadLoop();

        std::thread visualization_thread_;
        std::atomic_bool visualization_running_{false};
        std::atomic_bool visualization_update_requested_{false};
        int visualization_thread_sleep_ms_{2};
        double visualization_period_sec_{0.05};
        rclcpp::Time last_visualization_request_time_{0, 0, RCL_ROS_TIME};
        /** Published via atomic_store/load only (RT -> viz handoff, no mutex). */
        std::shared_ptr<VisualizationSnapshot> visualization_snapshot_;

        std::vector<int> rt_loop_cpus_;
        std::vector<int> mpc_thread_cpus_;
        std::vector<int> worker_thread_cpus_;
        bool rt_loop_scheduling_applied_{false};
        mutable std::atomic<int64_t> mpc_period_us_{10000};
        rclcpp::Time last_dds_publish_time_{0, 0, RCL_ROS_TIME};
    };
} // namespace ocs2::mpc_core
