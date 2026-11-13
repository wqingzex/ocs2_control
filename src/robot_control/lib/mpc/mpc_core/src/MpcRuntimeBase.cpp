#include "mpc_core/MpcRuntimeBase.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>

#include <mpc_core/rt/ThreadScheduling.h>
#include <ocs2_core/thread_support/ThreadPool.h>

namespace ocs2::mpc_core
{
    bool MpcRuntimeBase::initialPolicyReceived() const
    {
        return mpc_mrt_interface_ && mpc_mrt_interface_->initialPolicyReceived();
    }

    void MpcRuntimeBase::holdLastSentPositions() const
    {
        for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(i, ctrl_interfaces_.last_sent_joint_positions_[i]);
        }
    }

    void MpcRuntimeBase::publishCachedCurrentTargets() const
    {
        if (pose_reference_manager_)
        {
            pose_reference_manager_->publishCurrentTargetsFromCache();
        }
    }

    void MpcRuntimeBase::setMpcPeriodSec(double period_sec)
    {
        if (period_sec <= 0.0)
        {
            return;
        }
        mpc_period_us_.store(static_cast<int64_t>(period_sec * 1e6 + 0.5), std::memory_order_relaxed);
    }

    void MpcRuntimeBase::advanceMpc() const
    {
        const auto t0 = std::chrono::steady_clock::now();
        mpc_mrt_interface_->advanceMpc();
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
        const auto period_us = mpc_period_us_.load(std::memory_order_relaxed);
        if (period_us > 0 && us > period_us)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[CTL-001] MPC solve %ld us exceeds period %ld us (%.1f Hz); tracking will lag",
                                 static_cast<long>(us), static_cast<long>(period_us),
                                 1e6 / static_cast<double>(period_us));
        }
    }

    void MpcRuntimeBase::startVisualizationThread(int thread_sleep_ms, double visualization_period_sec)
    {
        if (visualization_running_.load())
        {
            return;
        }
        visualization_thread_sleep_ms_ = std::max(1, thread_sleep_ms);
        visualization_period_sec_ = std::max(0.001, visualization_period_sec);
        if (visualizer_)
        {
            visualizer_->setPosePublishPeriod(visualization_period_sec_);
        }
        visualization_running_ = true;
        // Do not set visualization_update_requested_ here — caller must call
        // requestVisualizationUpdate() after observation_ is ready (e.g. post-resetMpc),
        // otherwise the first snapshot can be empty/stale.
        visualization_thread_ = std::thread(&MpcRuntimeBase::visualizationThreadLoop, this);
    }

    void MpcRuntimeBase::stopVisualizationThread()
    {
        if (!visualization_running_.exchange(false) && !visualization_thread_.joinable())
        {
            return;
        }
        if (visualization_thread_.joinable())
        {
            visualization_thread_.join();
        }
        collision_active_.store(false, std::memory_order_release);
    }

    void MpcRuntimeBase::requestVisualizationUpdate()
    {
        auto snap = std::make_shared<VisualizationSnapshot>();
        snap->observation = observation_;
        snap->collision_state = buildCollisionState();
        snap->has_pred = pending_viz_has_pred_;
        snap->pred_time = pending_viz_pred_time_;
        snap->pred_state = std::move(pending_viz_pred_state_);
        pending_viz_has_pred_ = false;
        pending_viz_pred_time_ = 0.0;
        {
            auto traj_scope = rt_timing_.scope(&rt_timing_.eval_traj_us);
            if (policy_active_ && mpc_mrt_interface_)
            {
                snap->stateTrajectory = mpc_mrt_interface_->getVisualizationStateTrajectory();
                snap->has_policy_traj = snap->stateTrajectory && !snap->stateTrajectory->empty();
            }
        }
        std::atomic_store_explicit(&visualization_snapshot_, std::move(snap), std::memory_order_release);
        visualization_update_requested_ = true;
    }

    void MpcRuntimeBase::maybeRequestVisualizationUpdate(const rclcpp::Time& time)
    {
        if (!visualization_running_)
        {
            return;
        }
        if (last_visualization_request_time_.nanoseconds() != 0 &&
            (time - last_visualization_request_time_).seconds() < visualization_period_sec_)
        {
            return;
        }
        requestVisualizationUpdate();
        last_visualization_request_time_ = time;
    }

    void MpcRuntimeBase::visualizationThreadLoop()
    {
        applyWorkerThreadScheduling("viz");
        RCLCPP_DEBUG(node_->get_logger(),
                     "Visualization thread started (sleep=%d ms, MPC-rate)",
                     visualization_thread_sleep_ms_);
        while (visualization_running_.load())
        {
            if (visualization_update_requested_.load())
            {
                auto snap = std::atomic_load_explicit(&visualization_snapshot_, std::memory_order_acquire);
                if (snap)
                {
                    runVisualizationOnce(*snap);
                }
                visualization_update_requested_ = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(visualization_thread_sleep_ms_));
        }
        RCLCPP_DEBUG(node_->get_logger(), "Visualization thread stopped");
    }

    void MpcRuntimeBase::clearTrajectoryVisualization() const
    {
        visualizer_->clearTrajectoryHistory();
        pose_reference_manager_->resetTargetStateCache();
    }

    std::string MpcRuntimeBase::resolvePlanningUrdfPath() const
    {
        if (planning_urdf_variant_ != "xacro" || planning_urdf_path_.empty())
        {
            throw std::runtime_error(
                "OCS2 planning URDF must come from xacro (planning_urdf_variant:=xacro, "
                "planning_urdf_path set by launch_helpers). Static urdf/ lookup was removed.");
        }
        if (!std::filesystem::exists(planning_urdf_path_))
        {
            throw std::runtime_error(
                "planning_urdf_path does not exist: " + planning_urdf_path_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Using xacro-generated planning URDF: %s",
                    planning_urdf_path_.c_str());
        return planning_urdf_path_;
    }

    bool MpcRuntimeBase::shouldPublishDds(const rclcpp::Time& time)
    {
        if (dds_publish_period_sec_ <= 0.0)
        {
            return true;
        }
        if (last_dds_publish_time_.nanoseconds() != 0 &&
            (time - last_dds_publish_time_).seconds() < dds_publish_period_sec_)
        {
            return false;
        }
        last_dds_publish_time_ = time;
        return true;
    }

    void MpcRuntimeBase::applyWorkerThreadScheduling(const char* thread_name) const
    {
        const bool is_mpc = thread_name != nullptr && std::string(thread_name) == "MPC";
        if (is_mpc)
        {
            const std::vector<int>& cpus = mpc_thread_cpus_.empty() ? cpu_affinity_ : mpc_thread_cpus_;
            applyThisThreadSchedulingLogged(thread_priority_, cpus, node_->get_logger(), thread_name);
            return;
        }
        // viz: CFS on the MPC core. FIFO 85 on DDP cores would preempt DDP workers (priority 50).
        const std::vector<int>& cpus = mpc_thread_cpus_.empty() ? cpu_affinity_ : mpc_thread_cpus_;
        applyThisThreadSchedulingLogged(0, cpus, node_->get_logger(), thread_name);
    }

    void MpcRuntimeBase::applyRtLoopSchedulingOnce()
    {
        if (rt_loop_scheduling_applied_)
        {
            return;
        }
        rt_loop_scheduling_applied_ = true;
        if (rt_loop_cpus_.empty())
        {
            return;
        }
        applyThisThreadCpuAffinity(rt_loop_cpus_);
        RCLCPP_INFO(node_->get_logger(), "Pinned RT control loop to CPU [%s]",
                    formatCpuList(rt_loop_cpus_).c_str());
    }

    void MpcRuntimeBase::prepareThreadIsolation()
    {
        const auto plan = planIsolatedCpus(cpu_affinity_);
        mpc_thread_cpus_ = plan.mpc;
        rt_loop_cpus_ = plan.rt;
        worker_thread_cpus_ = plan.workers;
        if (plan.isolated)
        {
            ocs2::ThreadPool::setLaunchWorkerCpuAffinity(plan.workers);
            RCLCPP_INFO(node_->get_logger(),
                        "CPU isolation: RT=[%s] MPC=[%s] DDP=[%s] (viz shares MPC core at CFS)",
                        formatCpuList(plan.rt).c_str(),
                        formatCpuList(plan.mpc).c_str(),
                        formatCpuList(plan.workers).c_str());
        }
    }

    void MpcRuntimeBase::publishFsmCommand(int32_t command) const
    {
        if (fsm_command_publisher_)
        {
            std_msgs::msg::Int32 fsm_cmd;
            fsm_cmd.data = command;
            fsm_command_publisher_->publish(fsm_cmd);
        }
    }
} // namespace ocs2::mpc_core
