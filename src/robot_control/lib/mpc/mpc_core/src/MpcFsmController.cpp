#include <mpc_core/MpcFsmController.h>

#include <algorithm>

#include <Eigen/Dense>
#include <mpc_core/mpc/MpcExecutionParams.hpp>

namespace ocs2::mpc_core
{
    // ------------------------------------------------------------------------------------------------------------
    // FsmControllerBase hooks
    // ------------------------------------------------------------------------------------------------------------

    void MpcFsmController::onCycleBegin(const rclcpp::Time& time)
    {
        mpc_->beginRtCycle();
        mpc_->applyRtLoopSchedulingOnce();
        {
            auto obs_scope = mpc_->rtScopeObs();
            mpc_->updateObservation(time);
        }
        FsmController::onCycleBegin(time);
    }

    void MpcFsmController::onCycleEnd(const rclcpp::Time& time, const std::string& fsm_at_cycle_start)
    {
        FsmController::onCycleEnd(time, fsm_at_cycle_start);
        {
            auto viz_scope = mpc_->rtScopeViz();
            mpc_->maybeRequestVisualizationUpdate(time);
        }
        mpc_->endRtCycle(current_state_ ? current_state_->state_name_string : fsm_at_cycle_start);
    }

    void MpcFsmController::holdWhileExiting() { mpc_->holdLastSentPositions(); }

    void MpcFsmController::onEnterState(const FSMStateName entered)
    {
        FsmController::onEnterState(entered);
        if (entered == FSMStateName::OCS2)
        {
            mpc_->publishCachedCurrentTargets();
        }
    }

    std::shared_ptr<tf2_ros::Buffer> MpcFsmController::waistTfBuffer()
    {
        if (mpc_->pose_reference_manager_)
        {
            return mpc_->pose_reference_manager_->getTfBuffer();
        }
        return nullptr;
    }

    // ------------------------------------------------------------------------------------------------------------
    // Lifecycle pieces
    // ------------------------------------------------------------------------------------------------------------

    void MpcFsmController::createTask()
    {
        // MPC frequency parameter - default value 0 if not set
        auto_declare<int>("mpc_frequency", 0);
        mpc_ = createMpcComponent();
    }

    const pinocchio::Model* MpcFsmController::planningModel() const { return mpc_ ? mpc_->planningModel() : nullptr; }

    void MpcFsmController::createTaskStates()
    {
        states_.ocs2 = std::make_shared<StateOCS2>(ctrl_interfaces_, get_node(), mpc_);
    }

    // Joint limits and self-collision guard of MOVEJ, from the planning model
    void MpcFsmController::onInitFinished()
    {
        const auto* model = mpc_->planningModel();
        if (!model)
        {
            return;
        }

        const int dim = jointLimitDim();
        if (dim > 0 && dim <= model->nq)
        {
            const Eigen::VectorXd lower_limits = model->lowerPositionLimit.tail(dim);
            const Eigen::VectorXd upper_limits = model->upperPositionLimit.tail(dim);
            states_.movej->setJointLimitChecker(
                [lower_limits, upper_limits, dim](const std::vector<double>& target_pos)
                {
                    std::vector<double> clamped_pos = target_pos;
                    for (size_t i = 0; i < clamped_pos.size() && i < static_cast<size_t>(dim); ++i)
                    {
                        clamped_pos[i] = std::clamp(
                            clamped_pos[i],
                            static_cast<double>(lower_limits(static_cast<Eigen::Index>(i))),
                            static_cast<double>(upper_limits(static_cast<Eigen::Index>(i))));
                    }
                    return clamped_pos;
                });
            RCLCPP_INFO(get_node()->get_logger(),
                        "Joint limit checker initialized from Pinocchio model (controlled joints: %d)", dim);
        }
        else
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Joint limit checker skipped: joint count %d vs model nq %d", dim, model->nq);
        }

        if (mpc_->selfCollisionEnabled())
        {
            states_.movej->setCollisionCheckCallback(
                [mpc = mpc_](double /*threshold*/) { return mpc && mpc->isSelfCollisionActive(); });
            states_.movej->setCollisionEnabled(true);
            states_.movej->setCollisionThreshold(mpc_->selfCollisionMinimumDistance());
            RCLCPP_INFO(get_node()->get_logger(),
                        "MOVEJ self-collision -> HOLD enabled (threshold=%.4f m, sync on command)",
                        mpc_->selfCollisionMinimumDistance());
        }
    }

    void MpcFsmController::startRuntime()
    {
        const auto mpc_params = computeMpcExecutionParams(
            static_cast<double>(ctrl_interfaces_.frequency_),
            get_node()->get_parameter("mpc_frequency").as_int(),
            get_node()->get_logger());
        mpc_->startVisualizationThread(mpc_params.thread_sleep_ms, mpc_params.mpc_period_sec);
    }

    void MpcFsmController::stopRuntime() { mpc_->stopVisualizationThread(); }
} // namespace ocs2::mpc_core
