// The operator's base velocity becomes the desired base input of every solve, and nothing else in the targets moves.
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h"

namespace
{
using ocs2::TargetTrajectories;
using ocs2::scalar_t;
using ocs2::vector_t;
using ocs2::whole_body_mpc::SwitchedHumanoidReferenceManager;

constexpr int kInputDim = 22;  // base 2 + 20 joints
constexpr double kMarker = 7.0;

TargetTrajectories targets()
{
    return TargetTrajectories({0.0, 1.0}, {vector_t::Zero(14), vector_t::Zero(14)},
                              {vector_t::Constant(kInputDim, kMarker), vector_t::Constant(kInputDim, kMarker)});
}

vector_t velocity(double v, double omega)
{
    vector_t velocity(2);
    velocity << v, omega;
    return velocity;
}

const std::vector<vector_t>& solveWith(SwitchedHumanoidReferenceManager& manager)
{
    manager.setTargetTrajectories(targets());
    manager.preSolverRun(0.0, 1.0, vector_t::Zero(23));
    return manager.getTargetTrajectories().inputTrajectory;
}
}  // namespace

TEST(BaseVelocityReference, isWrittenIntoEverySampleAndTheRestIsKept)
{
    SwitchedHumanoidReferenceManager manager;
    manager.setBaseInputDim(2);
    manager.setBaseVelocityReference(velocity(0.3, -0.1));

    const auto& inputs = solveWith(manager);

    ASSERT_EQ(inputs.size(), 2u);
    for (const auto& input : inputs)
    {
        EXPECT_DOUBLE_EQ(input(0), 0.3);
        EXPECT_DOUBLE_EQ(input(1), -0.1);
        EXPECT_DOUBLE_EQ(input(2), kMarker) << "an entry after the base changed";
        EXPECT_DOUBLE_EQ(input(5), kMarker) << "the coupling parameters must stay as they were";
    }
}

TEST(BaseVelocityReference, isZeroUntilTheFirstRequestArrives)
{
    SwitchedHumanoidReferenceManager manager;
    manager.setBaseInputDim(2);

    const auto& inputs = solveWith(manager);

    EXPECT_DOUBLE_EQ(inputs.front()(0), 0.0);
    EXPECT_DOUBLE_EQ(inputs.front()(1), 0.0);
}

TEST(BaseVelocityReference, aStaleRequestCountsAsZero)
{
    SwitchedHumanoidReferenceManager manager;
    manager.setBaseInputDim(2);
    manager.setBaseVelocityReferenceTimeout(0.02);
    manager.setBaseVelocityReference(velocity(0.3, 0.2));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    const auto& inputs = solveWith(manager);

    EXPECT_DOUBLE_EQ(inputs.front()(0), 0.0);
    EXPECT_DOUBLE_EQ(inputs.front()(1), 0.0);
    EXPECT_DOUBLE_EQ(inputs.front()(2), kMarker);
}

TEST(BaseVelocityReference, aFixedBaseLeavesTheTargetsAlone)
{
    SwitchedHumanoidReferenceManager manager;  // base input dim stays 0
    manager.setBaseVelocityReference(velocity(0.3, 0.2));

    const auto& inputs = solveWith(manager);

    EXPECT_DOUBLE_EQ(inputs.front()(0), kMarker);
    EXPECT_DOUBLE_EQ(inputs.front()(1), kMarker);
}
