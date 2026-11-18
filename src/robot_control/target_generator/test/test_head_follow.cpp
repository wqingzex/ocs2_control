#include <gtest/gtest.h>

#include <cmath>

#include "target_generator/head_follow.h"

using namespace target_generator;

namespace
{
    const Eigen::Quaterniond kIdentity = Eigen::Quaterniond::Identity();

    HeadFollowBase baseAt(const Eigen::Vector3d& xr, const Eigen::Vector3d& robot)
    {
        return {xr, kIdentity, robot, kIdentity};
    }

    Eigen::Quaterniond yaw(double radians)
    {
        return Eigen::Quaterniond(Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()));
    }
}

TEST(HeadFollow, noMovementKeepsTheRobotHeadWhereItWas)
{
    const auto base = baseAt({0.3, -0.1, 1.2}, {0.0, 0.0, 1.5});
    const auto target = headTarget(base, base.xr_position, base.xr_orientation);
    EXPECT_TRUE(target.position.isApprox(base.robot_position, 1e-12));
    EXPECT_NEAR(target.orientation.angularDistance(base.robot_orientation), 0.0, 1e-12);
}

TEST(HeadFollow, theHeadsMovementIsAddedToTheRobotHeadInTheSameAxes)
{
    const auto base = baseAt({1.0, 2.0, 3.0}, {0.0, 0.0, 1.5});
    const auto target = headTarget(base, Eigen::Vector3d(1.02, 1.99, 3.05), kIdentity);
    EXPECT_NEAR(target.position.x(), 0.02, 1e-12);
    EXPECT_NEAR(target.position.y(), -0.01, 1e-12);
    EXPECT_NEAR(target.position.z(), 1.55, 1e-12);
}

TEST(HeadFollow, theHeadsTurnIsAppliedOnTopOfTheRobotHeadOrientation)
{
    HeadFollowBase base = baseAt(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 1.5));
    base.robot_orientation = yaw(0.3);
    const auto target = headTarget(base, Eigen::Vector3d::Zero(), yaw(0.5));
    EXPECT_NEAR(target.orientation.angularDistance(yaw(0.8)), 0.0, 1e-9);
}

TEST(HeadFollow, aTurnOfTheHeadBeforeFollowingStartedDoesNotCount)
{
    HeadFollowBase base = baseAt(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 1.5));
    base.xr_orientation = yaw(1.0);  // the headset was already turned when following started
    const auto target = headTarget(base, Eigen::Vector3d::Zero(), yaw(1.0));
    EXPECT_NEAR(target.orientation.angularDistance(base.robot_orientation), 0.0, 1e-9);
}

TEST(HeadFollow, theResultIsANormalizedQuaternion)
{
    HeadFollowBase base = baseAt(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    base.robot_orientation = Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0);  // not unit on purpose
    const auto target = headTarget(base, Eigen::Vector3d::Zero(), kIdentity);
    EXPECT_NEAR(target.orientation.norm(), 1.0, 1e-12);
}

TEST(HeadFollow, aPoseIsUsableOnlyWhenItIsFiniteWithANonZeroQuaternion)
{
    EXPECT_TRUE(headPoseUsable(Eigen::Vector3d(0, 0, 1), kIdentity));
    EXPECT_FALSE(headPoseUsable(Eigen::Vector3d(std::nan(""), 0, 1), kIdentity));
    EXPECT_FALSE(headPoseUsable(Eigen::Vector3d(0, 0, 1), Eigen::Quaterniond(0, 0, 0, 0)));
    EXPECT_FALSE(headPoseUsable(Eigen::Vector3d(0, 0, 1), Eigen::Quaterniond(std::nan(""), 0, 0, 0)));
}
