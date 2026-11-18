// What the reference manager lets in as a target pose: finite numbers and a usable quaternion.
#include <gtest/gtest.h>

#include <limits>

#include <mpc_core/reference/TargetPose.h>

using ocs2::mpc_core::targetPoseUsable;

namespace
{
    Eigen::VectorXd validPose()
    {
        Eigen::VectorXd p(7);
        p << 0.3, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0;
        return p;
    }
}  // namespace

TEST(TargetPose, aNormalPoseIsUsable)
{
    EXPECT_TRUE(targetPoseUsable(validPose()));
}

TEST(TargetPose, nonFiniteNumbersAreNotUsable)
{
    for (int i = 0; i < 7; ++i)
    {
        Eigen::VectorXd nan_pose = validPose();
        nan_pose(i) = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(targetPoseUsable(nan_pose)) << "NaN at " << i;
        Eigen::VectorXd inf_pose = validPose();
        inf_pose(i) = std::numeric_limits<double>::infinity();
        EXPECT_FALSE(targetPoseUsable(inf_pose)) << "inf at " << i;
    }
}

TEST(TargetPose, aZeroQuaternionIsNotUsable)
{
    Eigen::VectorXd p = validPose();
    p.segment(3, 4).setZero();
    EXPECT_FALSE(targetPoseUsable(p));
}

TEST(TargetPose, aTooShortVectorIsNotUsable)
{
    EXPECT_FALSE(targetPoseUsable(Eigen::VectorXd::Zero(6)));
}
