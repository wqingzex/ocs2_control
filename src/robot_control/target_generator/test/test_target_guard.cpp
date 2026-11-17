#include <gtest/gtest.h>

#include <limits>

#include "target_generator/target_guard.h"

using namespace target_generator;

namespace
{
    const Eigen::Quaterniond kIdentity = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d kArm(0.30, 0.20, 0.40);
}

TEST(TargetGuard, acceptsATargetNearTheArmAndThePreviousTarget)
{
    const TargetGuardLimits limits;
    EXPECT_EQ(checkTarget(limits, kArm + Eigen::Vector3d(0.04, 0, 0), kIdentity, kArm, kArm), TargetVerdict::Accept);
}

TEST(TargetGuard, theFirstTargetHasNoPreviousOneToCompareWith)
{
    const TargetGuardLimits limits;
    EXPECT_EQ(checkTarget(limits, kArm + Eigen::Vector3d(0.2, 0, 0), kIdentity, kArm, std::nullopt),
              TargetVerdict::Accept);
}

TEST(TargetGuard, rejectsATargetFurtherThanTheLimitFromTheArm)
{
    const TargetGuardLimits limits;
    // The 2 m jump seen after a headset drop-out.
    EXPECT_EQ(checkTarget(limits, Eigen::Vector3d(2.3, 1.2, 0.4), kIdentity, kArm, std::nullopt),
              TargetVerdict::TooFarFromArm);
    EXPECT_EQ(checkTarget(limits, kArm + Eigen::Vector3d(0.31, 0, 0), kIdentity, kArm, std::nullopt),
              TargetVerdict::TooFarFromArm);
}

TEST(TargetGuard, rejectsAStepLargerThanTheLimit)
{
    const TargetGuardLimits limits;
    const Eigen::Vector3d previous = kArm + Eigen::Vector3d(0.10, 0, 0);
    EXPECT_EQ(checkTarget(limits, previous + Eigen::Vector3d(0.06, 0, 0), kIdentity, kArm, previous),
              TargetVerdict::StepTooLarge);
    EXPECT_EQ(checkTarget(limits, previous + Eigen::Vector3d(0.05, 0, 0), kIdentity, kArm, previous),
              TargetVerdict::Accept) << "the limit itself is allowed";
}

TEST(TargetGuard, rejectsNonFiniteAndZeroQuaternionEvenWhenSwitchedOff)
{
    TargetGuardLimits limits;
    limits.enabled = false;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(checkTarget(limits, Eigen::Vector3d(nan, 0, 0), kIdentity, kArm, std::nullopt), TargetVerdict::NotFinite);
    EXPECT_EQ(checkTarget(limits, kArm, Eigen::Quaterniond(0, 0, 0, 0), kArm, std::nullopt), TargetVerdict::NotFinite);
}

TEST(TargetGuard, theSwitchTurnsOffTheLimitsOnly)
{
    TargetGuardLimits limits;
    limits.enabled = false;
    EXPECT_EQ(checkTarget(limits, Eigen::Vector3d(2.3, 1.2, 0.4), kIdentity, kArm, kArm), TargetVerdict::Accept);
}
