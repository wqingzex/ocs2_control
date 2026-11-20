// A target given relative to the base is moved to the world frame with the base pose of the state; the derivative the
// end effector constraint subtracts from its linearization must be the real one.
#include <gtest/gtest.h>

#include <cmath>

#include "whole_body_mpc/constraint/WholeBodyEndEffectorConstraint.h"

namespace
{
using ocs2::scalar_t;
using ocs2::whole_body_mpc::bodyFrameTargetPosition;
using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
}  // namespace

TEST(BodyFrameTarget, aBaseTurnedQuarterAroundSeesTheTargetRotatedAndShifted)
{
    const auto target = bodyFrameTargetPosition(1.0, 2.0, M_PI / 2.0, Vector3(1.0, 0.0, 0.5));

    EXPECT_NEAR(target.world.x(), 1.0, 1e-12);
    EXPECT_NEAR(target.world.y(), 3.0, 1e-12);
    EXPECT_NEAR(target.world.z(), 0.5, 1e-12);
}

TEST(BodyFrameTarget, withTheBaseAtTheOriginTheTargetIsUnchanged)
{
    const Vector3 target(0.4, -0.2, 0.9);
    const auto world = bodyFrameTargetPosition(0.0, 0.0, 0.0, target).world;

    EXPECT_NEAR((world - target).norm(), 0.0, 1e-12);
}

TEST(BodyFrameTarget, theBaseJacobianMatchesFiniteDifferences)
{
    const scalar_t x = 0.3, y = -0.7, yaw = 0.9;
    const Vector3 target(0.5, 0.2, 0.8);
    const auto base = bodyFrameTargetPosition(x, y, yaw, target);
    constexpr scalar_t kStep = 1e-6;

    const Vector3 dx = (bodyFrameTargetPosition(x + kStep, y, yaw, target).world -
                        bodyFrameTargetPosition(x - kStep, y, yaw, target).world) / (2.0 * kStep);
    const Vector3 dy = (bodyFrameTargetPosition(x, y + kStep, yaw, target).world -
                        bodyFrameTargetPosition(x, y - kStep, yaw, target).world) / (2.0 * kStep);
    const Vector3 dyaw = (bodyFrameTargetPosition(x, y, yaw + kStep, target).world -
                          bodyFrameTargetPosition(x, y, yaw - kStep, target).world) / (2.0 * kStep);

    EXPECT_NEAR((base.baseJacobian.col(0) - dx).norm(), 0.0, 1e-6);
    EXPECT_NEAR((base.baseJacobian.col(1) - dy).norm(), 0.0, 1e-6);
    EXPECT_NEAR((base.baseJacobian.col(2) - dyaw).norm(), 0.0, 1e-6);
}
