#include <gtest/gtest.h>

#include <cmath>

#include "teleop_command/xr_alignment.h"

using teleop_command::headingYaw;
using teleop_command::motionFrame;

namespace
{
    Eigen::Quaterniond yawPitch(double yaw, double pitch)
    {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    }

    /// Displacement as XrInputHandler applies it: frame^-1 * world delta.
    Eigen::Vector3d toArm(const Eigen::Quaterniond& frame, const Eigen::Vector3d& world_delta)
    {
        return frame.inverse() * world_delta;
    }
} // namespace

TEST(XrAlignment, legacyFrameIsTheControllerOrientation)
{
    const Eigen::Quaterniond controller = yawPitch(0.3, 0.4);
    EXPECT_TRUE(motionFrame(false, yawPitch(1.0, 0.0), controller).isApprox(controller));
}

TEST(XrAlignment, operatorForwardMapsToRobotForwardWhateverTheRoomHeading)
{
    // The operator faces +90° in the XR world (Pico room origin rotated); moving the hand
    // "forward" for the operator is +Y in the XR world.
    const Eigen::Quaterniond head = yawPitch(M_PI / 2, 0.2);  // head slightly pitched down
    const Eigen::Quaterniond frame = motionFrame(true, head, Eigen::Quaterniond::Identity());
    const Eigen::Vector3d arm = toArm(frame, Eigen::Vector3d(0.0, 0.1, 0.0));
    EXPECT_NEAR(arm.x(), 0.1, 1e-12);
    EXPECT_NEAR(arm.y(), 0.0, 1e-12);
    EXPECT_NEAR(arm.z(), 0.0, 1e-12);
}

TEST(XrAlignment, controllerTiltNoLongerTiltsTheMotion)
{
    // Controller held pitched 40° at capture: legacy maps a horizontal push to a tilted arm motion,
    // head-yaw alignment keeps it horizontal.
    const Eigen::Quaterniond controller = yawPitch(0.0, 0.7);
    const Eigen::Vector3d push(0.1, 0.0, 0.0);
    const Eigen::Vector3d legacy = toArm(motionFrame(false, Eigen::Quaterniond::Identity(), controller), push);
    const Eigen::Vector3d aligned = toArm(motionFrame(true, Eigen::Quaterniond::Identity(), controller), push);
    EXPECT_GT(std::abs(legacy.z()), 0.05);
    EXPECT_NEAR(aligned.z(), 0.0, 1e-12);
    EXPECT_NEAR(aligned.x(), 0.1, 1e-12);
}

TEST(XrAlignment, verticalUpIsPreservedForAnyHeading)
{
    const Eigen::Quaterniond frame = motionFrame(true, yawPitch(-2.0, 0.0), Eigen::Quaterniond::Identity());
    const Eigen::Vector3d arm = toArm(frame, Eigen::Vector3d(0.0, 0.0, 0.05));
    EXPECT_NEAR(arm.z(), 0.05, 1e-12);
}

TEST(XrAlignment, headLookingStraightDownFallsBackToControllerHeading)
{
    double yaw = 0.0;
    EXPECT_FALSE(headingYaw(yawPitch(0.0, M_PI / 2), yaw));  // +X axis points straight down
    const Eigen::Quaterniond frame = motionFrame(true, yawPitch(0.0, M_PI / 2), yawPitch(0.5, 0.3));
    ASSERT_TRUE(headingYaw(frame, yaw));
    EXPECT_NEAR(yaw, 0.5, 1e-9);
}
