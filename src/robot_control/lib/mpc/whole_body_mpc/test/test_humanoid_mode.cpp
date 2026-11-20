// The humanoid mode word: the old body mode value 6 (head tracking as a body mode) is gone, head tracking is the head mode.
#include <gtest/gtest.h>

#include <stdexcept>

#include "whole_body_mpc/mode/HumanoidModeSchedule.h"

namespace
{
using ocs2::whole_body_mpc::BodyConstraintMode;
using ocs2::whole_body_mpc::HeadMode;
using ocs2::whole_body_mpc::HumanoidMode;
}  // namespace

TEST(HumanoidMode, theRemovedBodyModeValueSixIsRejected)
{
    EXPECT_THROW(HumanoidMode::decode(6), std::invalid_argument);
    EXPECT_THROW(HumanoidMode::decode(7), std::invalid_argument);
    EXPECT_THROW(HumanoidMode::decode(6 | 0x08 | 0x10), std::invalid_argument);
}

TEST(HumanoidMode, headTrackingIsAHeadModeAndSurvivesEncodeAndDecode)
{
    HumanoidMode mode;
    mode.bodyMode = BodyConstraintMode::FREE;
    mode.headMode = HeadMode::TRACKING;

    const auto decoded = HumanoidMode::decode(mode.encode());

    EXPECT_EQ(decoded.headMode, HeadMode::TRACKING);
    EXPECT_EQ(decoded.bodyMode, BodyConstraintMode::FREE);
}

TEST(HumanoidMode, everyRemainingBodyModeRoundTrips)
{
    for (auto body : {BodyConstraintMode::FREE, BodyConstraintMode::RELATIVE, BodyConstraintMode::TRACKING,
                      BodyConstraintMode::LOCKED, BodyConstraintMode::CUSTOM_LOCKED})
    {
        HumanoidMode mode;
        mode.bodyMode = body;
        EXPECT_EQ(HumanoidMode::decode(mode.encode()).bodyMode, body);
    }
}

TEST(HumanoidMode, theHeadCoupledBodyModeStillMeansHeadForward)
{
    HumanoidMode mode;
    mode.bodyMode = BodyConstraintMode::HEAD_COUPLED;

    const auto decoded = HumanoidMode::decode(mode.encode());

    EXPECT_EQ(decoded.headMode, HeadMode::FORWARD);
    EXPECT_EQ(decoded.effectiveBodyMode(), BodyConstraintMode::HEAD_COUPLED);
}
