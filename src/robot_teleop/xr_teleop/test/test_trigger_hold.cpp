// A controller that drops out of tracking must not open the gripper it was closing.
#include <gtest/gtest.h>

#include <xr_teleop/trigger_hold.hpp>

using xr_teleop::triggerOrHeld;

TEST(TriggerHold, aPresentControllerPublishesItsValueAndRemembersIt)
{
    double held = 0.0;
    EXPECT_DOUBLE_EQ(triggerOrHeld(true, 0.8, held), 0.8);
    EXPECT_DOUBLE_EQ(held, 0.8);
    EXPECT_DOUBLE_EQ(triggerOrHeld(true, 0.0, held), 0.0) << "a released trigger is a real value";
}

TEST(TriggerHold, anAbsentControllerKeepsTheLastValueInsteadOfZero)
{
    double held = 0.0;
    triggerOrHeld(true, 0.8, held);
    EXPECT_DOUBLE_EQ(triggerOrHeld(false, 0.0, held), 0.8);
    EXPECT_DOUBLE_EQ(triggerOrHeld(false, 0.0, held), 0.8);
    EXPECT_DOUBLE_EQ(triggerOrHeld(true, 0.3, held), 0.3) << "back in tracking: the live value again";
}

TEST(TriggerHold, aControllerNeverSeenPublishesZero)
{
    double held = 0.0;
    EXPECT_DOUBLE_EQ(triggerOrHeld(false, 0.0, held), 0.0);
}
