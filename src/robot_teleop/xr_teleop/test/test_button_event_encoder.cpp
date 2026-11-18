#include <gtest/gtest.h>

#include <vector>

#include "xr_teleop/button_event_encoder.hpp"
#include "golden_csv.hpp"

using xr_teleop::ButtonEventEncoder;
using xr_teleop::ControllerSample;
using xr_teleop::TeleState;
namespace ev = xr_teleop::event;

namespace
{
    // Column layout of xr_buttons_golden.csv (see the header comment in the file).
    enum Col
    {
        kLClick, kRClick, kX, kY, kA, kB, kLTrig, kRTrig, kLGrip, kRGrip, kLAx, kLAy, kRAx, kRAy,
        kEvent, kThLx, kThLy, kThLz, kThAx, kThAy, kThAz, kTrigL, kTrigR, kGripL, kGripR, kCols
    };

    TeleState stateFromRow(const std::vector<double>& r)
    {
        ControllerSample left, right;
        left.axis_click = r[kLClick] != 0;
        right.axis_click = r[kRClick] != 0;
        left.primary_button = r[kX] != 0;
        left.secondary_button = r[kY] != 0;
        right.primary_button = r[kA] != 0;
        right.secondary_button = r[kB] != 0;
        left.trigger = r[kLTrig];
        right.trigger = r[kRTrig];
        left.grip = r[kLGrip];
        right.grip = r[kRGrip];
        left.axis_x = r[kLAx];
        left.axis_y = r[kLAy];
        right.axis_x = r[kRAx];
        right.axis_y = r[kRAy];
        return xr_teleop::toTeleState(left, right);
    }

    /// Only the inputs that matter for a hand-written sequence.
    struct Keys
    {
        bool a = false, b = false, x = false, y = false;
        bool lstick = false, rstick = false, lgrip = false, rgrip = false;
        double ltrig = 0.0, rtrig = 0.0, lax = 0.0, ray = 0.0;
    };

    int tick(ButtonEventEncoder& enc, const Keys& k)
    {
        ControllerSample l, r;
        r.primary_button = k.a;
        r.secondary_button = k.b;
        l.primary_button = k.x;
        l.secondary_button = k.y;
        l.axis_click = k.lstick;
        r.axis_click = k.rstick;
        l.grip = k.lgrip ? 1.0 : 0.0;
        r.grip = k.rgrip ? 1.0 : 0.0;
        l.trigger = k.ltrig;
        r.trigger = k.rtrig;
        l.axis_x = k.lax;
        r.axis_y = k.ray;
        return enc.update(xr_teleop::toTeleState(l, r)).event;
    }
} // namespace

TEST(ButtonEventEncoder, eventCodesStartAtOneAfterNoEvent)
{
    EXPECT_EQ(ev::kNone, 0);
    EXPECT_EQ(ev::kFsmForward, 1);
    EXPECT_EQ(ev::kFsmBack, 2);
    EXPECT_EQ(ev::kToggleFollow, 3);
    EXPECT_EQ(ev::kToggleChassisDrive, 4);
}

TEST(ButtonEventEncoder, rightAIsFsmForwardOnPressOnly)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.a = true}), ev::kFsmForward);
    EXPECT_EQ(tick(enc, {.a = true}), ev::kNone);  // held: no repeat
    EXPECT_EQ(tick(enc, {}), ev::kNone);           // release: nothing
    EXPECT_EQ(tick(enc, {.a = true}), ev::kFsmForward);
}

TEST(ButtonEventEncoder, leftXIsFsmBackOnPressOnly)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.x = true}), ev::kFsmBack);
    EXPECT_EQ(tick(enc, {.x = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kNone);
}

TEST(ButtonEventEncoder, rightStickClickTogglesFollowOnPressNotOnRelease)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.rstick = true}), ev::kToggleFollow);
    EXPECT_EQ(tick(enc, {.rstick = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kNone);
}

TEST(ButtonEventEncoder, leftStickClickTogglesChassisDriveOnPressNotOnRelease)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.lstick = true}), ev::kToggleChassisDrive);
    EXPECT_EQ(tick(enc, {.lstick = true}), ev::kNone);  // held: no repeat
    EXPECT_EQ(tick(enc, {}), ev::kNone);                // release: nothing
    EXPECT_EQ(tick(enc, {.lstick = true}), ev::kToggleChassisDrive);
}

TEST(ButtonEventEncoder, theLeftStickClickIsAKeyOnItsOwnNotAChord)
{
    // Pressing both sticks does not mean a third thing: one event per frame, the right stick first.
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.lstick = true, .rstick = true}), ev::kToggleFollow);
    EXPECT_EQ(tick(enc, {.lstick = true, .rstick = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kNone);
    // A held grip or trigger does not change it either.
    EXPECT_EQ(tick(enc, {.lstick = true, .lgrip = true, .rtrig = 1.0}), ev::kToggleChassisDrive);
}

TEST(ButtonEventEncoder, everyOtherInputProducesNoEvent)
{
    ButtonEventEncoder enc;
    const std::vector<Keys> others = {
        {.b = true}, {.y = true}, {.lgrip = true}, {.rgrip = true},
        {.ltrig = 1.0}, {.rtrig = 1.0}, {.lax = 1.0}, {.ray = -1.0},
        {.b = true, .y = true, .lgrip = true, .rgrip = true, .ltrig = 1.0, .rtrig = 1.0},
    };
    for (const auto& keys : others)
    {
        EXPECT_EQ(tick(enc, keys), ev::kNone);
        EXPECT_EQ(tick(enc, {}), ev::kNone);  // releasing them is silent as well
    }
}

TEST(ButtonEventEncoder, commandKeysStillFireWhileOtherInputsAreHeld)
{
    // No chords any more: a held grip or trigger never changes what A, X or the right stick mean.
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.lgrip = true, .rtrig = 1.0}), ev::kNone);
    EXPECT_EQ(tick(enc, {.a = true, .lgrip = true, .rtrig = 1.0}), ev::kFsmForward);
    EXPECT_EQ(tick(enc, {.lgrip = true, .rtrig = 1.0}), ev::kNone);
    EXPECT_EQ(tick(enc, {.rstick = true, .rgrip = true}), ev::kToggleFollow);
}

TEST(ButtonEventEncoder, oneEventPerFrameWhenKeysArePressedTogether)
{
    // One Int32 per frame: A wins over X, X over the right stick; the others are not replayed later.
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.a = true, .x = true, .rstick = true}), ev::kFsmForward);
    EXPECT_EQ(tick(enc, {.a = true, .x = true, .rstick = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kNone);
    EXPECT_EQ(tick(enc, {.x = true, .rstick = true}), ev::kFsmBack);
}

TEST(ButtonEventEncoder, resetForgetsHeldKeys)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.a = true}), ev::kFsmForward);
    enc.reset();
    EXPECT_EQ(tick(enc, {.a = true}), ev::kFsmForward);  // still held, but seen as a new press after reset
}

TEST(ButtonEventEncoder, thumbstickYIsFlippedAndDeadzoned)
{
    ButtonEventEncoder enc;
    ControllerSample l, r;
    l.axis_y = 0.5;   // SDK: forward positive
    l.axis_x = 0.05;  // inside deadzone
    const auto out = enc.update(xr_teleop::toTeleState(l, r));
    EXPECT_DOUBLE_EQ(out.thumbstick[1], -0.5);
    EXPECT_DOUBLE_EQ(out.thumbstick[0], 0.0);
}

TEST(ButtonEventEncoderGolden, analogOutputsMatchPythonReferenceFrameByFrame)
{
    // The trace was recorded from the former Python node; its event column belongs to the old key
    // scheme and is not compared any more. Thumbsticks, triggers and grips are unchanged.
    const auto rows = xr_teleop::test::readCsv(xr_teleop::test::dataPath("xr_buttons_golden.csv"));
    ASSERT_GT(rows.size(), 1000u);

    ButtonEventEncoder enc;
    for (size_t i = 0; i < rows.size(); ++i)
    {
        const auto& r = rows[i];
        ASSERT_EQ(r.size(), static_cast<size_t>(kCols)) << "row " << i;
        const auto out = enc.update(stateFromRow(r));
        const double th[6] = {r[kThLx], r[kThLy], r[kThLz], r[kThAx], r[kThAy], r[kThAz]};
        for (int k = 0; k < 6; ++k)
        {
            ASSERT_NEAR(out.thumbstick[k], th[k], 1e-9) << "frame " << i << " axis " << k;
        }
        ASSERT_NEAR(out.left_trigger_pull, r[kTrigL], 1e-9) << "frame " << i;
        ASSERT_NEAR(out.right_trigger_pull, r[kTrigR], 1e-9) << "frame " << i;
        ASSERT_NEAR(out.left_grip_value, r[kGripL], 1e-9) << "frame " << i;
        ASSERT_NEAR(out.right_grip_value, r[kGripR], 1e-9) << "frame " << i;
    }
}
