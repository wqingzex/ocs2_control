#include <gtest/gtest.h>

#include <map>

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

    /// Helper for hand-written sequences: only the buttons that matter.
    struct Buttons
    {
        bool x = false, y = false, a = false, b = false;
        bool lgrip = false, rgrip = false, lstick = false, rstick = false;
        double ltrig = 0.0;
    };

    int tick(ButtonEventEncoder& enc, const Buttons& in)
    {
        ControllerSample l, r;
        l.primary_button = in.x;
        l.secondary_button = in.y;
        r.primary_button = in.a;
        r.secondary_button = in.b;
        l.grip = in.lgrip ? 1.0 : 0.0;
        r.grip = in.rgrip ? 1.0 : 0.0;
        l.axis_click = in.lstick;
        r.axis_click = in.rstick;
        l.trigger = in.ltrig;
        return enc.update(xr_teleop::toTeleState(l, r)).event;
    }
} // namespace

TEST(ButtonEventEncoderGolden, matchesPythonReferenceFrameByFrame)
{
    const auto rows = xr_teleop::test::readCsv(xr_teleop::test::dataPath("xr_buttons_golden.csv"));
    ASSERT_GT(rows.size(), 1000u);

    ButtonEventEncoder enc;
    std::map<int, int> coverage;
    for (size_t i = 0; i < rows.size(); ++i)
    {
        const auto& r = rows[i];
        ASSERT_EQ(r.size(), static_cast<size_t>(kCols)) << "row " << i;
        const auto out = enc.update(stateFromRow(r));
        coverage[out.event]++;
        ASSERT_EQ(out.event, static_cast<int>(r[kEvent])) << "frame " << i;
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
    // The trace must exercise every legacy event code.
    for (int code : {0, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23, 24, 25, 26, 27, 28,
                     29, 50, 51, 52, 53, 54})
    {
        EXPECT_GT(coverage[code], 0) << "event " << code << " not covered by the golden trace";
    }
}

TEST(ButtonEventEncoder, leftYRightAChordConsumesYAndDoesNotRepeat)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.y = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {.y = true, .a = true}), ev::kLeftYRightA);
    EXPECT_EQ(tick(enc, {.y = true, .a = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kNone);  // no standalone Y (3) on release
    EXPECT_EQ(tick(enc, {.a = true}), ev::kRightA);
}

TEST(ButtonEventEncoder, standaloneYFiresOnReleaseOnly)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.y = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {.y = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kLeftY);
}

TEST(ButtonEventEncoder, gripReleasedAfterChordDoesNotFireStandaloneGrip)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.lgrip = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {.lgrip = true, .ltrig = 1.0}), ev::kLeftGripTrigger);
    EXPECT_EQ(tick(enc, {}), ev::kNone);
    // A plain press/release of the grip afterwards is a standalone grip event again.
    EXPECT_EQ(tick(enc, {.lgrip = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {}), ev::kLeftGrip);
}

TEST(ButtonEventEncoder, yBChordWithBothGripsHasPriority)
{
    ButtonEventEncoder enc;
    EXPECT_EQ(tick(enc, {.lgrip = true, .rgrip = true}), ev::kNone);
    EXPECT_EQ(tick(enc, {.y = true, .lgrip = true, .rgrip = true}), ev::kLeftGripLeftY);
    EXPECT_EQ(tick(enc, {.y = true, .b = true, .lgrip = true, .rgrip = true}), ev::kBothGripsYB);
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
