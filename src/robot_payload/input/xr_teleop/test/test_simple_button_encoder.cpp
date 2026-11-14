#include <gtest/gtest.h>

#include <vector>

#include "xr_teleop/simple_button_encoder.hpp"

using xr_teleop::SimpleButtonEncoder;
using xr_teleop::TeleState;
namespace ev = xr_teleop::event;

namespace
{
    constexpr int64_t kFrameNs = 11'111'111;  // ~90 Hz

    struct Keys
    {
        bool a = false, x = false, y = false, b = false, rstick = false, lgrip = false, ltrig = false;
    };

    class Runner
    {
    public:
        int tick(const Keys& k)
        {
            TeleState s;
            s.right_a = k.a;
            s.left_x = k.x;
            s.left_y = k.y;
            s.right_b = k.b;
            s.right_stick_click = k.rstick;
            s.left_grip = k.lgrip;
            s.left_trigger = k.ltrig;
            t_ += kFrameNs;
            return enc.update(s, t_).event;
        }
        /// Hold keys for the given duration, return the non-zero events seen.
        std::vector<int> hold(const Keys& k, int64_t duration_ns)
        {
            std::vector<int> seen;
            for (int64_t elapsed = 0; elapsed < duration_ns; elapsed += kFrameNs)
            {
                if (int e = tick(k)) { seen.push_back(e); }
            }
            return seen;
        }
        SimpleButtonEncoder enc;

    private:
        int64_t t_ = 1'000'000'000;
    };
} // namespace

TEST(SimpleButtonEncoder, aAndXAreFsmKeysOnRisingEdge)
{
    Runner r;
    EXPECT_EQ(r.tick({.a = true}), ev::kRightA);
    EXPECT_EQ(r.tick({.a = true}), ev::kNone);
    EXPECT_EQ(r.tick({}), ev::kNone);
    EXPECT_EQ(r.tick({.x = true}), ev::kLeftX);
}

TEST(SimpleButtonEncoder, bAloneTogglesRecordingOnRelease)
{
    Runner r;
    EXPECT_EQ(r.tick({.b = true}), ev::kNone);
    EXPECT_EQ(r.tick({}), ev::kLeftGripRightA);  // 50 start
    EXPECT_TRUE(r.enc.recording());
    EXPECT_EQ(r.tick({.b = true}), ev::kNone);
    EXPECT_EQ(r.tick({}), ev::kLeftGripRightB);  // 51 stop
    EXPECT_FALSE(r.enc.recording());
}

TEST(SimpleButtonEncoder, yPlusBIsScaleChordAndDoesNotToggleRecording)
{
    Runner r;
    EXPECT_EQ(r.tick({.b = true}), ev::kNone);
    EXPECT_EQ(r.tick({.y = true, .b = true}), ev::kLeftYRightB);
    EXPECT_EQ(r.tick({.y = true}), ev::kNone);  // B released: chorded, no 50
    EXPECT_EQ(r.tick({}), ev::kNone);           // Y released: no delete
    EXPECT_FALSE(r.enc.recording());
}

TEST(SimpleButtonEncoder, yPlusAEntersMoveJ)
{
    Runner r;
    EXPECT_EQ(r.tick({.y = true}), ev::kNone);
    EXPECT_EQ(r.tick({.a = true, .y = true}), ev::kLeftYRightA);
    EXPECT_EQ(r.hold({.a = true, .y = true}, SimpleButtonEncoder::kLongPressNs * 2), std::vector<int>{});
}

TEST(SimpleButtonEncoder, longPressYDeletesOnceShortPressDoesNothing)
{
    Runner r;
    EXPECT_EQ(r.hold({.y = true}, SimpleButtonEncoder::kLongPressNs / 2), std::vector<int>{});
    EXPECT_EQ(r.tick({}), ev::kNone);
    EXPECT_EQ(r.hold({.y = true}, SimpleButtonEncoder::kLongPressNs * 2),
              std::vector<int>{ev::kLeftGripLeftY});  // 53 exactly once
}

TEST(SimpleButtonEncoder, rightStickClickMarksIntervention)
{
    Runner r;
    EXPECT_EQ(r.tick({.rstick = true}), ev::kRightGripRightStick);  // 52
    EXPECT_EQ(r.tick({.rstick = true}), ev::kNone);
}

TEST(SimpleButtonEncoder, gripsAndTriggersProduceNoEventsButAnalogPassesThrough)
{
    Runner r;
    EXPECT_EQ(r.hold({.lgrip = true, .ltrig = true}, 200'000'000), std::vector<int>{});
    TeleState s;
    s.left_grip = true;
    s.left_trigger_pull = 0.7;
    const auto out = r.enc.update(s, 5'000'000'000);
    EXPECT_EQ(out.event, ev::kNone);
    EXPECT_DOUBLE_EQ(out.thumbstick[2], 1.0);  // grip state for the clutch
    EXPECT_DOUBLE_EQ(out.left_trigger_pull, 0.7);
}
