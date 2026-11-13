#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "juxie_hardware/juxie_protocol.h"

namespace p = juxie_hardware::protocol;
using robot_comm::CanFrame;

namespace
{
    // The rate limiter exactly as it stood in JxHardware::sendMultiMotorCommand before the protocol was split out.
    struct Reference
    {
        int16_t sent;
        int32_t delta;
    };

    Reference referenceStep(int32_t target, int32_t last, int32_t last_delta, int32_t max_position_step_nct,
                            int32_t max_position_accel_nct)
    {
        const int32_t remaining = target - last;

        int32_t desired_delta = remaining;
        if (desired_delta > max_position_step_nct) {
            desired_delta = max_position_step_nct;
        } else if (desired_delta < -max_position_step_nct) {
            desired_delta = -max_position_step_nct;
        }

        int32_t delta = desired_delta;
        if (max_position_accel_nct > 0) {
            const int32_t min_delta = last_delta - max_position_accel_nct;
            const int32_t max_delta = last_delta + max_position_accel_nct;
            delta = std::clamp(desired_delta, min_delta, max_delta);
            delta = std::clamp(delta, -max_position_step_nct, max_position_step_nct);
        }

        if ((remaining >= 0 && delta > remaining) || (remaining <= 0 && delta < remaining)) {
            delta = remaining;
        }

        const int32_t sent = std::clamp(last + delta, static_cast<int32_t>(p::kRawMin),
                                        static_cast<int32_t>(p::kRawMax));
        return {static_cast<int16_t>(sent), sent - last};
    }

    CanFrame feedback12(uint8_t id, int16_t pos, int16_t vel, int16_t current, uint16_t error, uint8_t mode,
                        uint8_t status)
    {
        CanFrame frame;
        frame.id = p::kFeedbackBase + id;
        frame.fd = true;
        frame.len = 12;
        frame.data[0] = static_cast<uint8_t>(pos >> 8);
        frame.data[1] = static_cast<uint8_t>(pos & 0xFF);
        frame.data[2] = static_cast<uint8_t>(vel >> 8);
        frame.data[3] = static_cast<uint8_t>(vel & 0xFF);
        frame.data[4] = static_cast<uint8_t>(current >> 8);
        frame.data[5] = static_cast<uint8_t>(current & 0xFF);
        frame.data[6] = static_cast<uint8_t>(error & 0xFF);
        frame.data[7] = static_cast<uint8_t>(error >> 8);
        frame.data[10] = mode;
        frame.data[11] = status;
        return frame;
    }
} // namespace

// ---- angle <-> raw --------------------------------------------------------------------------------------------

TEST(JuxieProtocol, AnglesMapOntoTheCspRange)
{
    EXPECT_EQ(p::angleToRaw(-180.0), p::kRawMin);
    EXPECT_EQ(p::angleToRaw(180.0), p::kRawMax);
    EXPECT_EQ(p::angleToRaw(0.0), 0);
}

TEST(JuxieProtocol, AnglesOutsideTheRangeAreClamped)
{
    EXPECT_EQ(p::angleToRaw(500.0), p::kRawMax);
    EXPECT_EQ(p::angleToRaw(-500.0), p::kRawMin);
}

TEST(JuxieProtocol, RawToAngleRoundsToAThousandthOfADegree)
{
    EXPECT_DOUBLE_EQ(p::rawToAngle(p::kRawMax), 180.0);
    EXPECT_DOUBLE_EQ(p::rawToAngle(p::kRawMin), -180.0);
    EXPECT_DOUBLE_EQ(p::rawToAngle(0), 0.0);
    const double angle = p::rawToAngle(1000);
    EXPECT_DOUBLE_EQ(angle, std::round(angle * 1000) / 1000);
}

TEST(JuxieProtocol, AngleRoundTripStaysWithinOneCount)
{
    for (double deg = -179.9; deg < 180.0; deg += 7.3)
    {
        EXPECT_NEAR(p::rawToAngle(p::angleToRaw(deg)), deg, 360.0 / 65336.0 * 1.5) << deg;
    }
}

// ---- frames ---------------------------------------------------------------------------------------------------

TEST(JuxieProtocol, SyncFrame)
{
    const CanFrame frame = p::makeSyncFrame();

    EXPECT_EQ(frame.id, 0x80u);
    EXPECT_TRUE(frame.fd);
    EXPECT_FALSE(frame.brs);
    EXPECT_EQ(frame.len, 1);
    EXPECT_EQ(frame.data[0], 0x00);
}

TEST(JuxieProtocol, EnableFrameIsAnSdoWriteToTheMotor)
{
    const CanFrame frame = p::makeEnableFrame(3);

    EXPECT_EQ(frame.id, 0x603u);
    EXPECT_EQ(frame.len, 8);
    const uint8_t expected[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    for (int i = 0; i < 8; ++i)
    {
        EXPECT_EQ(frame.data[i], expected[i]) << i;
    }
}

TEST(JuxieProtocol, ControlByteFollowsTheFeedbackProtocol)
{
    EXPECT_EQ(p::controlByte(7, true), 0xD8);
    EXPECT_EQ(p::controlByte(7, false), 0xD0);
    EXPECT_EQ(p::controlByte(12, true), p::kCtrlWordCspClear);
    EXPECT_EQ(p::controlByte(12, false), p::kCtrlWordCsp);
    EXPECT_EQ(p::controlByte(0, false), p::kCtrlWordCsp); // length not locked yet: current protocol
    EXPECT_EQ(p::kCtrlWordCsp, 0xC6);
    EXPECT_EQ(p::kCtrlWordCspClear, 0xE6);
}

TEST(JuxieProtocol, MultiMotorFrameLayout)
{
    const CanFrame frame = p::makeMultiMotorFrame(0xC6, {0x1234, -2}, {5, 9});

    EXPECT_EQ(frame.id, 0x200u);
    EXPECT_TRUE(frame.fd);
    EXPECT_FALSE(frame.brs);
    EXPECT_EQ(frame.len, 64);
    // motor 0: control byte, position big-endian, four unused bytes
    EXPECT_EQ(frame.data[0], 0xC6);
    EXPECT_EQ(frame.data[1], 0x12);
    EXPECT_EQ(frame.data[2], 0x34);
    for (int i = 3; i < 7; ++i)
    {
        EXPECT_EQ(frame.data[i], 0) << i;
    }
    // motor 1 starts at byte 7; -2 = 0xFFFE
    EXPECT_EQ(frame.data[7], 0xC6);
    EXPECT_EQ(frame.data[8], 0xFF);
    EXPECT_EQ(frame.data[9], 0xFE);
    // the motor ids are at bytes 56 and 57, everything between is zero
    EXPECT_EQ(frame.data[56], 5);
    EXPECT_EQ(frame.data[57], 9);
    for (int i = 14; i < 56; ++i)
    {
        EXPECT_EQ(frame.data[i], 0) << i;
    }
}

TEST(JuxieProtocol, MultiMotorFrameCarriesAtMostEightMotors)
{
    const std::vector<int16_t> raw(12, 1);
    const std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    const CanFrame frame = p::makeMultiMotorFrame(0xC6, raw, ids);

    EXPECT_EQ(frame.data[56 + 7], 8);
    EXPECT_EQ(frame.data[63], 8) << "the 9th id must not be written";
}

// ---- rate limiter ---------------------------------------------------------------------------------------------

TEST(JuxieProtocol, LimiterMatchesTheOriginalDriverCodeOnRandomInputs)
{
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int32_t> position(-32668, 32668);
    std::uniform_int_distribution<int32_t> step(1, 200);
    std::uniform_int_distribution<int32_t> accel(0, 50);
    std::uniform_int_distribution<int32_t> small(-300, 300);

    for (int i = 0; i < 200000; ++i)
    {
        const int32_t target = position(rng);
        const int32_t last = position(rng);
        const int32_t last_delta = small(rng);
        const int32_t max_step = step(rng);
        const int32_t max_accel = accel(rng);

        const p::StepResult actual = p::limitStep(target, last, last_delta, {max_step, max_accel});
        const Reference expected = referenceStep(target, last, last_delta, max_step, max_accel);

        ASSERT_EQ(actual.sent, expected.sent) << i;
        ASSERT_EQ(actual.delta, expected.delta) << i;
    }
}

TEST(JuxieProtocol, LimiterCatchesUpWithoutOvershootingOrExceedingTheLimits)
{
    const p::StepLimits limits{25, 5};
    int32_t last = 0;
    int32_t last_delta = 0;
    const int32_t target = 1000;

    for (int cycle = 0; cycle < 200 && last != target; ++cycle)
    {
        const p::StepResult step = p::limitStep(target, last, last_delta, limits);
        EXPECT_LE(std::abs(step.delta), limits.max_step);
        EXPECT_LE(std::abs(step.delta - last_delta), limits.max_accel);
        EXPECT_LE(step.sent, target) << "overshot at cycle " << cycle;
        last = step.sent;
        last_delta = step.delta;
    }

    EXPECT_EQ(last, target);
}

TEST(JuxieProtocol, LimiterNeverLeavesTheProtocolRange)
{
    const p::StepResult up = p::limitStep(32668, 32660, 25, {25, 0});
    const p::StepResult down = p::limitStep(-32668, -32660, -25, {25, 0});

    EXPECT_LE(up.sent, p::kRawMax);
    EXPECT_GE(down.sent, p::kRawMin);
}

// ---- receiving ------------------------------------------------------------------------------------------------

TEST(JuxieProtocol, ClassifiesFeedbackAndEmergencyFramesOfOurMotors)
{
    const std::vector<uint8_t> ids = {1, 2};
    CanFrame feedback;
    feedback.id = 0x302;
    CanFrame emergency;
    emergency.id = 0x81;

    EXPECT_EQ(p::classify(feedback, ids).kind, p::FrameKind::Feedback);
    EXPECT_EQ(p::classify(feedback, ids).motor_id, 2);
    EXPECT_EQ(p::classify(emergency, ids).kind, p::FrameKind::Emergency);
    EXPECT_EQ(p::classify(emergency, ids).motor_id, 1);
}

TEST(JuxieProtocol, IgnoresFramesOfOtherMotorsAndOtherKinds)
{
    const std::vector<uint8_t> ids = {1, 2};
    CanFrame other_motor;
    other_motor.id = 0x309;
    CanFrame sync;
    sync.id = 0x80; // motor 0 is not ours
    CanFrame unrelated;
    unrelated.id = 0x500;

    EXPECT_EQ(p::classify(other_motor, ids).kind, p::FrameKind::Other);
    EXPECT_EQ(p::classify(sync, ids).kind, p::FrameKind::Other);
    EXPECT_EQ(p::classify(unrelated, ids).kind, p::FrameKind::Other);
}

TEST(JuxieProtocol, IgnoresExtendedRemoteAndErrorFramesEvenWithAMatchingId)
{
    // The driver used to compare the raw kernel can_id, whose flag bits kept such frames from matching.
    const std::vector<uint8_t> ids = {1};
    for (int variant = 0; variant < 3; ++variant)
    {
        CanFrame frame;
        frame.id = 0x301;
        frame.extended = variant == 0;
        frame.rtr = variant == 1;
        frame.error = variant == 2;

        EXPECT_EQ(p::classify(frame, ids).kind, p::FrameKind::Other) << variant;
    }
}

TEST(JuxieProtocol, DecodesTheTwelveByteFeedback)
{
    const CanFrame frame = feedback12(1, 0, 600, -250, 0, 0x03, 0x80);
    p::MotorFeedback out;

    ASSERT_EQ(p::decodeFeedback(frame, 12, out), p::DecodeResult::Ok);

    EXPECT_NEAR(out.position_rad, 0.0, 1e-9);
    EXPECT_NEAR(out.velocity_rad_s, 600 * 2 * M_PI / 60.0, 1e-9);
    EXPECT_DOUBLE_EQ(out.effort, -250.0);
    EXPECT_TRUE(out.enabled);
    EXPECT_FALSE(out.fault);
    EXPECT_EQ(out.mode, 0x03);
}

TEST(JuxieProtocol, TheTwelveByteFeedbackReportsAFaultFromTheStatusBitOrTheErrorWord)
{
    p::MotorFeedback out;

    ASSERT_EQ(p::decodeFeedback(feedback12(1, 0, 0, 0, 0, 0x03, 0xA0), 12, out), p::DecodeResult::Ok);
    EXPECT_TRUE(out.fault) << "status bit 5";

    ASSERT_EQ(p::decodeFeedback(feedback12(1, 0, 0, 0, 0x0004, 0x03, 0x80), 12, out), p::DecodeResult::Ok);
    EXPECT_TRUE(out.fault) << "non-zero error word";
    EXPECT_EQ(out.error_code, 0x0004);
}

TEST(JuxieProtocol, DecodesTheSevenByteFeedback)
{
    CanFrame frame;
    frame.len = 7;
    frame.data[0] = 0x7F;
    frame.data[1] = 0x9C; // 0x7F9C = 32668
    frame.data[2] = 0xFF;
    frame.data[3] = 0xF6; // -10
    frame.data[6] = 0x20; // enabled, no fault
    p::MotorFeedback out;

    ASSERT_EQ(p::decodeFeedback(frame, 7, out), p::DecodeResult::Ok);

    EXPECT_NEAR(out.position_rad, M_PI, 1e-9);
    EXPECT_NEAR(out.velocity_rad_s, -10 * 2 * M_PI / 60.0, 1e-9);
    EXPECT_DOUBLE_EQ(out.effort, 0.0);
    EXPECT_TRUE(out.enabled);
    EXPECT_FALSE(out.fault);

    frame.data[6] = 0x28; // fault bit
    ASSERT_EQ(p::decodeFeedback(frame, 7, out), p::DecodeResult::Ok);
    EXPECT_TRUE(out.fault);
}

TEST(JuxieProtocol, RejectsShortFramesAndUnknownProtocols)
{
    CanFrame frame;
    frame.len = 6;
    p::MotorFeedback out;

    EXPECT_EQ(p::decodeFeedback(frame, 7, out), p::DecodeResult::TooShort);
    frame.len = 11;
    EXPECT_EQ(p::decodeFeedback(frame, 12, out), p::DecodeResult::TooShort);
    EXPECT_EQ(p::decodeFeedback(frame, 9, out), p::DecodeResult::UnsupportedLen);
}

// ---- error codes ----------------------------------------------------------------------------------------------

TEST(JuxieProtocol, ErrorWordBitsMapToFaultCodesAndMessages)
{
    const auto faults = p::decodeErrorWord(0x0001 | 0x0004 | 0x1000);

    ASSERT_EQ(faults.size(), 3u);
    EXPECT_EQ(faults[0].fault_code, 0x0001);
    EXPECT_STREQ(faults[0].message, "过压");
    EXPECT_EQ(faults[1].fault_code, 0x0003);
    EXPECT_STREQ(faults[1].message, "过温");
    EXPECT_EQ(faults[2].fault_code, 0x000D);
    EXPECT_STREQ(faults[2].message, "位置跟踪误差过大");
    EXPECT_TRUE(p::decodeErrorWord(0).empty());
    EXPECT_TRUE(p::decodeErrorWord(0x2000).empty()) << "an unknown bit is not a known fault";
}

TEST(JuxieProtocol, EmergencyBytesOfTheOlderProtocol)
{
    const auto faults = p::decodeEmergencyBytes(0x80 | 0x01, 0x08);

    ASSERT_EQ(faults.size(), 3u);
    EXPECT_EQ(faults[0].fault_code, 0x0001);
    EXPECT_STREQ(faults[0].message, "负限位保护");
    EXPECT_EQ(faults[1].fault_code, 0x0008);
    EXPECT_STREQ(faults[1].message, "过压");
    EXPECT_EQ(faults[2].fault_code, 0x0009);
    EXPECT_STREQ(faults[2].message, "CAN通信故障");
}

TEST(JuxieProtocol, HardwareErrorCodeLayout)
{
    EXPECT_EQ(p::hardwareErrorCode(2, 0x0003), 0x03020003);
    EXPECT_EQ(p::hardwareErrorCode(0, p::kUnknownFault), 0x03000100);
    EXPECT_EQ(p::kFeedbackTimeoutError, 0x300200);
}
