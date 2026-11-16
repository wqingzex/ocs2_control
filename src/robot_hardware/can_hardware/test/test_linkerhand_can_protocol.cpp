// LinkerHand CAN protocol: frame layout, feedback parsing, tactile chunks and unit conversion, without ROS or a bus.

#include <gtest/gtest.h>

#include "can_hardware/hands/linkerhand/linkerhand_can_protocol.h"

namespace lh = can_hardware::linkerhand;
using robot_comm::CanFrame;

namespace
{
    CanFrame positionFeedback(uint32_t id, std::initializer_list<uint8_t> raw)
    {
        CanFrame frame;
        frame.id = id;
        frame.len = static_cast<uint8_t>(raw.size() + 1);
        frame.data[0] = lh::kPositionCommand;
        std::size_t i = 1;
        for (const auto value : raw)
        {
            frame.data[i++] = value;
        }
        return frame;
    }
} // namespace

TEST(LinkerHandCanProtocol, OnlyL6IsSupported)
{
    const auto* l6 = lh::findModel("linkerhand_l6");
    ASSERT_NE(l6, nullptr);
    EXPECT_EQ(l6->joint_count, 6u);
    EXPECT_STREQ(l6->model_name, "l6");
    EXPECT_EQ(lh::findModel("linkerhand_o7"), nullptr);
    EXPECT_EQ(lh::findModel("xhand1"), nullptr);
    EXPECT_EQ(lh::findModel(""), nullptr);
}

TEST(LinkerHandCanProtocol, SidesHaveTheirOwnIds)
{
    EXPECT_EQ(lh::canIdForSide("left"), 0x28u);
    EXPECT_EQ(lh::canIdForSide("right"), 0x27u);
}

TEST(LinkerHandCanProtocol, PositionRequestIsOneByte)
{
    const auto frame = lh::makePositionRequest(0x28);
    EXPECT_EQ(frame.id, 0x28u);
    EXPECT_TRUE(frame.isStandardData());
    EXPECT_FALSE(frame.fd);
    EXPECT_EQ(frame.len, 1);
    EXPECT_EQ(frame.data[0], lh::kPositionCommand);
}

TEST(LinkerHandCanProtocol, JointFrameIsCommandThenOneBytePerJoint)
{
    const lh::RawJoints values{10, 20, 30, 40, 50, 60, 70};
    const auto frame = lh::makeJointFrame(0x27, lh::kMaxTorqueCommand, values, 6);
    EXPECT_EQ(frame.id, 0x27u);
    EXPECT_EQ(frame.len, 7);
    EXPECT_EQ(frame.data[0], lh::kMaxTorqueCommand);
    for (std::size_t i = 0; i < 6; ++i)
    {
        EXPECT_EQ(frame.data[i + 1], values[i]);
    }
    EXPECT_EQ(frame.data[7], 0) << "bytes past the joints stay zero";
}

TEST(LinkerHandCanProtocol, PositionFeedbackIsAcceptedOnlyFromThisHand)
{
    lh::RawJoints raw{};
    EXPECT_TRUE(lh::parsePositionFeedback(positionFeedback(0x28, {1, 2, 3, 4, 5, 6}), 0x28, 6, raw));
    EXPECT_EQ(raw[0], 1);
    EXPECT_EQ(raw[5], 6);

    EXPECT_FALSE(lh::parsePositionFeedback(positionFeedback(0x27, {1, 2, 3, 4, 5, 6}), 0x28, 6, raw)) << "other hand";
    EXPECT_FALSE(lh::parsePositionFeedback(positionFeedback(0x28, {1, 2, 3}), 0x28, 6, raw)) << "too short";

    auto not_position = positionFeedback(0x28, {1, 2, 3, 4, 5, 6});
    not_position.data[0] = lh::kMaxTorqueCommand;
    EXPECT_FALSE(lh::parsePositionFeedback(not_position, 0x28, 6, raw));

    auto extended = positionFeedback(0x28, {1, 2, 3, 4, 5, 6});
    extended.extended = true;
    EXPECT_FALSE(lh::parsePositionFeedback(extended, 0x28, 6, raw));
    auto error = positionFeedback(0x28, {1, 2, 3, 4, 5, 6});
    error.error = true;
    EXPECT_FALSE(lh::parsePositionFeedback(error, 0x28, 6, raw));
}

TEST(LinkerHandCanProtocol, TactileRequestAndChunk)
{
    const auto& l6 = *lh::findModel("linkerhand_l6");
    const auto request = lh::makeTactileRequest(0x27, lh::Finger::Middle, l6.tactile_layout);
    EXPECT_EQ(request.len, 2);
    EXPECT_EQ(request.data[0], 0xB3);
    EXPECT_EQ(request.data[1], l6.tactile_layout);

    CanFrame chunk_frame;
    chunk_frame.id = 0x27;
    chunk_frame.len = static_cast<uint8_t>(l6.tactile_values_per_frame + 2);
    chunk_frame.data[0] = 0xB3;
    chunk_frame.data[1] = 0x30; // row 3, column 0
    for (std::size_t i = 0; i < l6.tactile_values_per_frame; ++i)
    {
        chunk_frame.data[i + 2] = static_cast<uint8_t>(100 + i);
    }
    lh::TactileChunk chunk;
    ASSERT_TRUE(lh::parseTactileFrame(chunk_frame, 0x27, l6, chunk));
    EXPECT_EQ(chunk.finger, lh::Finger::Middle);
    EXPECT_EQ(chunk.row, 3u);
    EXPECT_EQ(chunk.column, 0u);
    EXPECT_EQ(chunk.values[0], 100);

    auto bad_row = chunk_frame;
    bad_row.data[1] = static_cast<uint8_t>(l6.tactile_frame_count << 4);
    EXPECT_FALSE(lh::parseTactileFrame(bad_row, 0x27, l6, chunk));
    auto bad_len = chunk_frame;
    bad_len.len = 4;
    EXPECT_FALSE(lh::parseTactileFrame(bad_len, 0x27, l6, chunk));
    EXPECT_FALSE(lh::parseTactileFrame(positionFeedback(0x27, {1, 2, 3, 4, 5, 6}), 0x27, l6, chunk));
}

TEST(LinkerHandCanProtocol, RawIsInvertedAndClamped)
{
    EXPECT_EQ(lh::radiansToRaw(0.0, 0.0, 1.26), 255) << "lower limit = open = 255";
    EXPECT_EQ(lh::radiansToRaw(1.26, 0.0, 1.26), 0);
    EXPECT_EQ(lh::radiansToRaw(-1.0, 0.0, 1.26), 255);
    EXPECT_EQ(lh::radiansToRaw(9.0, 0.0, 1.26), 0);
    EXPECT_EQ(lh::radiansToRaw(0.5, 0.0, 0.0), 255) << "empty range";
    EXPECT_DOUBLE_EQ(lh::rawToRadians(255, 0.0, 1.26), 0.0);
    EXPECT_DOUBLE_EQ(lh::rawToRadians(0, 0.0, 1.26), 1.26);
    for (int raw = 0; raw <= 255; raw += 17)
    {
        EXPECT_EQ(lh::radiansToRaw(lh::rawToRadians(static_cast<uint8_t>(raw), 0.0, 1.39), 0.0, 1.39), raw);
    }
}

TEST(LinkerHandCanProtocol, ScaleToRaw)
{
    EXPECT_EQ(lh::scaleToRaw(0.0), 0);
    EXPECT_EQ(lh::scaleToRaw(1.0), 255);
    EXPECT_EQ(lh::scaleToRaw(0.5), 128);
    EXPECT_EQ(lh::scaleToRaw(-1.0), 0);
    EXPECT_EQ(lh::scaleToRaw(3.0), 255);
}
