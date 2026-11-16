#include <gtest/gtest.h>

#include "robot_comm_modbus/rtu_frame.h"

namespace rtu = robot_comm::modbus_rtu;

namespace
{
    std::vector<uint8_t> withCrc(std::vector<uint8_t> frame) { return rtu::detail::withCrc(std::move(frame)); }
}  // namespace

TEST(RtuFrame, ReadRequestMatchesTheSpecificationExample)
{
    // Modbus over serial line specification: read 10 holding registers from 0 at slave 1.
    EXPECT_EQ(rtu::readRegisters(0x01, rtu::kReadHoldingRegisters, 0x0000, 0x000A),
              (std::vector<uint8_t>{0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD}));
}

TEST(RtuFrame, WriteRequests)
{
    const auto single = rtu::writeRegister(0x09, 0x03E8, 0x0100);
    ASSERT_EQ(single.size(), 8u);
    EXPECT_EQ(std::vector<uint8_t>(single.begin(), single.begin() + 6),
              (std::vector<uint8_t>{0x09, 0x06, 0x03, 0xE8, 0x01, 0x00}));
    const auto multi = rtu::writeRegisters(0x28, 0x0000, {0x00FF, 0x1234});
    EXPECT_EQ(std::vector<uint8_t>(multi.begin(), multi.end() - 2),
              (std::vector<uint8_t>{0x28, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0xFF, 0x12, 0x34}));
    EXPECT_EQ(rtu::crc16(multi.data(), multi.size() - 2), multi[multi.size() - 2] | (multi.back() << 8));
}

TEST(RtuFrame, ReplyLength)
{
    EXPECT_EQ(rtu::replyLength({}), 0u);
    EXPECT_EQ(rtu::replyLength({0x01}), 0u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x04}), 0u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x04, 0x06}), 11u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x84}), 5u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x10}), 8u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x06}), 8u);
    EXPECT_EQ(rtu::replyLength({0x01, 0x2B, 0x00}), 3u);  // unknown function: ends where it is
    EXPECT_TRUE(rtu::replySpec(std::chrono::milliseconds(40)).frame_length);
}

TEST(RtuFrame, ParseReadReply)
{
    const auto reply = withCrc({0x01, 0x03, 0x04, 0x00, 0x2A, 0x01, 0x00});
    const auto result = rtu::parseReadReply(reply, 0x01, rtu::kReadHoldingRegisters, 2);
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.registers, (std::vector<uint16_t>{0x002A, 0x0100}));
}

TEST(RtuFrame, ReadAcceptsTheOtherReadFunctionCode)
{
    const auto reply = withCrc({0x09, 0x04, 0x02, 0x12, 0x34});
    EXPECT_TRUE(rtu::parseReadReply(reply, 0x09, rtu::kReadHoldingRegisters, 1).ok);
}

TEST(RtuFrame, ParseRejects)
{
    auto good = withCrc({0x01, 0x03, 0x02, 0x00, 0x01});
    auto bad_crc = good;
    bad_crc.back() ^= 0xFF;
    EXPECT_EQ(rtu::parseReadReply(bad_crc, 0x01, 0x03, 1).error, "CRC mismatch");
    EXPECT_NE(rtu::parseReadReply(good, 0x02, 0x03, 1).error.find("reply from slave 1"), std::string::npos);
    EXPECT_NE(rtu::parseReadReply(good, 0x01, 0x03, 2).error.find("expected 4"), std::string::npos);
    EXPECT_NE(rtu::parseReadReply({0x01, 0x03}, 0x01, 0x03, 1).error.find("too short"), std::string::npos);
    EXPECT_NE(rtu::parseWriteReply(good, 0x01, rtu::kWriteSingleRegister, 0, 0).error.find("function code"),
              std::string::npos);
}

TEST(RtuFrame, ExceptionReply)
{
    const auto reply = withCrc({0x01, 0x83, 0x02});
    const auto result = rtu::parseReadReply(reply, 0x01, rtu::kReadHoldingRegisters, 1);
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.exception, 0x02);
}

TEST(RtuFrame, ParseWriteReplyChecksTheEcho)
{
    const auto echo = withCrc({0x28, 0x10, 0x00, 0x06, 0x00, 0x03});
    EXPECT_TRUE(rtu::parseWriteReply(echo, 0x28, rtu::kWriteMultipleRegisters, 0x0006, 3).ok);
    EXPECT_FALSE(rtu::parseWriteReply(echo, 0x28, rtu::kWriteMultipleRegisters, 0x0006, 4).ok);
    const auto single = withCrc({0x09, 0x06, 0x03, 0xE8, 0x00, 0x05});
    EXPECT_TRUE(rtu::parseWriteReply(single, 0x09, rtu::kWriteSingleRegister, 0x03E8, 0x0005).ok);
}
