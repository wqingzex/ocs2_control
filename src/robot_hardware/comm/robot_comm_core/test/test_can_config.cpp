#include <gtest/gtest.h>

#include "robot_comm/can.h"

using robot_comm::CanConfig;
using robot_comm::parseCanConfig;

namespace
{
    CanConfig juxieDefaults()
    {
        CanConfig defaults;
        defaults.bitrate = 1'000'000;
        defaults.data_bitrate = 5'000'000;
        defaults.restart_ms = 100;
        return defaults;
    }
} // namespace

TEST(CanConfig, KeepsTheDefaultsWhenNothingIsGiven)
{
    CanConfig config;
    std::string error;

    ASSERT_TRUE(parseCanConfig({}, "", juxieDefaults(), config, error)) << error;

    EXPECT_EQ(config.interface, "can0");
    EXPECT_EQ(config.bitrate, 1'000'000u);
    EXPECT_EQ(config.data_bitrate, 5'000'000u);
    EXPECT_EQ(config.restart_ms, 100);
    EXPECT_TRUE(config.manage_interface);
    EXPECT_TRUE(config.fd());
}

TEST(CanConfig, ReadsTheSharedParameterNames)
{
    CanConfig config;
    std::string error;
    const std::unordered_map<std::string, std::string> params = {
        {"can_interface", "can1"},     {"can_bitrate", "500000"},   {"can_data_bitrate", "2000000"},
        {"can_restart_ms", "0"},       {"can_manage_interface", "false"},
    };

    ASSERT_TRUE(parseCanConfig(params, "", juxieDefaults(), config, error)) << error;

    EXPECT_EQ(config.interface, "can1");
    EXPECT_EQ(config.bitrate, 500'000u);
    EXPECT_EQ(config.data_bitrate, 2'000'000u);
    EXPECT_EQ(config.restart_ms, 0);
    EXPECT_FALSE(config.manage_interface);
}

TEST(CanConfig, ADevicePrefixSelectsItsOwnKeys)
{
    CanConfig config;
    std::string error;
    const std::unordered_map<std::string, std::string> params = {
        {"left_ee_can_interface", "can2"}, {"can_interface", "can0"}};

    ASSERT_TRUE(parseCanConfig(params, "left_ee_", CanConfig{}, config, error)) << error;

    EXPECT_EQ(config.interface, "can2");
    EXPECT_FALSE(config.fd());
}

TEST(CanConfig, RejectsAnInterfaceNameThatIsNotPlain)
{
    CanConfig config;
    std::string error;

    EXPECT_FALSE(parseCanConfig({{"can_interface", "can0;reboot"}}, "", CanConfig{}, config, error));
    EXPECT_NE(error.find("can_interface"), std::string::npos);
}

TEST(CanConfig, RejectsBadNumbersAndBooleans)
{
    CanConfig config;
    std::string error;

    EXPECT_FALSE(parseCanConfig({{"can_bitrate", "fast"}}, "", CanConfig{}, config, error));
    EXPECT_FALSE(parseCanConfig({{"can_bitrate", "-5"}}, "", CanConfig{}, config, error));
    EXPECT_FALSE(parseCanConfig({{"can_bitrate", "0"}}, "", CanConfig{}, config, error));
    EXPECT_FALSE(parseCanConfig({{"can_data_bitrate", "1e6"}}, "", CanConfig{}, config, error));
    EXPECT_FALSE(parseCanConfig({{"can_restart_ms", "999999"}}, "", CanConfig{}, config, error));
    EXPECT_FALSE(parseCanConfig({{"can_manage_interface", "maybe"}}, "", CanConfig{}, config, error));
}

TEST(CanConfig, AFailedParseLeavesTheOutputUntouched)
{
    CanConfig config;
    config.interface = "keep";
    std::string error;

    ASSERT_FALSE(parseCanConfig({{"can_bitrate", "x"}}, "", CanConfig{}, config, error));

    EXPECT_EQ(config.interface, "keep");
}

TEST(CanFrame, OnlyAPlainDataFrameIsStandardData)
{
    robot_comm::CanFrame frame;
    EXPECT_TRUE(frame.isStandardData());
    frame.extended = true;
    EXPECT_FALSE(frame.isStandardData());
    frame.extended = false;
    frame.error = true;
    EXPECT_FALSE(frame.isStandardData());
}
