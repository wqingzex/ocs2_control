#include <gtest/gtest.h>

#include "robot_comm_can/interface_configurators.h"

using robot_comm::CanConfig;
using robot_comm::NoopConfigurator;
using robot_comm::SudoIpConfigurator;

namespace
{
    CanConfig juxieConfig(const std::string& interface = "can0")
    {
        CanConfig config;
        config.interface = interface;
        config.bitrate = 1'000'000;
        config.data_bitrate = 5'000'000;
        config.restart_ms = 100;
        return config;
    }
} // namespace

TEST(SudoIpConfigurator, TheJuxieCommandIsExactlyWhatTheDriverAlwaysRan)
{
    // The same text as in jx_hardware.cpp before the refactor, and as the sudoers rule in scripts/setup.sh.
    EXPECT_EQ(SudoIpConfigurator::buildCommand(juxieConfig()),
              "sudo ip link set can0 down && "
              "sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on");
}

TEST(SudoIpConfigurator, UsesTheConfiguredInterfaceInBothCommands)
{
    EXPECT_EQ(SudoIpConfigurator::buildCommand(juxieConfig("can1")),
              "sudo ip link set can1 down && "
              "sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on");
}

TEST(SudoIpConfigurator, ClassicCanLeavesOutTheFdOptions)
{
    CanConfig config;
    config.interface = "can0";
    config.bitrate = 500'000;
    config.data_bitrate = 0;
    config.restart_ms = 0;

    EXPECT_EQ(SudoIpConfigurator::buildCommand(config),
              "sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000");
}

TEST(SudoIpConfigurator, RefusesAnInterfaceNameThatIsNotPlain)
{
    SudoIpConfigurator configurator;
    CanConfig config = juxieConfig("can0;reboot");

    EXPECT_FALSE(configurator.apply(config));
    EXPECT_FALSE(configurator.lastError().empty());
}

TEST(NoopConfigurator, AcceptsAnything)
{
    NoopConfigurator configurator;

    EXPECT_TRUE(configurator.apply(juxieConfig("vcan0")));
}

TEST(MakeConfigurator, PicksTheConfiguratorFromManageInterface)
{
    CanConfig managed = juxieConfig();
    CanConfig external = juxieConfig();
    external.manage_interface = false;

    EXPECT_NE(dynamic_cast<SudoIpConfigurator*>(robot_comm::makeConfigurator(managed).get()), nullptr);
    EXPECT_NE(dynamic_cast<NoopConfigurator*>(robot_comm::makeConfigurator(external).get()), nullptr);
}
