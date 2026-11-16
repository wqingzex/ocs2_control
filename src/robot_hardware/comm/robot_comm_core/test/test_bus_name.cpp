#include <gtest/gtest.h>

#include "robot_comm/bus_name.h"

using robot_comm::parseBusName;

namespace
{
    std::string errorOf(const std::string& text)
    {
        std::string error;
        EXPECT_FALSE(parseBusName(text, error).has_value()) << text;
        return error;
    }
}  // namespace

TEST(BusName, UsbRs485WithDefaults)
{
    std::string error;
    const auto name = parseBusName("usb_rs485:///dev/ttyUSB0", error);
    ASSERT_TRUE(name) << error;
    EXPECT_EQ(name->scheme, "usb_rs485");
    EXPECT_EQ(name->device, "/dev/ttyUSB0");
    EXPECT_EQ(name->baud(), 115200);
    EXPECT_TRUE(name->lowLatency());
    EXPECT_EQ(name->text(), "usb_rs485:///dev/ttyUSB0?baud=115200&low_latency=1");
}

TEST(BusName, UsbSerialOptions)
{
    std::string error;
    const auto name = parseBusName("usb_serial:///dev/serial/by-id/usb-FTDI_A9BJ-if00?low_latency=0&baud=3000000", error);
    ASSERT_TRUE(name) << error;
    EXPECT_EQ(name->device, "/dev/serial/by-id/usb-FTDI_A9BJ-if00");
    EXPECT_EQ(name->baud(), 3000000);
    EXPECT_FALSE(name->lowLatency());
}

TEST(BusName, ArmPorts)
{
    std::string error;
    const auto rs485 = parseBusName("arm_rs485://192.168.1.190/left/port2", error);
    ASSERT_TRUE(rs485) << error;
    EXPECT_EQ(rs485->host, "192.168.1.190");
    EXPECT_EQ(rs485->side, "left");
    EXPECT_EQ(rs485->port, "port2");
    EXPECT_TRUE(rs485->isArm());
    EXPECT_EQ(rs485->key(), "arm_rs485://192.168.1.190/left/port2");

    const auto canfd = parseBusName("arm_canfd://192.168.1.190/right", error);
    ASSERT_TRUE(canfd) << error;
    EXPECT_EQ(canfd->side, "right");
    EXPECT_TRUE(canfd->port.empty());
}

TEST(BusName, UsbCan)
{
    std::string error;
    const auto name = parseBusName("usb_can://can0", error);
    ASSERT_TRUE(name) << error;
    EXPECT_EQ(name->device, "can0");
}

TEST(BusName, SameUsbPortIsOneLineWhateverTheKind)
{
    std::string error;
    const auto a = parseBusName("usb_rs485:///dev/ttyUSB0", error);
    const auto b = parseBusName("usb_serial:///dev/ttyUSB0?baud=2500000", error);
    ASSERT_TRUE(a && b);
    EXPECT_EQ(a->key(), b->key());
    const auto left = parseBusName("arm_rs485://10.0.0.1/left/port1", error);
    const auto right = parseBusName("arm_rs485://10.0.0.1/right/port1", error);
    ASSERT_TRUE(left && right);
    EXPECT_NE(left->key(), right->key());
}

TEST(BusName, RejectsMalformedNames)
{
    EXPECT_NE(errorOf("/dev/ttyUSB0").find("no '<kind>://' prefix"), std::string::npos);
    EXPECT_NE(errorOf("rs485:///dev/ttyUSB0").find("unknown kind 'rs485'"), std::string::npos);
    EXPECT_NE(errorOf("usb_rs485://ttyUSB0").find("absolute device path"), std::string::npos);
    EXPECT_NE(errorOf("usb_rs485:///dev/ttyUSB0?baud=fast").find("baud must be a positive integer"), std::string::npos);
    EXPECT_NE(errorOf("usb_rs485:///dev/ttyUSB0?parity=E").find("unknown option 'parity'"), std::string::npos);
    EXPECT_NE(errorOf("usb_rs485:///dev/ttyUSB0?baud").find("malformed option"), std::string::npos);
    EXPECT_NE(errorOf("arm_rs485://192.168.1.190/A/port1").find("side must be left or right"), std::string::npos);
    EXPECT_NE(errorOf("arm_rs485://192.168.1.190/left").find("needs a port"), std::string::npos);
    EXPECT_NE(errorOf("arm_rs485://192.168.1.190/left/com1").find("needs a port"), std::string::npos);
    EXPECT_NE(errorOf("arm_canfd://192.168.1.190/left/port1").find("has no port"), std::string::npos);
    EXPECT_NE(errorOf("arm_rs485://bad host/left/port1").find("is not an address"), std::string::npos);
    EXPECT_NE(errorOf("arm_rs485://1.2.3.4/left/port1?baud=1").find("options are only allowed"), std::string::npos);
    EXPECT_NE(errorOf("usb_can://-x").find("not a valid interface name"), std::string::npos);
}
