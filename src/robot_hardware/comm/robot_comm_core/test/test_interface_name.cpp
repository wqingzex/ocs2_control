#include <gtest/gtest.h>

#include "robot_comm/interface_name.h"

using robot_comm::isValidInterfaceName;

TEST(InterfaceName, AcceptsTheUsualCanNames)
{
    EXPECT_TRUE(isValidInterfaceName("can0"));
    EXPECT_TRUE(isValidInterfaceName("can1"));
    EXPECT_TRUE(isValidInterfaceName("can_body"));
    EXPECT_TRUE(isValidInterfaceName("vcan0.1"));
    EXPECT_TRUE(isValidInterfaceName("a"));
}

TEST(InterfaceName, AcceptsExactlyFifteenCharacters)
{
    EXPECT_TRUE(isValidInterfaceName("abcdefghijklmno"));
    EXPECT_FALSE(isValidInterfaceName("abcdefghijklmnop"));
}

TEST(InterfaceName, RejectsEmptyAndLeadingDash)
{
    EXPECT_FALSE(isValidInterfaceName(""));
    EXPECT_FALSE(isValidInterfaceName("-can0"));
}

TEST(InterfaceName, RejectsShellMetacharactersAndWhitespace)
{
    EXPECT_FALSE(isValidInterfaceName("can0;reboot"));
    EXPECT_FALSE(isValidInterfaceName("can0 down"));
    EXPECT_FALSE(isValidInterfaceName("$(id)"));
    EXPECT_FALSE(isValidInterfaceName("can0&&ls"));
    EXPECT_FALSE(isValidInterfaceName("can0\n"));
    EXPECT_FALSE(isValidInterfaceName("ca`n0"));
    EXPECT_FALSE(isValidInterfaceName("../can0"));
}
