#include <gtest/gtest.h>
#include "fa_teleop/xr_device_selector.hpp"

TEST(XrDeviceSelector, acceptsOnlyConfiguredHeadset)
{
    fa_teleop::XrDeviceSelector select("PICO-001");
    EXPECT_FALSE(select.accept("OTHER"));
    EXPECT_TRUE(select.accept("PICO-001"));
    EXPECT_FALSE(select.accept("OTHER"));
}

TEST(XrDeviceSelector, firstDeviceLocksSession)
{
    fa_teleop::XrDeviceSelector select;
    EXPECT_FALSE(select.accept(""));
    EXPECT_TRUE(select.accept("FIRST"));
    EXPECT_FALSE(select.accept("SECOND"));
    EXPECT_TRUE(select.accept("FIRST"));
}
