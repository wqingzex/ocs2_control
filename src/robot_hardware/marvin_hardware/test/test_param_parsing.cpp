#include <gtest/gtest.h>

#include "marvin_hardware/param_parsing.h"

using marvin_hardware::param_parsing::parseDouble;
using marvin_hardware::param_parsing::parseInt;

TEST(ParseInt, AcceptsOneIntegerWithSurroundingWhitespace)
{
    int value = 0;
    EXPECT_TRUE(parseInt("8080", value));
    EXPECT_EQ(value, 8080);
    EXPECT_TRUE(parseInt("  -1 \n", value));
    EXPECT_EQ(value, -1);
}

TEST(ParseInt, RejectsTrailingGarbageEmptyAndOverflow)
{
    int value = 7;
    EXPECT_FALSE(parseInt("12abc", value));  // std::stoi would return 12
    EXPECT_FALSE(parseInt("1.5", value));
    EXPECT_FALSE(parseInt("", value));
    EXPECT_FALSE(parseInt("   ", value));
    EXPECT_FALSE(parseInt("99999999999", value));
    EXPECT_EQ(value, 7);  // untouched on failure
}

TEST(ParseDouble, AcceptsNumbers)
{
    double value = 0.0;
    EXPECT_TRUE(parseDouble("10", value));
    EXPECT_DOUBLE_EQ(value, 10.0);
    EXPECT_TRUE(parseDouble(" 0.5 ", value));
    EXPECT_DOUBLE_EQ(value, 0.5);
    EXPECT_TRUE(parseDouble("1e-3", value));
    EXPECT_DOUBLE_EQ(value, 1e-3);
}

TEST(ParseDouble, RejectsGarbageNonFiniteAndOverflow)
{
    double value = 3.0;
    EXPECT_FALSE(parseDouble("fast", value));
    EXPECT_FALSE(parseDouble("1.0x", value));
    EXPECT_FALSE(parseDouble("", value));
    EXPECT_FALSE(parseDouble("nan", value));
    EXPECT_FALSE(parseDouble("inf", value));
    EXPECT_FALSE(parseDouble("1e999", value));
    EXPECT_DOUBLE_EQ(value, 3.0);
}
