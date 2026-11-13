#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "fa_teleop/xr_state_json.hpp"

using fa_teleop::parseDeviceStateJson;
using fa_teleop::parsePoseString;

namespace
{
    /// Build a PC-Service style payload: {"value": "<inner json as string>"}.
    std::string wrap(const nlohmann::json& inner)
    {
        return nlohmann::json{{"value", inner.dump()}}.dump();
    }

    nlohmann::json controller(const std::string& pose)
    {
        return {{"pose", pose}, {"trigger", 0.25}, {"grip", 0.75}, {"axisX", 0.5}, {"axisY", -0.5},
                {"axisClick", true}, {"primaryButton", true}, {"secondaryButton", false}, {"menuButton", false}};
    }
} // namespace

TEST(XrStateJson, parsesHeadControllersAndTimestamp)
{
    const nlohmann::json inner = {
        {"Head", {{"pose", "0,1.6,0,0,0,0,1"}}},
        {"Controller", {{"left", controller("-0.2,1.2,-0.3,0,0,0,1")}, {"right", controller("0.2,1.2,-0.3,0,0,0,1")}}},
        {"timeStampNs", 123456789},
    };
    std::string error;
    const auto frame = parseDeviceStateJson(wrap(inner), &error);
    ASSERT_TRUE(frame.has_value()) << error;
    EXPECT_EQ(frame->timestamp_ns, 123456789);
    EXPECT_TRUE(frame->has_head);
    EXPECT_DOUBLE_EQ(frame->head[1], 1.6);
    ASSERT_TRUE(frame->has_left);
    EXPECT_DOUBLE_EQ(frame->left.pose[0], -0.2);
    EXPECT_DOUBLE_EQ(frame->left.trigger, 0.25);
    EXPECT_DOUBLE_EQ(frame->left.grip, 0.75);
    EXPECT_TRUE(frame->left.axis_click);
    EXPECT_TRUE(frame->left.primary_button);
    EXPECT_FALSE(frame->left.secondary_button);
    EXPECT_TRUE(frame->has_right);
}

TEST(XrStateJson, missingControllerLeavesFlagFalse)
{
    const nlohmann::json inner = {{"Head", {{"pose", "0,1.6,0,0,0,0,1"}}}, {"Controller", nlohmann::json::object()}};
    const auto frame = parseDeviceStateJson(wrap(inner));
    ASSERT_TRUE(frame.has_value());
    EXPECT_TRUE(frame->has_head);
    EXPECT_FALSE(frame->has_left);
    EXPECT_FALSE(frame->has_right);
}

TEST(XrStateJson, malformedPoseIsIgnored)
{
    const nlohmann::json inner = {{"Controller", {{"left", controller("1,2,3")}}}};
    const auto frame = parseDeviceStateJson(wrap(inner));
    ASSERT_TRUE(frame.has_value());
    EXPECT_FALSE(frame->has_left);
}

TEST(XrStateJson, invalidJsonReturnsErrorInsteadOfThrowing)
{
    std::string error;
    EXPECT_FALSE(parseDeviceStateJson("{not json", &error).has_value());
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(parseDeviceStateJson(R"({"other": 1})", &error).has_value());
    EXPECT_FALSE(parseDeviceStateJson(R"({"value": "{broken"})", &error).has_value());
}

TEST(XrStateJson, poseStringNeedsExactlySevenNumbers)
{
    EXPECT_TRUE(parsePoseString("1,2,3,0,0,0,1").has_value());
    EXPECT_FALSE(parsePoseString("1,2,3,0,0,0").has_value());
    EXPECT_FALSE(parsePoseString("1,2,3,0,0,0,1,9").has_value());
    EXPECT_FALSE(parsePoseString("1,2,x,0,0,0,1").has_value());
}
