#include <gtest/gtest.h>

#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "robot_log/log_mode.hpp"

class LogMode : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    void SetUp() override
    {
        unsetenv(robot_log::kLogModeEnvironment);
        saved_default_level_ = rcutils_logging_get_default_logger_level();
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);
    }

    void TearDown() override
    {
        unsetenv(robot_log::kLogModeEnvironment);
        rcutils_logging_set_default_logger_level(saved_default_level_);
    }

    int saved_default_level_{RCUTILS_LOG_SEVERITY_INFO};
};

TEST_F(LogMode, NotSetOrNormalIsNotDebug)
{
    EXPECT_FALSE(robot_log::debugMode());
    setenv(robot_log::kLogModeEnvironment, "normal", 1);
    EXPECT_FALSE(robot_log::debugMode());
    setenv(robot_log::kLogModeEnvironment, "", 1);
    EXPECT_FALSE(robot_log::debugMode());
}

TEST_F(LogMode, DebugIsDebug)
{
    setenv(robot_log::kLogModeEnvironment, "debug", 1);
    EXPECT_TRUE(robot_log::debugMode());
}

TEST_F(LogMode, AnythingElseFallsBackToNormal)
{
    for (const char* bad : {"Debug", "verbose", "debug:xr", " debug", "1"})
    {
        setenv(robot_log::kLogModeEnvironment, bad, 1);
        EXPECT_FALSE(robot_log::debugMode()) << "'" << bad << "'";
    }
}

TEST_F(LogMode, DebugPutsTheDefaultLevelOfTheProcessAtDebug)
{
    setenv(robot_log::kLogModeEnvironment, "debug", 1);
    EXPECT_TRUE(robot_log::applyLogMode());
    EXPECT_EQ(rcutils_logging_get_default_logger_level(), RCUTILS_LOG_SEVERITY_DEBUG);
}

TEST_F(LogMode, DebugKeepsTheNoisyRosInternalsAtInfoWhileOtherLoggersFollowTheDefault)
{
    setenv(robot_log::kLogModeEnvironment, "debug", 1);
    robot_log::applyLogMode();
    for (const auto& name : robot_log::noisyRosLoggers())
    {
        EXPECT_EQ(rcutils_logging_get_logger_effective_level(name.c_str()), RCUTILS_LOG_SEVERITY_INFO) << name;
    }
    // A child of a pinned logger follows it; a logger the software creates has no entry and follows the default.
    EXPECT_EQ(rcutils_logging_get_logger_effective_level("controller_manager.pal_statistics"), RCUTILS_LOG_SEVERITY_INFO);
    for (const char* ours : {"arm_controller", "motion_planning", "teleop_command", "xr_target_node", "some_new_component"})
    {
        EXPECT_EQ(rcutils_logging_get_logger_effective_level(ours), RCUTILS_LOG_SEVERITY_DEBUG) << ours;
    }
}

TEST_F(LogMode, NormalTouchesNothing)
{
    EXPECT_FALSE(robot_log::applyLogMode());
    EXPECT_EQ(rcutils_logging_get_default_logger_level(), RCUTILS_LOG_SEVERITY_INFO);

    // A level given on the command line (--ros-args --log-level warn) still counts in normal.
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
    EXPECT_FALSE(robot_log::applyLogMode());
    EXPECT_EQ(rcutils_logging_get_default_logger_level(), RCUTILS_LOG_SEVERITY_WARN);
}

TEST_F(LogMode, ApplyingTwiceChangesNothingMore)
{
    setenv(robot_log::kLogModeEnvironment, "debug", 1);
    robot_log::applyLogMode();
    robot_log::applyLogMode();
    EXPECT_EQ(rcutils_logging_get_default_logger_level(), RCUTILS_LOG_SEVERITY_DEBUG);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return result;
}
