// The planners report through the ROS logging interface (RCLCPP_*_STREAM with the "motion_planning" logger).
// This captures what reaches rcutils, so the test fails if a message goes anywhere else (std::cerr, a custom sink).
#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include <rcutils/logging.h>

#include "motion_planning/path_planner/movej.h"

namespace
{
    struct Captured
    {
        int severity;
        std::string logger;
        std::string text;
    };

    std::vector<Captured>& captured()
    {
        static std::vector<Captured> messages;
        return messages;
    }

    void captureHandler(const rcutils_log_location_t*, int severity, const char* name, rcutils_time_point_value_t,
                        const char* format, va_list* args)
    {
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        captured().push_back({severity, name != nullptr ? name : "", buffer});
    }

    class PlannerLogging : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
            captured().clear();
            previous_ = rcutils_logging_get_output_handler();
            rcutils_logging_set_output_handler(captureHandler);
        }

        void TearDown() override { rcutils_logging_set_output_handler(previous_); }

        static planning::TrajectoryInitParameters twoJointMove()
        {
            planning::TrajectoryInitParameters parameters(2);
            parameters.start_point.joint_pos(0) = 0.0;
            parameters.start_point.joint_pos(1) = 0.0;
            parameters.end_point.joint_pos(0) = 0.5;
            parameters.end_point.joint_pos(1) = -0.3;
            return parameters;
        }

        rcutils_logging_output_handler_t previous_{nullptr};
    };
} // namespace

TEST_F(PlannerLogging, AValidMoveLogsNothing)
{
    planning::moveJ planner;
    EXPECT_TRUE(planner.init(twoJointMove()));
    EXPECT_TRUE(captured().empty());
}

TEST_F(PlannerLogging, ANonZeroStartVelocityIsAnErrorOnTheMotionPlanningLogger)
{
    auto parameters = twoJointMove();
    parameters.start_point.joint_vel(1) = 0.4;

    planning::moveJ planner;
    EXPECT_FALSE(planner.init(parameters));

    ASSERT_EQ(captured().size(), 1u);
    EXPECT_EQ(captured()[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
    EXPECT_EQ(captured()[0].logger, "motion_planning");
    EXPECT_NE(captured()[0].text.find("starting velocity of joint 1"), std::string::npos) << captured()[0].text;
    EXPECT_NE(captured()[0].text.find("is not zero"), std::string::npos) << captured()[0].text;  // was "1is not zero"
}

TEST_F(PlannerLogging, StartAndEndOnTheSamePointIsReported)
{
    auto parameters = twoJointMove();
    parameters.end_point.joint_pos(0) = 0.0;
    parameters.end_point.joint_pos(1) = 0.0;

    planning::moveJ planner;
    EXPECT_FALSE(planner.init(parameters));

    ASSERT_FALSE(captured().empty());
    EXPECT_EQ(captured().back().severity, RCUTILS_LOG_SEVERITY_ERROR);
    EXPECT_NE(captured().back().text.find("same point"), std::string::npos) << captured().back().text;
}
