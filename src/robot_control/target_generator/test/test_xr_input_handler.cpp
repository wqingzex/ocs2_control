// XrInputHandler end to end inside one process: headset topics in, FSM commands / arm targets / hand
// percentages out. Key table: docs/使用手册.md section 6.1 (event codes in xr_teleop/button_event_encoder.hpp).
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "target_generator/XrInputHandler.h"

using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;

namespace
{
    // Event codes published by xr_bridge on /teleop/controller_state.
    constexpr int kFsmForward = 1;
    constexpr int kFsmBack = 2;
    constexpr int kToggleFollow = 3;
    // FSM states / commands on /fsm_command.
    constexpr int kHome = 1, kHold = 2, kOcs2 = 3, kMoveJ = 4, kRest = 100;

    Pose pose(double x, double y, double z)
    {
        Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.w = 1.0;
        return p;
    }

    class XrInputHandlerTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
        }
        static void TearDownTestSuite() { rclcpp::shutdown(); }

        void SetUp() override
        {
            // A longer stale limit than the 200 ms default: the tests publish a pose, then spin for 150 ms between steps.
            rclcpp::NodeOptions options;
            options.parameter_overrides({rclcpp::Parameter("xr_guard.stale_ms", 500)});
            node_ = std::make_shared<rclcpp::Node>("target_generator_under_test", options);
            probe_ = std::make_shared<rclcpp::Node>("xr_input_handler_probe");

            auto left_target = node_->create_publisher<Pose>("left_target", 10);
            auto right_target = node_->create_publisher<Pose>("right_target", 10);
            handler_ = std::make_unique<target_generator::XrInputHandler>(
                node_, nullptr, left_target, right_target,
                std::vector<std::string>{"left_hand_controller", "right_hand_controller"},
                1.0);

            event_pub_ = probe_->create_publisher<std_msgs::msg::Int32>("/teleop/controller_state", 10);
            fsm_state_pub_ = probe_->create_publisher<std_msgs::msg::Int32>("/fsm_state", rclcpp::QoS(1).transient_local());
            left_pose_pub_ = probe_->create_publisher<Pose>("/teleop/left_ee_pose", 10);
            right_pose_pub_ = probe_->create_publisher<Pose>("/teleop/right_ee_pose", 10);
            trigger_pub_ = probe_->create_publisher<geometry_msgs::msg::Twist>("/teleop/trigger_values", 10);
            fsm_sub_ = probe_->create_subscription<std_msgs::msg::Int32>(
                "/fsm_command", 10, [this](std_msgs::msg::Int32::SharedPtr m) { fsm_.push_back(m->data); });
            target_sub_ = probe_->create_subscription<Pose>(
                "left_target", 10, [this](Pose::SharedPtr m) { left_targets_.push_back(*m); });
            right_target_sub_ = probe_->create_subscription<Pose>(
                "right_target", 10, [this](Pose::SharedPtr m) { right_targets_.push_back(*m); });
            left_hand_sub_ = probe_->create_subscription<std_msgs::msg::Float64>(
                "/left_hand_controller/target_percent", 10,
                [this](std_msgs::msg::Float64::SharedPtr m) { left_hand_ = m->data; });
            right_hand_sub_ = probe_->create_subscription<std_msgs::msg::Float64>(
                "/right_hand_controller/target_percent", 10,
                [this](std_msgs::msg::Float64::SharedPtr m) { right_hand_ = m->data; });

            executor_.add_node(node_);
            executor_.add_node(probe_);
            waitForSubscribers();
            left_pose_pub_->publish(pose(0.0, 0.0, 0.0));  // a fresh controller pose, as the start of following needs
            spinFor(100ms);
            robotLeftAt(0.30, 0.20, 0.40);
        }

        void TearDown() override
        {
            executor_.cancel();
            handler_.reset();
        }

        void waitForSubscribers()
        {
            spinUntil([this]() {
                return event_pub_->get_subscription_count() > 0 && left_pose_pub_->get_subscription_count() > 0 &&
                       trigger_pub_->get_subscription_count() > 0;
            });
        }

        bool spinUntil(const std::function<bool()>& done, std::chrono::milliseconds timeout = 3000ms)
        {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (done()) { return true; }
                executor_.spin_some(10ms);
                std::this_thread::sleep_for(5ms);
            }
            return done();
        }

        void spinFor(std::chrono::milliseconds duration)
        {
            spinUntil([]() { return false; }, duration);
        }

        void press(int event)
        {
            std_msgs::msg::Int32 m;
            m.data = event;
            event_pub_->publish(m);
            spinFor(150ms);
        }

        /// The controller reports its real state on /fsm_state.
        void fsmIs(int state)
        {
            std_msgs::msg::Int32 m;
            m.data = state;
            fsm_state_pub_->publish(m);
            spinFor(100ms);
        }

        void robotLeftAt(double x, double y, double z)
        {
            auto m = std::make_shared<geometry_msgs::msg::PoseStamped>();
            m->header.frame_id = "base_link";
            m->pose = pose(x, y, z);
            handler_->robotLeftPoseCallback(m);
        }

        void moveLeftController(double x, double y, double z)
        {
            left_pose_pub_->publish(pose(x, y, z));
            spinFor(150ms);
        }

        void moveRightController(double x, double y, double z)
        {
            right_pose_pub_->publish(pose(x, y, z));
            spinFor(150ms);
        }

        void robotRightAt(double x, double y, double z)
        {
            auto m = std::make_shared<geometry_msgs::msg::PoseStamped>();
            m->header.frame_id = "base_link";
            m->pose = pose(x, y, z);
            handler_->robotRightPoseCallback(m);
        }

        /// The first command sent after a key press (ignores the 0 reset after a REST pulse).
        std::optional<int> sentFsm() const
        {
            for (int c : fsm_) { if (c != 0) { return c; } }
            return std::nullopt;
        }

        rclcpp::Node::SharedPtr node_, probe_;
        std::unique_ptr<target_generator::XrInputHandler> handler_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr event_pub_, fsm_state_pub_;
        rclcpp::Publisher<Pose>::SharedPtr left_pose_pub_, right_pose_pub_;
        rclcpp::Subscription<Pose>::SharedPtr right_target_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr trigger_pub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_sub_;
        rclcpp::Subscription<Pose>::SharedPtr target_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_hand_sub_, right_hand_sub_;
        std::vector<int> fsm_;
        std::vector<Pose> left_targets_, right_targets_;
        std::optional<double> left_hand_, right_hand_;
    };
} // namespace

TEST_F(XrInputHandlerTest, rightAStepsTheFsmForward)
{
    fsmIs(kHome);
    press(kFsmForward);
    EXPECT_EQ(sentFsm(), kHold);

    fsm_.clear();
    fsmIs(kHold);
    press(kFsmForward);
    EXPECT_EQ(sentFsm(), kOcs2);

    fsm_.clear();
    fsmIs(kOcs2);
    press(kFsmForward);  // nothing beyond OCS2
    EXPECT_EQ(sentFsm(), std::nullopt);
}

TEST_F(XrInputHandlerTest, leftXStepsTheFsmBack)
{
    fsmIs(kOcs2);
    press(kFsmBack);
    EXPECT_EQ(sentFsm(), kHold);

    fsm_.clear();
    fsmIs(kMoveJ);
    press(kFsmBack);
    EXPECT_EQ(sentFsm(), kHold);

    fsm_.clear();
    fsmIs(kHold);
    press(kFsmBack);
    EXPECT_EQ(sentFsm(), kHome);

    fsm_.clear();
    fsmIs(kHome);
    press(kFsmBack);  // HOME <-> REST
    EXPECT_EQ(sentFsm(), kRest);
}

TEST_F(XrInputHandlerTest, codesOfTheFormerKeySchemesDoNothing)
{
    fsmIs(kOcs2);
    for (int old_code : {4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 29, 50, 51, 52, 53, 54})
    {
        press(old_code);
    }
    EXPECT_TRUE(fsm_.empty());
    moveLeftController(0.10, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty()) << "an old code started following";
}

TEST_F(XrInputHandlerTest, rightStickStartsFollowingOnlyInOcs2)
{
    fsmIs(kHold);
    press(kToggleFollow);
    moveLeftController(0.10, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty()) << "followed outside OCS2";

    fsmIs(kOcs2);
    press(kToggleFollow);                // UPDATE: base = controller now, robot arm now
    moveLeftController(0.15, 0.0, 0.0);  // 5 cm forward from where following started
    ASSERT_FALSE(left_targets_.empty());
    const auto& target = left_targets_.back();
    EXPECT_NEAR(target.position.x, 0.35, 1e-6);
    EXPECT_NEAR(target.position.y, 0.20, 1e-6);
    EXPECT_NEAR(target.position.z, 0.40, 1e-6);
}

TEST_F(XrInputHandlerTest, rightStickAgainStopsFollowing)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.05, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());

    press(kToggleFollow);  // STORAGE
    left_targets_.clear();
    moveLeftController(0.20, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty());
}

// Whatever the source (headset glitch, tracking origin moved, bad frame), one frame must not drag the arm far:
// a target far from the end effector, or a big step from the previous target, is dropped.
TEST_F(XrInputHandlerTest, aTargetFarFromTheArmIsDropped)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(2.0, 1.0, 0.0);  // arm is at (0.30, 0.20, 0.40): the target would be ~2.3 m away
    // (A stale frame from SetUp may still produce a target at the arm itself, so look at what was sent, not how many.)
    for (const auto& target : left_targets_)
    {
        EXPECT_LT(target.position.x, 1.0) << "a target 2 m away was sent";
    }
}

TEST_F(XrInputHandlerTest, aBigStepFromThePreviousTargetIsDroppedAndTheNextSmallOneIsKept)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.04, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());
    EXPECT_NEAR(left_targets_.back().position.x, 0.34, 1e-6);

    moveLeftController(0.14, 0.0, 0.0);  // +10 cm in one frame, still within reach of the arm
    for (const auto& target : left_targets_)
    {
        EXPECT_LT(target.position.x, 0.40) << "a 10 cm step was sent";
    }

    moveLeftController(0.08, 0.0, 0.0);  // +4 cm from the last accepted target
    EXPECT_NEAR(left_targets_.back().position.x, 0.38, 1e-6);
}

TEST_F(XrInputHandlerTest, theStepReferenceStartsOverWhenFollowingIsStartedAgain)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.04, 0.0, 0.0);  // last target of this session: (0.34, 0.20, 0.40)
    ASSERT_FALSE(left_targets_.empty());
    EXPECT_NEAR(left_targets_.back().position.x, 0.34, 1e-6);

    press(kToggleFollow);  // stop
    moveLeftController(0.0, 0.0, 0.0);
    press(kToggleFollow);  // start again, base = controller now
    left_targets_.clear();
    moveLeftController(0.0, 0.04, 0.0);  // new target (0.30, 0.24, 0.40) is 5.6 cm from the old one, 4 cm from the arm
    ASSERT_FALSE(left_targets_.empty());
    EXPECT_NEAR(left_targets_.back().position.y, 0.24, 1e-6)
        << "the previous session's target was used as the step reference";
}

// The controller ignores HOME while in OCS2 (only HOLD is accepted), so the state stays OCS2. The handler must go by
// the state the controller reports, not by the command that was sent.
TEST_F(XrInputHandlerTest, aCommandTheControllerIgnoresDoesNotStopFollowing)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.03, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());

    auto command_pub = probe_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    std_msgs::msg::Int32 home;
    home.data = kHome;
    command_pub->publish(home);  // no /fsm_state follows: the controller stayed in OCS2
    spinFor(150ms);

    left_targets_.clear();
    moveLeftController(0.06, 0.0, 0.0);
    EXPECT_FALSE(left_targets_.empty()) << "following stopped because of a command that changed nothing";
}

TEST_F(XrInputHandlerTest, leavingOcs2StopsFollowingAndNeedsTheStickAgain)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.05, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());

    fsmIs(kHold);  // e.g. left X: back to STORAGE automatically
    fsmIs(kOcs2);  // right A again: no jump, the arms wait for the stick
    left_targets_.clear();
    moveLeftController(0.20, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty());
}

// Checkpoint 1 of the XR path: an arm starts following only when its input can be trusted.
TEST_F(XrInputHandlerTest, theStickDoesNotStartFollowingWithoutAFreshControllerPose)
{
    fsmIs(kOcs2);
    spinFor(800ms);  // the last pose is older than stale_ms
    press(kToggleFollow);
    moveLeftController(0.10, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty()) << "followed from a stale base";

    moveLeftController(0.10, 0.0, 0.0);  // now there is a fresh pose
    press(kToggleFollow);
    moveLeftController(0.13, 0.0, 0.0);
    EXPECT_FALSE(left_targets_.empty());
}

// The target is not quantized: a hand that moves 1 mm moves the target 1 mm.
TEST_F(XrInputHandlerTest, aSmallMoveOfTheHandMovesTheTarget)
{
    fsmIs(kOcs2);
    press(kToggleFollow);
    moveLeftController(0.001, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());
    EXPECT_NEAR(left_targets_.back().position.x, 0.301, 1e-6);  // the arm is at x = 0.30
    moveLeftController(0.002, 0.0, 0.0);
    EXPECT_NEAR(left_targets_.back().position.x, 0.302, 1e-6);
}

// The right arm has no current pose from the robot (single-arm robot, or the topic is not there yet): its base would
// be the default (0, 0, 0), so it must not follow; the left arm still does.
TEST_F(XrInputHandlerTest, anArmWithoutACurrentPoseFromTheRobotDoesNotFollow)
{
    fsmIs(kOcs2);
    moveRightController(0.0, 0.0, 0.0);
    press(kToggleFollow);
    moveLeftController(0.03, 0.0, 0.0);
    moveRightController(0.05, 0.0, 0.0);
    EXPECT_FALSE(left_targets_.empty()) << "the left arm should follow";
    EXPECT_TRUE(right_targets_.empty()) << "the right arm followed without a current pose";

    robotRightAt(0.30, -0.20, 0.40);  // the robot reports it now
    press(kToggleFollow);             // stop
    moveRightController(0.0, 0.0, 0.0);
    press(kToggleFollow);             // start again
    moveRightController(0.03, 0.0, 0.0);
    ASSERT_FALSE(right_targets_.empty());
    EXPECT_NEAR(right_targets_.back().position.x, 0.33, 1e-6);
}

TEST_F(XrInputHandlerTest, triggersOpenAndCloseTheHandsProportionally)
{
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.7;   // left trigger 70 % pulled
    t.angular.x = 1.0;  // right trigger fully pulled
    trigger_pub_->publish(t);
    ASSERT_TRUE(spinUntil([this]() { return left_hand_ && right_hand_; }));
    EXPECT_NEAR(*left_hand_, 0.3, 1e-9);  // target_percent: 1 open, 0 closed
    EXPECT_NEAR(*right_hand_, 0.0, 1e-9);
}

TEST_F(XrInputHandlerTest, noKeySwitchesTheTriggersAwayFromProportionalControl)
{
    press(15);  // formerly left Y + right B: switch the triggers to open/close toggling
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.4;
    trigger_pub_->publish(t);
    ASSERT_TRUE(spinUntil([this]() { return left_hand_.has_value(); }));
    EXPECT_NEAR(*left_hand_, 0.6, 1e-9);
}
