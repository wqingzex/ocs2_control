// XrInputHandler end to end inside one process: headset topics in, FSM commands / arm targets / hand
// percentages out. Key table: docs/使用手册.md section 6.1 (event codes in xr_teleop/button_event_encoder.hpp).
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
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
#include <robot_control_msgs/msg/whole_body_current_state.hpp>
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
    constexpr int kToggleChassisDrive = 4;
    // Base drive scales the fixture sets (full stick = this speed).
    constexpr double kLinearScale = 0.1;   // m/s
    constexpr double kAngularScale = 0.2;  // rad/s
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

        /// Whether the handler under test is built with xr_base = true (the chassis tests need it).
        virtual bool chassisDriveEnabled() const { return true; }

        void SetUp() override
        {
            // A longer stale limit than the 200 ms default: the tests publish a pose, then spin for 150 ms between steps.
            rclcpp::NodeOptions options;
            std::vector<rclcpp::Parameter> parameters = {rclcpp::Parameter("xr_guard.stale_ms", 500),
                                                         rclcpp::Parameter("xr_chassis.stale_ms", 500),
                                                         rclcpp::Parameter("xr_chassis.linear_scale", kLinearScale),
                                                         rclcpp::Parameter("xr_chassis.angular_scale", kAngularScale)};
            // Chassis drive is off unless the parameter says so (the default), see XrInputHandlerChassisOffTest.
            if (chassisDriveEnabled()) { parameters.emplace_back("xr_base", true); }
            options.parameter_overrides(parameters);
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
            head_pose_pub_ = probe_->create_publisher<Pose>("/teleop/head_pose", 10);
            robot_head_pub_ = probe_->create_publisher<geometry_msgs::msg::PoseStamped>("head_current_pose", 10);
            whole_body_state_pub_ = probe_->create_publisher<robot_control_msgs::msg::WholeBodyCurrentState>(
                "/whole_body_controller/current_state", 10);
            head_target_sub_ = probe_->create_subscription<Pose>(
                "head_target", 10, [this](Pose::SharedPtr m) { head_targets_.push_back(*m); });
            thumbstick_pub_ = probe_->create_publisher<geometry_msgs::msg::Twist>("/teleop/thumbstick_axes", 10);
            base_velocity_sub_ = probe_->create_subscription<geometry_msgs::msg::Twist>(
                "base_velocity_target", 10,
                [this](geometry_msgs::msg::Twist::SharedPtr m) { base_velocities_.push_back(*m); });
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
                       trigger_pub_->get_subscription_count() > 0 && head_pose_pub_->get_subscription_count() > 0 &&
                       robot_head_pub_->get_subscription_count() > 0 &&
                       whole_body_state_pub_->get_subscription_count() > 0 &&
                       thumbstick_pub_->get_subscription_count() > 0;
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

        /// The whole body controller reports its head task (WholeBodyCurrentState::HEAD_*).
        void headTaskIs(std::uint8_t head_state)
        {
            robot_control_msgs::msg::WholeBodyCurrentState m;
            m.head_state = head_state;
            whole_body_state_pub_->publish(m);
            spinFor(100ms);
        }

        void robotHeadAt(double x, double y, double z)
        {
            geometry_msgs::msg::PoseStamped m;
            m.header.frame_id = "base_footprint";
            m.pose = pose(x, y, z);
            robot_head_pub_->publish(m);
            spinFor(100ms);
        }

        void moveHead(const Pose& p)
        {
            head_pose_pub_->publish(p);
            spinFor(150ms);
        }

        void moveHead(double x, double y, double z) { moveHead(pose(x, y, z)); }


        /// One /teleop/thumbstick_axes frame as xr_teleop publishes it (forward = negative y).
        void sticks(double left_x, double left_y, double right_x, double right_y)
        {
            geometry_msgs::msg::Twist t;
            t.linear.x = left_x;
            t.linear.y = left_y;
            t.angular.x = right_x;
            t.angular.y = right_y;
            thumbstick_pub_->publish(t);
            spinFor(100ms);
        }

        /// OCS2, base locked, fresh stick input, then the left stick key: the sticks now drive the base.
        void startChassisDrive()
        {
            fsmIs(kOcs2);
            sticks(0.0, 0.0, 0.0, 0.0);
            press(kToggleChassisDrive);
        }

        /// The last base velocity that was published (all zero when there is none).
        geometry_msgs::msg::Twist lastBaseVelocity() const
        {
            return base_velocities_.empty() ? geometry_msgs::msg::Twist{} : base_velocities_.back();
        }

        static bool isStopped(const geometry_msgs::msg::Twist& t)
        {
            return t.linear.x == 0.0 && t.linear.y == 0.0 && t.linear.z == 0.0 && t.angular.x == 0.0 &&
                   t.angular.y == 0.0 && t.angular.z == 0.0;
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
        rclcpp::Publisher<Pose>::SharedPtr head_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_head_pub_;
        rclcpp::Publisher<robot_control_msgs::msg::WholeBodyCurrentState>::SharedPtr whole_body_state_pub_;
        rclcpp::Subscription<Pose>::SharedPtr head_target_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr thumbstick_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_velocity_sub_;
        std::vector<geometry_msgs::msg::Twist> base_velocities_;
        std::vector<Pose> left_targets_, right_targets_, head_targets_;
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
    for (int old_code : {5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 29, 50, 51, 52, 53, 54})
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

// Head tracking: the head follows the headset only while the arms follow (OCS2, right stick), and only while the whole
// body controller reports its head task as HEAD_TRACKING. The head target is in the robot's base frame, the same frame
// as head_current_pose, so the headset's movement is added to the robot head as it is.
namespace
{
    using WholeBodyState = robot_control_msgs::msg::WholeBodyCurrentState;
}

TEST_F(XrInputHandlerTest, theHeadFollowsWhileFollowingAndTheHeadTaskIsTracking)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    moveHead(0.0, 0.0, 0.0);
    EXPECT_TRUE(head_targets_.empty()) << "the head followed before the arms started following";

    moveLeftController(0.0, 0.0, 0.0);  // a fresh controller pose (the steps above took longer than stale_ms)
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);  // the base of the head is taken here: the target is the robot head itself
    ASSERT_FALSE(head_targets_.empty());
    EXPECT_NEAR(head_targets_.back().position.x, 0.0, 1e-6);
    EXPECT_NEAR(head_targets_.back().position.z, 1.5, 1e-6);

    moveHead(0.02, 0.0, 0.01);
    EXPECT_NEAR(head_targets_.back().position.x, 0.02, 1e-6);
    EXPECT_NEAR(head_targets_.back().position.z, 1.51, 1e-6);
}

TEST_F(XrInputHandlerTest, theHeadsTurnIsAppliedToTheRobotHead)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());

    Pose turned = pose(0.0, 0.0, 0.0);
    turned.orientation.z = std::sin(0.25);  // 0.5 rad about +Z
    turned.orientation.w = std::cos(0.25);
    moveHead(turned);
    const auto& q = head_targets_.back().orientation;
    EXPECT_NEAR(q.z, std::sin(0.25), 1e-6);
    EXPECT_NEAR(q.w, std::cos(0.25), 1e-6);
}

TEST_F(XrInputHandlerTest, theHeadDoesNotFollowUnlessTheHeadTaskIsTracking)
{
    fsmIs(kOcs2);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    for (const auto state : {WholeBodyState::HEAD_DISABLED, WholeBodyState::HEAD_GAZE, WholeBodyState::HEAD_FORWARD})
    {
        headTaskIs(state);
        moveHead(0.01, 0.0, 0.0);
    }
    EXPECT_TRUE(head_targets_.empty());
}

TEST_F(XrInputHandlerTest, theHeadDoesNotFollowWithoutACurrentHeadPoseFromTheRobot)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    press(kToggleFollow);  // head_current_pose never arrived: the base would be (0, 0, 0)
    moveHead(0.0, 0.0, 0.0);
    moveHead(0.01, 0.0, 0.0);
    EXPECT_TRUE(head_targets_.empty());
}

TEST_F(XrInputHandlerTest, theHeadStopsWhenTheHeadTaskLeavesTrackingAndStartsOverWhenItIsBack)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    moveHead(0.02, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());

    headTaskIs(WholeBodyState::HEAD_FORWARD);
    head_targets_.clear();
    moveHead(0.04, 0.0, 0.0);
    EXPECT_TRUE(head_targets_.empty());

    // Back to tracking with the robot head somewhere else and the headset far from where it was: the old base is gone.
    robotHeadAt(0.0, 0.0, 1.6);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    moveHead(0.5, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());
    EXPECT_NEAR(head_targets_.back().position.x, 0.0, 1e-6) << "the head used the base of the earlier session";
    EXPECT_NEAR(head_targets_.back().position.z, 1.6, 1e-6);
}

TEST_F(XrInputHandlerTest, theHeadStopsWhenTheArmsStopFollowingAndWhenOcs2IsLeft)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());

    press(kToggleFollow);  // STORAGE
    head_targets_.clear();
    moveHead(0.02, 0.0, 0.0);
    EXPECT_TRUE(head_targets_.empty()) << "the head followed after the arms stopped";

    moveLeftController(0.0, 0.0, 0.0);  // a fresh controller pose for the second start
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());
    fsmIs(kHold);
    head_targets_.clear();
    moveHead(0.02, 0.0, 0.0);
    EXPECT_TRUE(head_targets_.empty()) << "the head followed outside OCS2";
}

TEST_F(XrInputHandlerTest, aHeadPoseThatIsNotFiniteIsDroppedAndTheNextGoodOneIsKept)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    const auto before = head_targets_.size();

    moveHead(std::nan(""), 0.0, 0.0);
    Pose zero_quaternion = pose(0.0, 0.0, 0.0);
    zero_quaternion.orientation.w = 0.0;
    moveHead(zero_quaternion);
    EXPECT_EQ(head_targets_.size(), before);

    moveHead(0.02, 0.0, 0.0);
    ASSERT_GT(head_targets_.size(), before);
    EXPECT_NEAR(head_targets_.back().position.x, 0.02, 1e-6);
}

// The same checks as for the arms: a big step from the previous head target is dropped, the next small one is kept.
TEST_F(XrInputHandlerTest, aBigStepOfTheHeadIsDroppedAndTheNextSmallOneIsKept)
{
    fsmIs(kOcs2);
    headTaskIs(WholeBodyState::HEAD_TRACKING);
    robotHeadAt(0.0, 0.0, 1.5);
    press(kToggleFollow);
    moveHead(0.0, 0.0, 0.0);
    ASSERT_FALSE(head_targets_.empty());

    moveHead(0.2, 0.0, 0.0);  // 20 cm in one frame, within reach of the head but a jump
    for (const auto& target : head_targets_)
    {
        EXPECT_LT(target.position.x, 0.1) << "a 20 cm step was sent";
    }

    moveHead(0.04, 0.0, 0.0);
    EXPECT_NEAR(head_targets_.back().position.x, 0.04, 1e-6);
}

// Chassis drive: the left stick key makes the sticks drive the base (left stick: forward / sideways, right stick:
// turn). Only in OCS2, only while the whole body controller keeps the base locked (it publishes the same velocity topic
// itself when the base is unlocked), only with fresh stick input. Anything else that stops being true stops the base.
namespace
{
    using WholeBodyState = robot_control_msgs::msg::WholeBodyCurrentState;
}

TEST_F(XrInputHandlerTest, theSticksDoNothingUntilChassisDriveIsOn)
{
    fsmIs(kOcs2);
    sticks(0.0, -1.0, 1.0, 0.0);
    EXPECT_TRUE(base_velocities_.empty());
}

TEST_F(XrInputHandlerTest, chassisDriveIsRefusedOutsideOcs2)
{
    fsmIs(kHold);
    sticks(0.0, 0.0, 0.0, 0.0);
    press(kToggleChassisDrive);
    sticks(0.0, -1.0, 0.0, 0.0);
    EXPECT_TRUE(base_velocities_.empty()) << "drove the base outside OCS2";
}

TEST_F(XrInputHandlerTest, chassisDriveIsRefusedWithoutFreshStickInput)
{
    fsmIs(kOcs2);
    spinFor(800ms);  // no stick frame for longer than xr_chassis.stale_ms
    press(kToggleChassisDrive);
    sticks(0.0, -1.0, 0.0, 0.0);
    EXPECT_TRUE(base_velocities_.empty());
}

TEST_F(XrInputHandlerTest, theSticksDriveTheBaseInTheRobotsAxes)
{
    startChassisDrive();

    sticks(0.0, -1.0, 0.0, 0.0);  // left stick forward (negative y): forward
    EXPECT_NEAR(lastBaseVelocity().linear.x, kLinearScale, 1e-9);
    EXPECT_NEAR(lastBaseVelocity().linear.y, 0.0, 1e-9);

    sticks(1.0, 0.0, 0.0, 0.0);  // left stick right: the robot's right is negative y
    EXPECT_NEAR(lastBaseVelocity().linear.x, 0.0, 1e-9);
    EXPECT_NEAR(lastBaseVelocity().linear.y, -kLinearScale, 1e-9);

    sticks(0.0, 0.0, 1.0, 0.0);  // right stick right: turn clockwise, negative about +Z
    EXPECT_NEAR(lastBaseVelocity().angular.z, -kAngularScale, 1e-9);
    EXPECT_NEAR(lastBaseVelocity().linear.x, 0.0, 1e-9);

    sticks(0.0, 0.5, -0.5, 0.0);  // half way back, half way to the left
    EXPECT_NEAR(lastBaseVelocity().linear.x, -0.5 * kLinearScale, 1e-9);
    EXPECT_NEAR(lastBaseVelocity().angular.z, 0.5 * kAngularScale, 1e-9);
}

TEST_F(XrInputHandlerTest, centeredSticksSendZeroSoTheBaseDoesNotKeepMoving)
{
    startChassisDrive();
    sticks(0.0, -1.0, 0.0, 0.0);
    ASSERT_FALSE(isStopped(lastBaseVelocity()));
    sticks(0.0, 0.0, 0.0, 0.0);
    EXPECT_TRUE(isStopped(lastBaseVelocity()));
}

TEST_F(XrInputHandlerTest, stickValuesAreClampedAndNonFiniteValuesStopTheBase)
{
    startChassisDrive();
    sticks(0.0, -5.0, 0.0, 0.0);
    EXPECT_NEAR(lastBaseVelocity().linear.x, kLinearScale, 1e-9) << "a stick value above 1 went through";

    sticks(std::nan(""), -1.0, 0.0, 0.0);
    EXPECT_TRUE(isStopped(lastBaseVelocity())) << "a non-finite stick value moved the base";
}

TEST_F(XrInputHandlerTest, theLeftStickKeyAgainStopsTheBaseAndTheSticksAreIgnoredAfterwards)
{
    startChassisDrive();
    sticks(0.0, -1.0, 0.0, 0.0);
    ASSERT_FALSE(isStopped(lastBaseVelocity()));

    press(kToggleChassisDrive);
    EXPECT_TRUE(isStopped(lastBaseVelocity())) << "no zero velocity when leaving chassis drive";
    const auto sent = base_velocities_.size();
    sticks(0.0, -1.0, 0.0, 0.0);
    EXPECT_EQ(base_velocities_.size(), sent);
}

TEST_F(XrInputHandlerTest, leavingOcs2StopsTheBase)
{
    startChassisDrive();
    sticks(0.0, -1.0, 0.0, 0.0);
    ASSERT_FALSE(isStopped(lastBaseVelocity()));

    fsmIs(kHold);
    EXPECT_TRUE(isStopped(lastBaseVelocity()));
    const auto sent = base_velocities_.size();
    sticks(0.0, -1.0, 0.0, 0.0);
    EXPECT_EQ(base_velocities_.size(), sent) << "kept driving after OCS2 was left";

    fsmIs(kOcs2);  // back in OCS2: the key has to be pressed again
    sticks(0.0, -1.0, 0.0, 0.0);
    EXPECT_EQ(base_velocities_.size(), sent);
}

// The headset drops out or the bridge dies: no more stick frames, so no zero frame either. The base must not keep the
// last velocity.
TEST_F(XrInputHandlerTest, noStickInputForTooLongStopsTheBase)
{
    startChassisDrive();
    sticks(0.0, -1.0, 0.0, 0.0);
    ASSERT_FALSE(isStopped(lastBaseVelocity()));

    spinFor(900ms);
    EXPECT_TRUE(isStopped(lastBaseVelocity())) << "the base kept the last velocity without stick input";
    const auto sent = base_velocities_.size();
    sticks(0.0, -1.0, 0.0, 0.0);  // input is back: chassis drive is off until the key is pressed again
    EXPECT_EQ(base_velocities_.size(), sent);
}

TEST_F(XrInputHandlerTest, enteringChassisDriveStopsTheArmsFromFollowing)
{
    fsmIs(kOcs2);
    moveLeftController(0.0, 0.0, 0.0);
    press(kToggleFollow);
    moveLeftController(0.03, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());

    sticks(0.0, 0.0, 0.0, 0.0);
    press(kToggleChassisDrive);
    left_targets_.clear();
    moveLeftController(0.05, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty()) << "the arm followed while the sticks drive the base";
}

TEST_F(XrInputHandlerTest, theRightStickKeyDoesNotStartFollowingWhileDrivingTheBase)
{
    startChassisDrive();
    moveLeftController(0.0, 0.0, 0.0);
    press(kToggleFollow);
    moveLeftController(0.03, 0.0, 0.0);
    EXPECT_TRUE(left_targets_.empty()) << "arm following and base driving at the same time";
}

// Chassis drive is opt in: with xr_base left at its default (false) the handler has no publisher on the base
// velocity topic at all, and the left stick key does nothing (an accidental click must not stop the arms or drive the base).
namespace
{
    class XrInputHandlerChassisOffTest : public XrInputHandlerTest
    {
    protected:
        bool chassisDriveEnabled() const override { return false; }
    };
}

TEST_F(XrInputHandlerChassisOffTest, thereIsNoPublisherOnTheBaseVelocityTopic)
{
    spinFor(300ms);  // discovery
    EXPECT_EQ(probe_->count_publishers("base_velocity_target"), 0u);
}

TEST_F(XrInputHandlerChassisOffTest, theLeftStickKeyDoesNothingAndTheArmsKeepFollowing)
{
    fsmIs(kOcs2);
    moveLeftController(0.0, 0.0, 0.0);
    press(kToggleFollow);
    moveLeftController(0.03, 0.0, 0.0);
    ASSERT_FALSE(left_targets_.empty());

    sticks(0.0, 0.0, 0.0, 0.0);
    press(kToggleChassisDrive);
    sticks(0.0, -1.0, 1.0, 0.0);
    EXPECT_TRUE(base_velocities_.empty()) << "the base was driven with xr_base = false";

    left_targets_.clear();
    moveLeftController(0.05, 0.0, 0.0);
    EXPECT_FALSE(left_targets_.empty()) << "the left stick key stopped the arms with chassis drive off";
}
