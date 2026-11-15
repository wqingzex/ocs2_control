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

#include "teleop_command/XrInputHandler.h"

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
            node_ = std::make_shared<rclcpp::Node>("teleop_command_under_test");
            bridge_ = std::make_shared<rclcpp::Node>("xr_target_node");  // XR is enabled only while it exists
            probe_ = std::make_shared<rclcpp::Node>("xr_input_handler_probe");

            auto left_target = node_->create_publisher<Pose>("left_target", 10);
            auto right_target = node_->create_publisher<Pose>("right_target", 10);
            handler_ = std::make_unique<teleop_command::XrInputHandler>(
                node_, nullptr, left_target, right_target,
                std::vector<std::string>{"left_hand_controller", "right_hand_controller"},
                1.0, "base_link");

            event_pub_ = probe_->create_publisher<std_msgs::msg::Int32>("/teleop/controller_state", 10);
            left_pose_pub_ = probe_->create_publisher<Pose>("/teleop/left_ee_pose", 10);
            trigger_pub_ = probe_->create_publisher<geometry_msgs::msg::Twist>("/teleop/trigger_values", 10);
            fsm_sub_ = probe_->create_subscription<std_msgs::msg::Int32>(
                "/fsm_command", 10, [this](std_msgs::msg::Int32::SharedPtr m) { fsm_.push_back(m->data); });
            target_sub_ = probe_->create_subscription<Pose>(
                "left_target", 10, [this](Pose::SharedPtr m) { left_targets_.push_back(*m); });
            left_hand_sub_ = probe_->create_subscription<std_msgs::msg::Float64>(
                "/left_hand_controller/target_percent", 10,
                [this](std_msgs::msg::Float64::SharedPtr m) { left_hand_ = m->data; });
            right_hand_sub_ = probe_->create_subscription<std_msgs::msg::Float64>(
                "/right_hand_controller/target_percent", 10,
                [this](std_msgs::msg::Float64::SharedPtr m) { right_hand_ = m->data; });

            executor_.add_node(node_);
            executor_.add_node(bridge_);
            executor_.add_node(probe_);
            waitForSubscribers();
            // The handler enables XR on a left pose once /xr_target_node is visible in the graph.
            ASSERT_TRUE(spinUntil([this]() {
                left_pose_pub_->publish(pose(0.0, 0.0, 0.0));
                return handler_->isEnabled();
            })) << "XR control never enabled";
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

        /// What teleop_command_node does with every /fsm_command it sees.
        void fsmIs(int state)
        {
            auto m = std::make_shared<std_msgs::msg::Int32>();
            m->data = state;
            handler_->fsmCommandCallback(m);
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

        /// The first command sent after a key press (ignores the 0 reset after a REST pulse).
        std::optional<int> sentFsm() const
        {
            for (int c : fsm_) { if (c != 0) { return c; } }
            return std::nullopt;
        }

        rclcpp::Node::SharedPtr node_, bridge_, probe_;
        std::unique_ptr<teleop_command::XrInputHandler> handler_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr event_pub_;
        rclcpp::Publisher<Pose>::SharedPtr left_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr trigger_pub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_sub_;
        rclcpp::Subscription<Pose>::SharedPtr target_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_hand_sub_, right_hand_sub_;
        std::vector<int> fsm_;
        std::vector<Pose> left_targets_;
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
