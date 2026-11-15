#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <robot_log/flight.hpp>
#include <robot_log/flight_types.hpp>
#include <robot_log/health.hpp>

#include "xr_teleop/button_event_encoder.hpp"
#include "xr_teleop/xr_pose_converter.hpp"
#include "xr_teleop/xr_sdk_client.hpp"
#include "xr_teleop/xr_device_selector.hpp"

namespace xr_teleop
{
    /// XRoboToolkit (Pico App + PC Service) -> /xr/* topics consumed by XrInputHandler (list: docs).
    /// One message set per new headset frame; none for frame_timeout_ms -> connection_status false.
    /// The SDK is re-initialised after the PC Service link is down for reconnect_interval_s.
    class XrBridgeNode : public rclcpp::Node
    {
    public:
        explicit XrBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~XrBridgeNode() override;

    private:
        void startSdk();
        void onFrame(const XrFrame& frame);
        void onSdkEvent(XrSdkClient::Event event, const std::string& detail);
        void onTimer();
        void publishFrame(const XrFrame& frame);
        void updateConnection(std::chrono::steady_clock::time_point now);

        // Parameters
        double poll_rate_hz_ = 200.0;
        std::chrono::milliseconds frame_timeout_{300};
        std::chrono::duration<double> reconnect_interval_{3.0};

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_event_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_thumbstick_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_trigger_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_grip_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connection_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_device_id_;
        rclcpp::TimerBase::SharedPtr timer_;

        // SDK thread -> timer thread hand-off
        std::mutex frame_mutex_;
        std::optional<XrFrame> pending_frame_;
        XrDeviceSelector device_selector_;
        std::chrono::steady_clock::time_point last_frame_time_{};
        std::atomic<bool> server_connected_{false};
        std::atomic<uint64_t> parse_errors_{0};

        // Health of this hop (src/robot_log/README.md): frames accepted from the headset (SDK thread) and frames published,
        // reported once per window by health_timer_.
        void reportHealth();
        void recordFrameIn(const XrFrame& frame);
        // What came in and went out, kept in shared memory for the incident recorder (nullptr: no ring, no recording).
        std::shared_ptr<robot_log::FlightRecorder> flight_;
        robot_log::RateStats received_stats_;
        robot_log::RateStats published_stats_;
        uint64_t last_reported_parse_errors_ = 0;
        double health_window_s_ = 10.0;
        double gap_warn_ms_ = 100.0;
        rclcpp::TimerBase::SharedPtr health_timer_;

        // Timer-thread state
        XrSdkClient sdk_;
        XrPoseConverter pose_converter_;
        ButtonEventEncoder encoder_;
        int64_t last_timestamp_ns_ = 0;
        bool connected_ = false;
        uint64_t published_frames_ = 0;
        std::string published_device_id_;
        std::chrono::steady_clock::time_point disconnected_since_{};
        std::chrono::steady_clock::time_point last_status_publish_{};
    };
} // namespace xr_teleop
