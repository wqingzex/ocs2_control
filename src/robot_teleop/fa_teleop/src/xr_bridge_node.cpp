#include "fa_teleop/xr_bridge_node.hpp"
#include <robot_log/log_mode.hpp>
#include <cstdio>

#include <rclcpp_components/register_node_macro.hpp>

namespace fa_teleop
{
    namespace
    {
        geometry_msgs::msg::Pose toMsg(const Eigen::Isometry3d& pose)
        {
            const Pose7 p = XrPoseConverter::toPose7(pose);
            geometry_msgs::msg::Pose msg;
            msg.position.x = p[0];
            msg.position.y = p[1];
            msg.position.z = p[2];
            msg.orientation.x = p[3];
            msg.orientation.y = p[4];
            msg.orientation.z = p[5];
            msg.orientation.w = p[6];
            return msg;
        }

        const char* eventName(XrSdkClient::Event event)
        {
            switch (event)
            {
            case XrSdkClient::Event::kServerConnected: return "PC Service 已连接";
            case XrSdkClient::Event::kServerDisconnected: return "PC Service 已断开";
            case XrSdkClient::Event::kDeviceFound: return "发现头显";
            case XrSdkClient::Event::kDeviceMissing: return "头显离线";
            case XrSdkClient::Event::kDeviceConnected: return "头显已连接";
            }
            return "未知事件";
        }
    } // namespace

    // Node name kept as "xr_target_node": VRInputHandler enables VR control only while a node with
    // this exact name exists (VRInputHandler::XR_NODE_NAME).
    XrBridgeNode::XrBridgeNode(const rclcpp::NodeOptions& options) : Node("xr_target_node", options)
    {
        robot_log::applyLogMode();  // debug mode of this run, applied once
        poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 200.0);
        frame_timeout_ = std::chrono::milliseconds(declare_parameter<int>("frame_timeout_ms", 300));
        reconnect_interval_ = std::chrono::duration<double>(declare_parameter<double>("reconnect_interval_s", 3.0));
        const int depth = declare_parameter<int>("publish_queue_depth", 50);
        if (poll_rate_hz_ <= 0.0 || frame_timeout_.count() <= 0 || depth <= 0)
        {
            throw std::invalid_argument("xr_bridge: poll_rate_hz, frame_timeout_ms and publish_queue_depth must be > 0");
        }
        const std::string scheme = declare_parameter<std::string>("control_scheme", "legacy");
        if (scheme != "legacy" && scheme != "simple")
        {
            throw std::invalid_argument("xr_bridge: control_scheme must be 'legacy' or 'simple', got '" + scheme + "'");
        }
        simple_scheme_ = scheme == "simple";
        const auto target_id = declare_parameter<std::string>("device_id", "");
        device_selector_ = XrDeviceSelector(target_id);

        pub_head_ = create_publisher<geometry_msgs::msg::Pose>("/teleop/head_pose", depth);
        pub_left_ = create_publisher<geometry_msgs::msg::Pose>("/teleop/left_ee_pose", depth);
        pub_right_ = create_publisher<geometry_msgs::msg::Pose>("/teleop/right_ee_pose", depth);
        pub_event_ = create_publisher<std_msgs::msg::Int32>("/teleop/controller_state", depth);
        pub_thumbstick_ = create_publisher<geometry_msgs::msg::Twist>("/teleop/thumbstick_axes", depth);
        pub_trigger_ = create_publisher<geometry_msgs::msg::Twist>("/teleop/trigger_values", depth);
        pub_grip_ = create_publisher<geometry_msgs::msg::Twist>("/teleop/grip_values", depth);
        pub_connection_ = create_publisher<std_msgs::msg::Bool>(
            "/teleop/connection_status", rclcpp::QoS(1).reliable().transient_local());
        pub_device_id_ = create_publisher<std_msgs::msg::String>(
            "/teleop/device_id", rclcpp::QoS(1).reliable().transient_local());

        flight_ = robot_log::FlightRecorder::create("xr_bridge");
        if (!flight_) { RCLCPP_WARN(get_logger(), "无法创建飞行记录环，本模块不记录"); }

        startSdk();
        disconnected_since_ = std::chrono::steady_clock::now();

        const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { onTimer(); });

        // One health line per window: how many frames came in and went out, the longest silence between two frames.
        gap_warn_ms_ = declare_parameter<double>("health_gap_warn_ms", 100.0);
        health_window_s_ = robot_log::healthWindowSeconds();
        health_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(health_window_s_)),
            [this]() { reportHealth(); });

        RCLCPP_INFO(get_logger(), "XR Bridge 已启动：XRoboToolkit → /xr/*（按头显帧发布，轮询 %.0f Hz）",
                    poll_rate_hz_);
        RCLCPP_INFO(get_logger(), "断线判定：%ld ms 无新帧；PC Service 断开 %.1f s 后自动重连",
                    static_cast<long>(frame_timeout_.count()), reconnect_interval_.count());
        RCLCPP_INFO(get_logger(), "头显选择：%s", target_id.empty() ? "等待首台发送追踪数据的头显" : target_id.c_str());
        RCLCPP_INFO(get_logger(), "按键方案: %s", simple_scheme_
            ? "simple（A/X 状态机，Y+B 15，Y+A 54，B 录制开始/停止，长按 Y 删除，右摇杆 干预）"
            : "legacy（与原 xr_target_node 一致）");
        std_msgs::msg::Bool status;
        status.data = false;
        pub_connection_->publish(status);
    }

    XrBridgeNode::~XrBridgeNode()
    {
        timer_.reset();
        sdk_.stop();
    }

    void XrBridgeNode::startSdk()
    {
        std::string error;
        const bool ok = sdk_.start(
            [this](const XrFrame& frame) { onFrame(frame); },
            [this](XrSdkClient::Event event, const std::string& detail) { onSdkEvent(event, detail); },
            [this](const std::string& err)
            {
                if (parse_errors_.fetch_add(1) % 1000 == 0)
                {
                    RCLCPP_WARN(get_logger(), "[XR-004] 头显数据解析失败（已累计 %lu 次）：%s",
                                static_cast<unsigned long>(parse_errors_.load()), err.c_str());
                }
            },
            &error);
        if (!ok)
        {
            RCLCPP_ERROR(get_logger(), "[XR-001] 连接 XRoboToolkit PC Service 失败：%s（将定期重试）", error.c_str());
        }
    }

    void XrBridgeNode::onFrame(const XrFrame& frame)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (frame.timestamp_ns == 0 || !(frame.has_head || frame.has_left || frame.has_right)) { return; }
        if (!device_selector_.accept(frame.device_id)) { return; }
        pending_frame_ = frame;
        last_frame_time_ = std::chrono::steady_clock::now();
        received_stats_.record();
        recordFrameIn(frame);
    }

    void XrBridgeNode::recordFrameIn(const XrFrame& frame)
    {
        if (!flight_) { return; }
        double v[26];
        int n = 0;
        for (const Pose7* pose : {&frame.head, &frame.left.pose, &frame.right.pose})
        {
            for (double x : *pose) { v[n++] = x; }
        }
        v[n++] = frame.left.trigger;
        v[n++] = frame.left.grip;
        v[n++] = frame.right.trigger;
        v[n++] = frame.right.grip;
        v[n++] = static_cast<double>(frame.timestamp_ns) * 1e-9;
        const std::uint32_t has = (frame.has_head ? 1u : 0u) | (frame.has_left ? 2u : 0u) | (frame.has_right ? 4u : 0u);
        flight_->record(robot_log::flight::kXrFrameIn, v, static_cast<std::size_t>(n), has);
    }

    void XrBridgeNode::onSdkEvent(XrSdkClient::Event event, const std::string& detail)
    {
        if (event == XrSdkClient::Event::kServerConnected) { server_connected_ = true; }
        if (event == XrSdkClient::Event::kServerDisconnected) { server_connected_ = false; }
        RCLCPP_INFO(get_logger(), "SDK 事件: %s %s", eventName(event), detail.c_str());
    }

    void XrBridgeNode::onTimer()
    {
        const auto now = std::chrono::steady_clock::now();
        std::optional<XrFrame> frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            frame.swap(pending_frame_);
        }
        // Frame gate: publish once per new headset frame (timestamp 0 = no data yet).
        if (frame && frame->timestamp_ns != 0 && frame->timestamp_ns != last_timestamp_ns_)
        {
            last_timestamp_ns_ = frame->timestamp_ns;
            publishFrame(*frame);
        }
        updateConnection(now);

        // Re-initialise the SDK when the PC Service link stays down.
        if (!connected_ && !server_connected_ && now - disconnected_since_ >= reconnect_interval_)
        {
            RCLCPP_WARN(get_logger(), "[XR-002] PC Service 未连接，重新初始化 XRoboToolkit SDK ...");
            sdk_.stop();
            startSdk();
            disconnected_since_ = now;
        }
    }

    void XrBridgeNode::publishFrame(const XrFrame& frame)
    {
        if (frame.device_id != published_device_id_)
        {
            std_msgs::msg::String id;
            id.data = frame.device_id;
            pub_device_id_->publish(id);
            published_device_id_ = frame.device_id;
            RCLCPP_INFO(get_logger(), "接收头显追踪数据，设备 ID: %s", id.data.c_str());
        }
        const auto& poses = pose_converter_.update(frame);
        const TeleState tele = toTeleState(frame.left, frame.right);
        const EncodedControls controls =
            simple_scheme_ ? simple_encoder_.update(tele, frame.timestamp_ns) : encoder_.update(tele);

        pub_head_->publish(toMsg(poses.head));
        pub_left_->publish(toMsg(poses.left));
        pub_right_->publish(toMsg(poses.right));

        std_msgs::msg::Int32 event;
        event.data = controls.event;
        pub_event_->publish(event);

        geometry_msgs::msg::Twist thumbstick;
        thumbstick.linear.x = controls.thumbstick[0];
        thumbstick.linear.y = controls.thumbstick[1];
        thumbstick.linear.z = controls.thumbstick[2];
        thumbstick.angular.x = controls.thumbstick[3];
        thumbstick.angular.y = controls.thumbstick[4];
        thumbstick.angular.z = controls.thumbstick[5];
        pub_thumbstick_->publish(thumbstick);

        geometry_msgs::msg::Twist trigger;
        trigger.linear.x = controls.left_trigger_pull;
        trigger.angular.x = controls.right_trigger_pull;
        pub_trigger_->publish(trigger);

        geometry_msgs::msg::Twist grip;
        grip.linear.x = controls.left_grip_value;
        grip.angular.x = controls.right_grip_value;
        pub_grip_->publish(grip);

        if (controls.event != event::kNone)
        {
            RCLCPP_DEBUG(get_logger(), "按键事件 %d", controls.event);
        }
        ++published_frames_;
        published_stats_.record();
        if (flight_)
        {
            double age_ms = 0.0;
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                age_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_frame_time_).count();
            }
            flight_->record(robot_log::flight::kXrPublish, {age_ms, static_cast<double>(controls.event)});
        }
    }

    void XrBridgeNode::reportHealth()
    {
        const auto received = received_stats_.take(health_window_s_);
        const auto published = published_stats_.take(health_window_s_);
        const uint64_t parse_errors = parse_errors_.load();
        const uint64_t new_parse_errors = parse_errors - last_reported_parse_errors_;
        last_reported_parse_errors_ = parse_errors;

        char extra[160];
        std::snprintf(extra, sizeof(extra), "received_hz=%.1f received_gap_max_ms=%.1f parse_fail=%llu connected=%d",
                      received.rate_hz, received.gap_max_ms, static_cast<unsigned long long>(new_parse_errors),
                      connected_ ? 1 : 0);
        const std::string line = robot_log::formatRateLine(get_name(), health_window_s_, published, extra);
        // The headset sent frames, but with a hole in them longer than health_gap_warn_ms (shorter than frame_timeout_ms,
        // so it never became a disconnect): that is what makes the arm stutter.
        if (received.count > 0 && received.gap_max_ms > gap_warn_ms_)
        {
            RCLCPP_WARN(get_logger(), "[XR-006] %s", line.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "%s", line.c_str());
        }
    }

    void XrBridgeNode::updateConnection(std::chrono::steady_clock::time_point now)
    {
        std::chrono::steady_clock::time_point last;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            last = last_frame_time_;
        }
        const bool connected = last.time_since_epoch().count() != 0 && now - last < frame_timeout_;
        if (connected != connected_)
        {
            connected_ = connected;
            if (connected)
            {
                RCLCPP_INFO(get_logger(), published_frames_ > 0 ? "头显数据恢复，继续发布 /xr/*"
                                                                : "头显已连接，开始发布 /xr/*");
            }
            else
            {
                disconnected_since_ = now;
                RCLCPP_WARN(get_logger(), "[XR-003] %ld ms 未收到头显数据：停止发布，机械臂保持最后目标",
                            static_cast<long>(frame_timeout_.count()));
            }
            std_msgs::msg::Bool status;
            status.data = connected;
            pub_connection_->publish(status);
            last_status_publish_ = now;
        }
        else if (now - last_status_publish_ >= std::chrono::seconds(1))
        {
            std_msgs::msg::Bool status;
            status.data = connected_;
            pub_connection_->publish(status);
            last_status_publish_ = now;
        }
    }
} // namespace fa_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(fa_teleop::XrBridgeNode)
