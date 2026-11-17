#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "dexcap_teleop/trajectory_filter.hpp"

namespace
{

constexpr std::size_t kSourceJointCount = 9;
constexpr std::size_t kArmJointCount = 7;
constexpr std::size_t kCommandJointCount = 14;
constexpr double kHalfPi = std::numbers::pi_v<double> / 2.0;
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

using SourceArm = std::array<double, kSourceJointCount>;
using RobotArm = std::array<double, kArmJointCount>;
using RobotCommand = std::array<double, kCommandJointCount>;
using SteadyTime = std::chrono::steady_clock::time_point;

std::vector<double> repeated_limits(const std::array<double, 7> &limits)
{
    std::vector<double> result;
    result.reserve(14);
    result.insert(result.end(), limits.begin(), limits.end());
    result.insert(result.end(), limits.begin(), limits.end());
    return result;
}

}  // namespace

class TeleopJointMapperNode final : public rclcpp::Node
{
public:
    TeleopJointMapperNode()
        : Node("arm_joint_mapper")
    {
        source_ = declare_parameter<std::string>("source", "dexcap");
        robot_ = declare_parameter<std::string>("robot", "tianji");
        interpolation_type_ =
            declare_parameter<std::string>("interpolation_type", "online");
        left_topic_ = declare_parameter<std::string>(
            "left_arm_topic", "/dexcap/left_arm");
        right_topic_ = declare_parameter<std::string>(
            "right_arm_topic", "/dexcap/right_arm");
        robot_joint_states_topic_ = declare_parameter<std::string>(
            "robot_joint_states_topic", "/joint_states");
        output_topic_ = declare_parameter<std::string>(
            "output_topic",
            "/arm_controller/joint_target_position");
        fsm_state_topic_ = declare_parameter<std::string>(
            "fsm_state_topic", "/fsm_state");
        movej_state_ = declare_parameter<int>("movej_state", 4);
        publish_rate_ = std::clamp(
            declare_parameter<double>("publish_rate", 1000.0),
            1.0, 1000.0);
        input_timeout_ = std::max(
            declare_parameter<double>("input_timeout", 0.2), 0.01);
        require_robot_state_ =
            declare_parameter<bool>("require_robot_state", true);

        trajectory_limits_.max_velocity =
            declare_parameter<double>("online_max_velocity", 3.0);
        trajectory_limits_.max_acceleration =
            declare_parameter<double>("online_max_acceleration", 30.0);
        trajectory_limits_.max_jerk =
            declare_parameter<double>("online_max_jerk", 300.0);
        trajectory_limits_.tracking_frequency =
            declare_parameter<double>("online_tracking_frequency", 20.0);
        trajectory_limits_.target_filter_alpha =
            declare_parameter<double>("online_target_filter_alpha", 0.8);
        trajectory_limits_.position_tolerance =
            declare_parameter<double>("online_position_tolerance", 1.0e-4);
        trajectory_limits_.velocity_tolerance =
            declare_parameter<double>("online_velocity_tolerance", 1.0e-3);
        trajectory_limits_.acceleration_tolerance =
            declare_parameter<double>("online_acceleration_tolerance", 1.0e-2);

        load_robot_profile();
        for (auto &filter : filters_)
        {
            filter = dexcap_teleop::make_trajectory_filter(
                interpolation_type_, trajectory_limits_);
        }

        publisher_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                output_topic_, 10);
        left_subscription_ =
            create_subscription<sensor_msgs::msg::JointState>(
                left_topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::SharedPtr message)
                {
                    source_callback(message, false);
                });
        right_subscription_ =
            create_subscription<sensor_msgs::msg::JointState>(
                right_topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::SharedPtr message)
                {
                    source_callback(message, true);
                });
        robot_state_subscription_ =
            create_subscription<sensor_msgs::msg::JointState>(
                robot_joint_states_topic_, rclcpp::SensorDataQoS(),
                std::bind(
                    &TeleopJointMapperNode::robot_state_callback, this,
                    std::placeholders::_1));
        fsm_subscription_ = create_subscription<std_msgs::msg::Int32>(
            fsm_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &TeleopJointMapperNode::fsm_callback, this,
                std::placeholders::_1));

        const auto period =
            std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TeleopJointMapperNode::publish_command, this));

        RCLCPP_INFO(
            get_logger(),
            "Joint mapper ready: source=%s, robot=%s, interpolation=%s, "
            "input=[%s, %s], output=%s at %.1f Hz; MOVEJ=%d",
            source_.c_str(), robot_.c_str(), interpolation_type_.c_str(),
            left_topic_.c_str(), right_topic_.c_str(),
            output_topic_.c_str(), publish_rate_, movej_state_);
    }

private:
    void load_robot_profile()
    {
        const std::string prefix = "profiles." + robot_ + ".";
        left_sign_ = to_array<7>(declare_parameter<std::vector<double>>(
            prefix + "left_sign", std::vector<double>{}),
            prefix + "left_sign");
        right_sign_ = to_array<7>(declare_parameter<std::vector<double>>(
            prefix + "right_sign", std::vector<double>{}),
            prefix + "right_sign");
        left_offset_k_ = to_array<7>(declare_parameter<std::vector<double>>(
            prefix + "left_offset_k", std::vector<double>{}),
            prefix + "left_offset_k");
        right_offset_k_ = to_array<7>(declare_parameter<std::vector<double>>(
            prefix + "right_offset_k", std::vector<double>{}),
            prefix + "right_offset_k");

        constexpr std::array<double, 7> default_lower = {
            -178.0 * std::numbers::pi_v<double> / 180.0,
            -118.0 * std::numbers::pi_v<double> / 180.0,
            -168.0 * std::numbers::pi_v<double> / 180.0,
            -143.0 * std::numbers::pi_v<double> / 180.0,
            -168.0 * std::numbers::pi_v<double> / 180.0,
             -58.0 * std::numbers::pi_v<double> / 180.0,
             -78.0 * std::numbers::pi_v<double> / 180.0};
        constexpr std::array<double, 7> default_upper = {
             178.0 * std::numbers::pi_v<double> / 180.0,
             118.0 * std::numbers::pi_v<double> / 180.0,
             168.0 * std::numbers::pi_v<double> / 180.0,
              58.0 * std::numbers::pi_v<double> / 180.0,
             168.0 * std::numbers::pi_v<double> / 180.0,
              58.0 * std::numbers::pi_v<double> / 180.0,
              78.0 * std::numbers::pi_v<double> / 180.0};
        lower_limits_ = to_array<14>(
            declare_parameter<std::vector<double>>(
                prefix + "lower_limits", repeated_limits(default_lower)),
            prefix + "lower_limits");
        upper_limits_ = to_array<14>(
            declare_parameter<std::vector<double>>(
                prefix + "upper_limits", repeated_limits(default_upper)),
            prefix + "upper_limits");
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            if (lower_limits_[index] >= upper_limits_[index])
            {
                throw std::invalid_argument(
                    "Every lower limit must be smaller than its upper limit");
            }
        }
    }

    template<std::size_t Size>
    static std::array<double, Size> to_array(
        const std::vector<double> &values, const std::string &name)
    {
        if (values.size() != Size)
        {
            throw std::invalid_argument(
                name + " must contain " + std::to_string(Size) + " values");
        }
        std::array<double, Size> result{};
        std::copy(values.begin(), values.end(), result.begin());
        return result;
    }

    bool read_source_arm(
        const sensor_msgs::msg::JointState &message,
        const bool right, SourceArm &result) const
    {
        std::unordered_map<std::string, std::size_t> indices;
        for (std::size_t index = 0; index < message.name.size(); ++index)
        {
            indices.emplace(message.name[index], index);
        }
        const std::string prefix =
            right ? "right_arm_joint_" : "left_arm_joint_";
        for (std::size_t index = 0; index < result.size(); ++index)
        {
            const auto found =
                indices.find(prefix + std::to_string(index + 1));
            if (found == indices.end() ||
                found->second >= message.position.size() ||
                !std::isfinite(message.position[found->second]))
            {
                return false;
            }
            result[index] = message.position[found->second];
        }
        return true;
    }

    void source_callback(
        const sensor_msgs::msg::JointState::SharedPtr message,
        const bool right)
    {
        SourceArm arm{};
        if (!read_source_arm(*message, right, arm))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "%s input does not contain all nine finite source joints",
                right ? "Right" : "Left");
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (right)
        {
            right_source_ = arm;
            right_input_time_ = std::chrono::steady_clock::now();
            have_right_source_ = true;
        }
        else
        {
            left_source_ = arm;
            left_input_time_ = std::chrono::steady_clock::now();
            have_left_source_ = true;
        }
        update_mapped_target_locked();
    }

    RobotArm map_arm(
        const SourceArm &source, const std::array<double, 7> &sign,
        const std::array<double, 7> &offset_k) const
    {
        RobotArm mapped{};
        for (std::size_t index = 0; index < 6; ++index)
        {
            mapped[index] =
                sign[index] * source[index] + offset_k[index] * kHalfPi;
        }
        mapped[6] = sign[6] *
            (source[6] + source[7] + source[8]) +
            offset_k[6] * kHalfPi;
        return mapped;
    }

    std::optional<double> select_periodic_angle(
        const double value, const std::size_t index,
        const double reference) const
    {
        std::optional<double> selected;
        double best_distance = std::numeric_limits<double>::infinity();
        for (int turns = -2; turns <= 2; ++turns)
        {
            const double candidate =
                value + static_cast<double>(turns) * kTwoPi;
            if (candidate < lower_limits_[index] ||
                candidate > upper_limits_[index])
            {
                continue;
            }
            const double distance = std::abs(candidate - reference);
            if (distance < best_distance)
            {
                selected = candidate;
                best_distance = distance;
            }
        }
        return selected;
    }

    void update_mapped_target_locked()
    {
        if (!have_left_source_ || !have_right_source_)
        {
            return;
        }
        const auto left = map_arm(
            left_source_, left_sign_, left_offset_k_);
        const auto right = map_arm(
            right_source_, right_sign_, right_offset_k_);
        RobotCommand candidate{};
        bool any_invalid = false;
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            const double raw = index < 7
                ? left[index]
                : right[index - 7];
            const double reference = have_valid_target_
                ? valid_target_[index]
                : (have_robot_state_
                    ? robot_state_[index]
                    : 0.5 * (lower_limits_[index] + upper_limits_[index]));
            const auto selected =
                select_periodic_angle(raw, index, reference);
            if (!selected.has_value())
            {
                any_invalid = true;
                candidate[index] = have_valid_target_
                    ? valid_target_[index]
                    : 0.0;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Mapped joint %zu is out of range; retaining only that "
                    "joint at %.6f rad",
                    index + 1, candidate[index]);
                continue;
            }
            candidate[index] = *selected;
        }

        if (any_invalid && !have_valid_target_)
        {
            RCLCPP_WARN(
                get_logger(),
                "One or more joints in the first mapped target are out of "
                "range; only those joints use zero");
        }

        valid_target_ = candidate;
        have_valid_target_ = true;
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            filters_[index]->set_target(valid_target_[index]);
        }
    }

    void robot_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr message)
    {
        std::unordered_map<std::string, std::size_t> indices;
        for (std::size_t index = 0; index < message->name.size(); ++index)
        {
            indices.emplace(message->name[index], index);
        }
        RobotCommand state{};
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            const std::string name = index < 7
                ? "left_joint" + std::to_string(index + 1)
                : "right_joint" + std::to_string(index - 6);
            const auto found = indices.find(name);
            if (found == indices.end() ||
                found->second >= message->position.size() ||
                !std::isfinite(message->position[found->second]))
            {
                return;
            }
            state[index] = message->position[found->second];
        }
        std::lock_guard<std::mutex> lock(mutex_);
        robot_state_ = state;
        have_robot_state_ = true;
    }

    void fsm_callback(const std_msgs::msg::Int32::SharedPtr message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool was_movej = movej_active_;
        fsm_received_ = true;
        movej_active_ = message->data == movej_state_;
        if (movej_active_ && !was_movej)
        {
            filters_initialized_ = false;
            RCLCPP_INFO(
                get_logger(),
                "FSM entered MOVEJ (%d); command publishing enabled",
                movej_state_);
        }
        else if (!movej_active_ && was_movej)
        {
            RCLCPP_WARN(
                get_logger(),
                "FSM left MOVEJ; command publishing disabled");
        }
    }

    void initialize_filters_locked()
    {
        const RobotCommand start = have_robot_state_
            ? robot_state_
            : RobotCommand{};
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            filters_[index]->reset(start[index]);
            if (have_valid_target_)
            {
                filters_[index]->set_target(valid_target_[index]);
            }
            last_published_[index] = start[index];
        }
        filters_initialized_ = true;
        last_update_time_ = std::chrono::steady_clock::now();
    }

    void publish_command()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!fsm_received_ || !movej_active_ || !have_valid_target_)
        {
            return;
        }
        if (require_robot_state_ && !have_robot_state_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for the current 14-joint robot state");
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const bool stale =
            !have_left_source_ || !have_right_source_ ||
            std::chrono::duration<double>(now - left_input_time_).count() >
                input_timeout_ ||
            std::chrono::duration<double>(now - right_input_time_).count() >
                input_timeout_;
        if (stale)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Source arm data timed out; command publishing stopped");
            return;
        }
        if (!filters_initialized_)
        {
            initialize_filters_locked();
        }

        double dt =
            std::chrono::duration<double>(now - last_update_time_).count();
        dt = std::clamp(dt, 1.0e-5, 0.05);
        last_update_time_ = now;
        RobotCommand command{};
        for (std::size_t index = 0; index < kCommandJointCount; ++index)
        {
            command[index] = filters_[index]->update(dt);
            if (!std::isfinite(command[index]) ||
                command[index] < lower_limits_[index] ||
                command[index] > upper_limits_[index])
            {
                command[index] = last_published_[index];
                filters_[index]->reset(command[index]);
                filters_[index]->set_target(valid_target_[index]);
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Interpolated joint %zu exceeded its limit; retaining "
                    "only that joint at %.6f rad",
                    index + 1, command[index]);
            }
            last_published_[index] = command[index];
        }

        std_msgs::msg::Float64MultiArray message;
        message.data.assign(command.begin(), command.end());
        publisher_->publish(message);
    }

    std::mutex mutex_;
    std::string source_;
    std::string robot_;
    std::string interpolation_type_;
    std::string left_topic_;
    std::string right_topic_;
    std::string robot_joint_states_topic_;
    std::string output_topic_;
    std::string fsm_state_topic_;
    int movej_state_{4};
    double publish_rate_{1000.0};
    double input_timeout_{0.2};
    bool require_robot_state_{true};
    bool fsm_received_{false};
    bool movej_active_{false};
    bool have_left_source_{false};
    bool have_right_source_{false};
    bool have_robot_state_{false};
    bool have_valid_target_{false};
    bool filters_initialized_{false};
    SourceArm left_source_{};
    SourceArm right_source_{};
    RobotCommand robot_state_{};
    RobotCommand valid_target_{};
    RobotCommand last_published_{};
    std::array<double, 7> left_sign_{};
    std::array<double, 7> right_sign_{};
    std::array<double, 7> left_offset_k_{};
    std::array<double, 7> right_offset_k_{};
    std::array<double, 14> lower_limits_{};
    std::array<double, 14> upper_limits_{};
    dexcap_teleop::TrajectoryLimits trajectory_limits_;
    std::array<
        std::unique_ptr<dexcap_teleop::TrajectoryFilter>, 14> filters_;
    SteadyTime left_input_time_{};
    SteadyTime right_input_time_{};
    SteadyTime last_update_time_{};
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        left_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        right_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        robot_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<TeleopJointMapperNode>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("arm_joint_mapper"),
            "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
