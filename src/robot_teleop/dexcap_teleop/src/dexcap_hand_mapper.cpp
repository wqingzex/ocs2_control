// DexCap glove -> 6-DoF LinkerHand joint targets (thumb flex / yaw, four fingers). Ported from ctrl's
// dexcap_o6_hand_mapper, which targeted the LinkerHand O6; the joint names are the same on the L6 and the defaults
// here use the L6 joint ranges (target_at_one = the L6 upper limits). The mapping has not been tried on a real L6.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

constexpr std::size_t kDexCapJointCount = 21;
constexpr std::size_t kO6JointCount = 6;

using DexCapState = std::array<double, kDexCapJointCount>;
using O6State = std::array<double, kO6JointCount>;
using SteadyTime = std::chrono::steady_clock::time_point;

// DexCap joint-state publisher order is proximal-to-distal. Always look up the
// values by name because the SDK's native order is the reverse within a finger.
constexpr std::array<const char *, kDexCapJointCount> kRightDexCapJointNames = {
    "glove_joint_r_1_1",  // thumb ROP
    "glove_joint_r_1_2",  // thumb SWP
    "glove_joint_r_1_3",  // thumb MCP
    "glove_joint_r_1_4",  // thumb PIP
    "glove_joint_r_1_5",  // thumb DIP
    "glove_joint_r_2_1",  // index SWP
    "glove_joint_r_2_2",  // index MCP
    "glove_joint_r_2_3",  // index PIP
    "glove_joint_r_2_4",  // index DIP
    "glove_joint_r_3_1",  // middle SWP
    "glove_joint_r_3_2",  // middle MCP
    "glove_joint_r_3_3",  // middle PIP
    "glove_joint_r_3_4",  // middle DIP
    "glove_joint_r_4_1",  // ring SWP
    "glove_joint_r_4_2",  // ring MCP
    "glove_joint_r_4_3",  // ring PIP
    "glove_joint_r_4_4",  // ring DIP
    "glove_joint_r_5_1",  // little SWP
    "glove_joint_r_5_2",  // little MCP
    "glove_joint_r_5_3",  // little PIP
    "glove_joint_r_5_4",  // little DIP
};

constexpr std::array<const char *, kDexCapJointCount> kLeftDexCapJointNames = {
    "glove_joint_l_1_1",  // thumb ROP
    "glove_joint_l_1_2",  // thumb SWP
    "glove_joint_l_1_3",  // thumb MCP
    "glove_joint_l_1_4",  // thumb PIP
    "glove_joint_l_1_5",  // thumb DIP
    "glove_joint_l_2_1",  // index SWP
    "glove_joint_l_2_2",  // index MCP
    "glove_joint_l_2_3",  // index PIP
    "glove_joint_l_2_4",  // index DIP
    "glove_joint_l_3_1",  // middle SWP
    "glove_joint_l_3_2",  // middle MCP
    "glove_joint_l_3_3",  // middle PIP
    "glove_joint_l_3_4",  // middle DIP
    "glove_joint_l_4_1",  // ring SWP
    "glove_joint_l_4_2",  // ring MCP
    "glove_joint_l_4_3",  // ring PIP
    "glove_joint_l_4_4",  // ring DIP
    "glove_joint_l_5_1",  // little SWP
    "glove_joint_l_5_2",  // little MCP
    "glove_joint_l_5_3",  // little PIP
    "glove_joint_l_5_4",  // little DIP
};

constexpr std::array<const char *, kO6JointCount> kRightO6JointNames = {
    "right_hand_thumb_joint1",
    "right_hand_thumb_joint2",
    "right_hand_index_joint",
    "right_hand_middle_joint",
    "right_hand_ring_joint",
    "right_hand_pinky_joint",
};

constexpr std::array<const char *, kO6JointCount> kLeftO6JointNames = {
    "left_hand_thumb_joint1",
    "left_hand_thumb_joint2",
    "left_hand_index_joint",
    "left_hand_middle_joint",
    "left_hand_ring_joint",
    "left_hand_pinky_joint",
};

constexpr std::array<std::size_t, 12> kFourFingerFlexSourceIndices = {
    6, 7, 8,       // index MCP, PIP, DIP
    10, 11, 12,    // middle MCP, PIP, DIP
    14, 15, 16,    // ring MCP, PIP, DIP
    18, 19, 20,    // little MCP, PIP, DIP
};
constexpr std::array<std::size_t, 3> kThumbFlexSourceIndices = {
    2, 3, 4,       // thumb MCP, PIP, DIP
};

template <std::size_t Size>
std::array<double, Size> ToArray(
    const std::vector<double> &values,
    const std::string &parameterName)
{
    if (values.size() != Size)
    {
        throw std::runtime_error(
            parameterName + " must contain exactly " + std::to_string(Size) +
            " values");
    }

    std::array<double, Size> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

template <std::size_t Size>
std::vector<double> ToVector(const std::array<double, Size> &values)
{
    return {values.begin(), values.end()};
}

double ClampUnit(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

double WrapToPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class DexCapO6HandMapper final : public rclcpp::Node
{
public:
    DexCapO6HandMapper()
        : Node("dexcap_hand_mapper")
    {
        handSide_ = declare_parameter<std::string>("hand_side", "right");
        if (handSide_ != "left" && handSide_ != "right")
            throw std::runtime_error("hand_side must be 'left' or 'right'");

        const auto &defaultDexCapJointNames = handSide_ == "left"
            ? kLeftDexCapJointNames
            : kRightDexCapJointNames;
        for (std::size_t index = 0; index < kDexCapJointCount; ++index)
            dexCapJointNames_[index] = defaultDexCapJointNames[index];

        const auto &defaultO6JointNames = handSide_ == "left"
            ? kLeftO6JointNames
            : kRightO6JointNames;
        std::vector<std::string> defaultRobotJointNames(
            defaultO6JointNames.begin(), defaultO6JointNames.end());
        const auto configuredRobotJointNames =
            declare_parameter<std::vector<std::string>>(
                "robot_joint_names", defaultRobotJointNames);
        if (configuredRobotJointNames.size() != kO6JointCount)
            throw std::runtime_error(
                "robot_joint_names must contain exactly 6 names");
        std::copy(
            configuredRobotJointNames.begin(),
            configuredRobotJointNames.end(),
            robotJointNames_.begin());

        const std::string inputTopic = declare_parameter<std::string>(
            "input_topic", "/dexcap/" + handSide_ + "_hand");
        const std::string robotJointStatesTopic = declare_parameter<std::string>(
            "robot_joint_states_topic", "/joint_states");
        const std::string previewTopic = declare_parameter<std::string>(
            "preview_topic",
            "/dexcap_" + handSide_ + "_hand_mapper/joint_target_position");
        const std::string commandTopic = declare_parameter<std::string>(
            "command_topic",
            "/" + handSide_ + "_hand_controller/joint_target_position");
        const std::string fsmStateTopic = declare_parameter<std::string>(
            "fsm_state_topic", "/fsm_state");
        publishRate_ = std::clamp(
            declare_parameter<double>("publish_rate", 100.0), 1.0, 500.0);
        requireCurrentHandState_ =
            declare_parameter<bool>("require_current_hand_state", true);
        allowedFsmStates_ = declare_parameter<std::vector<int64_t>>(
            "allowed_fsm_states", {3, 4});
        if (allowedFsmStates_.empty())
            throw std::runtime_error("allowed_fsm_states must not be empty");
        calibrationWindowSeconds_ = std::clamp(
            declare_parameter<double>("calibration_window_seconds", 1.0),
            0.1, 5.0);
        minimumCalibrationSamples_ = static_cast<int>(std::max<int64_t>(
            declare_parameter<int64_t>("minimum_calibration_samples", 15), 1));
        minimumSourceRange_ = std::max(
            declare_parameter<double>("minimum_source_range", 0.03), 1e-6);
        inputTimeoutSeconds_ = std::max(
            declare_parameter<double>("input_timeout", 0.20), 0.01);
        filterCutoffHz_ = std::max(
            declare_parameter<double>("filter_cutoff_hz", 6.0), 0.0);

        const auto fingerWeights = ToArray<3>(
            declare_parameter<std::vector<double>>(
                "finger_flex_weights", {0.45, 0.35, 0.20}),
            "finger_flex_weights");
        const double fingerWeightSum =
            fingerWeights[0] + fingerWeights[1] + fingerWeights[2];
        if (fingerWeightSum <= 0.0)
            throw std::runtime_error("finger_flex_weights must have a positive sum");
        for (std::size_t index = 0; index < fingerWeights_.size(); ++index)
            fingerWeights_[index] = fingerWeights[index] / fingerWeightSum;

        const auto thumbYawWeights = ToArray<2>(
            declare_parameter<std::vector<double>>(
                "thumb_yaw_weights", {0.70, 0.30}),
            "thumb_yaw_weights");
        const double thumbYawWeightSum =
            thumbYawWeights[0] + thumbYawWeights[1];
        if (thumbYawWeightSum <= 0.0)
            throw std::runtime_error("thumb_yaw_weights must have a positive sum");
        // Parameter order is SWP, ROP.
        thumbYawWeights_[0] = thumbYawWeights[0] / thumbYawWeightSum;
        thumbYawWeights_[1] = thumbYawWeights[1] / thumbYawWeightSum;

        targetAtZero_ = ToArray<kO6JointCount>(
            declare_parameter<std::vector<double>>(
                "target_at_zero", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
            "target_at_zero");
        targetAtOne_ = ToArray<kO6JointCount>(
            declare_parameter<std::vector<double>>(
                "target_at_one", {0.99, 1.39, 1.26, 1.26, 1.26, 1.26}),
            "target_at_one");
        maxVelocity_ = ToArray<kO6JointCount>(
            declare_parameter<std::vector<double>>(
                "max_velocity", {2.5, 2.5, 4.0, 4.0, 4.0, 4.0}),
            "max_velocity");
        for (double &velocity : maxVelocity_)
            velocity = std::max(velocity, 0.0);

        LoadCalibrationParameters();

        previewPublisher_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(previewTopic, 10);
        commandPublisher_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(commandTopic, 10);
        gloveSubscription_ = create_subscription<sensor_msgs::msg::JointState>(
            inputTopic, rclcpp::SensorDataQoS(),
            std::bind(
                &DexCapO6HandMapper::GloveJointStateCallback, this,
                std::placeholders::_1));
        robotJointStateSubscription_ =
            create_subscription<sensor_msgs::msg::JointState>(
                robotJointStatesTopic, rclcpp::SensorDataQoS(),
                std::bind(
                &DexCapO6HandMapper::RobotJointStateCallback, this,
                std::placeholders::_1));
        const auto fsmQos = rclcpp::QoS(1).reliable().transient_local();
        fsmStateSubscription_ = create_subscription<std_msgs::msg::Int32>(
            fsmStateTopic, fsmQos,
            [this](const std_msgs::msg::Int32::SharedPtr message)
            {
                fsmState_.store(message->data);
            });

        calibrateOpenService_ = create_service<std_srvs::srv::Trigger>(
            "~/calibrate_open",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                CaptureFullCalibration("open", openCalibration_, response);
            });
        calibrateFourFingerFistService_ = create_service<std_srvs::srv::Trigger>(
            "~/calibrate_four_finger_fist",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                CaptureFullCalibration(
                    "four_finger_fist", fourFingerFistCalibration_, response);
            });
        calibrateThumbFlexService_ = create_service<std_srvs::srv::Trigger>(
            "~/calibrate_thumb_flex",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                CaptureThumbFlexCalibration(response);
            });
        calibrateThumbOutService_ = create_service<std_srvs::srv::Trigger>(
            "~/calibrate_thumb_out",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                CaptureThumbCalibration(
                    "thumb_out", thumbOutCalibration_, response);
            });
        calibrateThumbOppositionService_ =
            create_service<std_srvs::srv::Trigger>(
                "~/calibrate_thumb_opposition",
                [this](
                    const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr response)
                {
                    CaptureThumbCalibration(
                        "thumb_opposition", thumbOppositionCalibration_, response);
                });
        calibrationStatusService_ = create_service<std_srvs::srv::Trigger>(
            "~/calibration_status",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                response->success = CalibrationReadyLocked();
                response->message = CalibrationStatusLocked();
            });
        resetCalibrationService_ = create_service<std_srvs::srv::Trigger>(
            "~/reset_calibration",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                ResetCalibration(response);
            });
        const auto period = std::chrono::duration<double>(1.0 / publishRate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DexCapO6HandMapper::PublishTarget, this));

        RCLCPP_INFO(
            get_logger(),
            "Independent DexCap->O6 mapper ready: %s -> preview %s; "
            "FSM-gated command %s",
            inputTopic.c_str(), previewTopic.c_str(), commandTopic.c_str());
        RCLCPP_INFO(
            get_logger(),
            "O6 command order: thumb_joint1, thumb_joint2, index, middle, ring, pinky");
        if (CalibrationReady())
            RCLCPP_INFO(get_logger(), "A complete hand calibration was loaded");
        else
            RCLCPP_WARN(
                get_logger(),
                "Hand calibration is incomplete; use the five ~/calibrate_* services");
    }

private:
    struct TimedSample
    {
        SteadyTime received;
        DexCapState positions;
    };

    void LoadCalibrationParameters()
    {
        const auto open = declare_parameter<std::vector<double>>(
            "open_calibration", std::vector<double>{});
        const auto fourFingerFist = declare_parameter<std::vector<double>>(
            "four_finger_fist_calibration", std::vector<double>{});
        const auto thumbFlex = declare_parameter<std::vector<double>>(
            "thumb_flex_calibration", std::vector<double>{});
        const auto thumbOut = declare_parameter<std::vector<double>>(
            "thumb_out_calibration", std::vector<double>{});
        const auto thumbOpposition = declare_parameter<std::vector<double>>(
            "thumb_opposition_calibration", std::vector<double>{});

        if (!open.empty())
            openCalibration_ = ToArray<kDexCapJointCount>(
                open, "open_calibration");
        if (!fourFingerFist.empty())
            fourFingerFistCalibration_ = ToArray<kDexCapJointCount>(
                fourFingerFist, "four_finger_fist_calibration");
        if (!thumbFlex.empty())
            thumbFlexCalibration_ = ToArray<3>(
                thumbFlex, "thumb_flex_calibration");
        if (!thumbOut.empty())
            thumbOutCalibration_ = ToArray<2>(
                thumbOut, "thumb_out_calibration");
        if (!thumbOpposition.empty())
            thumbOppositionCalibration_ = ToArray<2>(
                thumbOpposition, "thumb_opposition_calibration");
    }

    bool ExtractDexCapState(
        const sensor_msgs::msg::JointState &message,
        DexCapState &state) const
    {
        if (message.name.size() != message.position.size())
            return false;

        std::unordered_map<std::string, std::size_t> indices;
        indices.reserve(message.name.size());
        for (std::size_t index = 0; index < message.name.size(); ++index)
            indices.emplace(message.name[index], index);

        for (std::size_t index = 0; index < kDexCapJointCount; ++index)
        {
            const auto iterator = indices.find(dexCapJointNames_[index]);
            if (iterator == indices.end())
                return false;
            const double value = message.position[iterator->second];
            if (!std::isfinite(value))
                return false;
            // All glove sensors are circular angles. Keep their canonical
            // representation in [-pi, pi]; calibration and mapping below use
            // circular statistics and shortest angular differences.
            state[index] = WrapToPi(value);
        }
        return true;
    }

    void GloveJointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr message)
    {
        DexCapState state{};
        if (!ExtractDexCapState(*message, state))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "DexCap JointState is missing one or more expected %s-glove joints",
                handSide_.c_str());
            return;
        }

        const SteadyTime received = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        latestInput_ = state;
        latestInputTime_ = received;
        haveInput_ = true;
        calibrationSamples_.push_back({received, state});
        PruneCalibrationSamplesLocked(received);

        if (CalibrationReadyLocked())
        {
            latestRawTarget_ = ComputeTargetLocked(state);
            haveTarget_ = true;
        }
    }

    void RobotJointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr message)
    {
        if (message->name.size() != message->position.size())
            return;

        std::unordered_map<std::string, std::size_t> indices;
        indices.reserve(message->name.size());
        for (std::size_t index = 0; index < message->name.size(); ++index)
            indices.emplace(message->name[index], index);

        O6State state{};
        for (std::size_t index = 0; index < kO6JointCount; ++index)
        {
            const auto iterator = indices.find(robotJointNames_[index]);
            if (iterator == indices.end())
                return;
            const double value = message->position[iterator->second];
            if (!std::isfinite(value))
                return;
            state[index] = value;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        currentHandState_ = state;
        haveCurrentHandState_ = true;
    }

    void PruneCalibrationSamplesLocked(const SteadyTime now)
    {
        const auto window = std::chrono::duration<double>(
            calibrationWindowSeconds_);
        while (!calibrationSamples_.empty() &&
               now - calibrationSamples_.front().received > window)
        {
            calibrationSamples_.pop_front();
        }
    }

    std::optional<DexCapState> AverageRecentSamplesLocked()
    {
        const SteadyTime now = std::chrono::steady_clock::now();
        PruneCalibrationSamplesLocked(now);
        if (calibrationSamples_.size() <
            static_cast<std::size_t>(minimumCalibrationSamples_))
        {
            return std::nullopt;
        }

        DexCapState average{};
        // Circular mean for every sensor. A linear average of samples around
        // +pi/-pi would incorrectly produce a value near zero.
        for (std::size_t index = 0; index < average.size(); ++index)
        {
            double sumSin = 0.0;
            double sumCos = 0.0;
            for (const TimedSample &sample : calibrationSamples_)
            {
                sumSin += std::sin(sample.positions[index]);
                sumCos += std::cos(sample.positions[index]);
            }
            average[index] = std::atan2(sumSin, sumCos);
        }
        return average;
    }

    void CaptureFullCalibration(
        const std::string &name,
        std::optional<DexCapState> &destination,
        const std_srvs::srv::Trigger::Response::SharedPtr &response)
    {
        DexCapState average{};
        std::size_t sampleCount = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto value = AverageRecentSamplesLocked();
            if (!value.has_value())
            {
                response->success = false;
                response->message =
                    "Not enough recent glove samples; hold the pose for about " +
                    std::to_string(calibrationWindowSeconds_) + " seconds";
                return;
            }
            average = *value;
            sampleCount = calibrationSamples_.size();
            destination = average;
            ResetMappedTargetLocked();
        }

        set_parameter(rclcpp::Parameter(
            name + "_calibration", ToVector(average)));
        response->success = true;
        response->message =
            "Captured " + name + " from " + std::to_string(sampleCount) +
            " recent samples";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        LogCalibrationReadiness();
    }

    void CaptureThumbCalibration(
        const std::string &name,
        std::optional<std::array<double, 2>> &destination,
        const std_srvs::srv::Trigger::Response::SharedPtr &response)
    {
        std::array<double, 2> thumb{};
        std::size_t sampleCount = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto value = AverageRecentSamplesLocked();
            if (!value.has_value())
            {
                response->success = false;
                response->message =
                    "Not enough recent glove samples; hold the pose for about " +
                    std::to_string(calibrationWindowSeconds_) + " seconds";
                return;
            }
            // Stored order is ROP, SWP, matching source indices 0 and 1.
            thumb = {(*value)[0], (*value)[1]};
            sampleCount = calibrationSamples_.size();
            destination = thumb;
            ResetMappedTargetLocked();
        }

        set_parameter(rclcpp::Parameter(
            name + "_calibration", ToVector(thumb)));
        response->success = true;
        response->message =
            "Captured " + name + " from " + std::to_string(sampleCount) +
            " recent samples";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        LogCalibrationReadiness();
    }

    void CaptureThumbFlexCalibration(
        const std_srvs::srv::Trigger::Response::SharedPtr &response)
    {
        std::array<double, 3> thumbFlex{};
        std::size_t sampleCount = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto value = AverageRecentSamplesLocked();
            if (!value.has_value())
            {
                response->success = false;
                response->message =
                    "Not enough recent glove samples; hold the pose for about " +
                    std::to_string(calibrationWindowSeconds_) + " seconds";
                return;
            }
            thumbFlex = {
                (*value)[kThumbFlexSourceIndices[0]],
                (*value)[kThumbFlexSourceIndices[1]],
                (*value)[kThumbFlexSourceIndices[2]],
            };
            sampleCount = calibrationSamples_.size();
            thumbFlexCalibration_ = thumbFlex;
            ResetMappedTargetLocked();
        }

        set_parameter(rclcpp::Parameter(
            "thumb_flex_calibration", ToVector(thumbFlex)));
        response->success = true;
        response->message =
            "Captured thumb_flex from " + std::to_string(sampleCount) +
            " recent samples";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        LogCalibrationReadiness();
    }

    void ResetCalibration(
        const std_srvs::srv::Trigger::Response::SharedPtr &response)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            openCalibration_.reset();
            fourFingerFistCalibration_.reset();
            thumbFlexCalibration_.reset();
            thumbOutCalibration_.reset();
            thumbOppositionCalibration_.reset();
            ResetMappedTargetLocked();
        }

        set_parameters({
            rclcpp::Parameter(
                "open_calibration", std::vector<double>{}),
            rclcpp::Parameter(
                "four_finger_fist_calibration", std::vector<double>{}),
            rclcpp::Parameter(
                "thumb_flex_calibration", std::vector<double>{}),
            rclcpp::Parameter(
                "thumb_out_calibration", std::vector<double>{}),
            rclcpp::Parameter(
                "thumb_opposition_calibration", std::vector<double>{}),
        });
        response->success = true;
        response->message = "Calibration cleared; O6 command output is unavailable";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }

    bool CalibrationReady() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return CalibrationReadyLocked();
    }

    bool CalibrationReadyLocked() const
    {
        if (!openCalibration_.has_value() ||
            !fourFingerFistCalibration_.has_value() ||
            !thumbFlexCalibration_.has_value() ||
            !thumbOutCalibration_.has_value() ||
            !thumbOppositionCalibration_.has_value())
        {
            return false;
        }

        for (const std::size_t index : kFourFingerFlexSourceIndices)
        {
            if (std::abs(
                    WrapToPi(
                        (*fourFingerFistCalibration_)[index] -
                        (*openCalibration_)[index])) < minimumSourceRange_)
            {
                return false;
            }
        }
        for (std::size_t index = 0; index < kThumbFlexSourceIndices.size(); ++index)
        {
            if (std::abs(
                    WrapToPi(
                        (*thumbFlexCalibration_)[index] -
                        (*openCalibration_)[kThumbFlexSourceIndices[index]])) <
                minimumSourceRange_)
            {
                return false;
            }
        }
        for (std::size_t index = 0; index < 2; ++index)
        {
            if (std::abs(
                    WrapToPi(
                        (*thumbOppositionCalibration_)[index] -
                        (*thumbOutCalibration_)[index])) < minimumSourceRange_)
            {
                return false;
            }
        }
        return true;
    }

    std::string CalibrationStatusLocked() const
    {
        std::vector<std::string> missing;
        if (!openCalibration_.has_value())
            missing.emplace_back("open");
        if (!fourFingerFistCalibration_.has_value())
            missing.emplace_back("four_finger_fist");
        if (!thumbFlexCalibration_.has_value())
            missing.emplace_back("thumb_flex");
        if (!thumbOutCalibration_.has_value())
            missing.emplace_back("thumb_out");
        if (!thumbOppositionCalibration_.has_value())
            missing.emplace_back("thumb_opposition");

        if (!missing.empty())
        {
            std::ostringstream stream;
            stream << "Missing calibration:";
            for (const std::string &name : missing)
                stream << ' ' << name;
            return stream.str();
        }

        std::vector<std::string> smallRanges;
        for (const std::size_t index : kFourFingerFlexSourceIndices)
        {
            if (std::abs(
                    WrapToPi(
                        (*fourFingerFistCalibration_)[index] -
                        (*openCalibration_)[index])) < minimumSourceRange_)
            {
                smallRanges.emplace_back(dexCapJointNames_[index]);
            }
        }
        for (std::size_t index = 0; index < kThumbFlexSourceIndices.size(); ++index)
        {
            const std::size_t sourceIndex = kThumbFlexSourceIndices[index];
            if (std::abs(
                    WrapToPi(
                        (*thumbFlexCalibration_)[index] -
                        (*openCalibration_)[sourceIndex])) < minimumSourceRange_)
            {
                smallRanges.emplace_back(dexCapJointNames_[sourceIndex]);
            }
        }
        for (std::size_t index = 0; index < 2; ++index)
        {
            if (std::abs(
                    WrapToPi(
                        (*thumbOppositionCalibration_)[index] -
                        (*thumbOutCalibration_)[index])) < minimumSourceRange_)
            {
                smallRanges.emplace_back(dexCapJointNames_[index]);
            }
        }

        if (!smallRanges.empty())
        {
            std::ostringstream stream;
            stream << "Calibration motion below minimum_source_range for:";
            for (const std::string &name : smallRanges)
                stream << ' ' << name;
            return stream.str();
        }
        return "Calibration ready";
    }

    void LogCalibrationReadiness()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string status = CalibrationStatusLocked();
        if (CalibrationReadyLocked())
            RCLCPP_INFO(get_logger(), "%s", status.c_str());
        else
            RCLCPP_WARN(get_logger(), "%s", status.c_str());
    }

    void ResetMappedTargetLocked()
    {
        haveTarget_ = false;
        filterInitialized_ = false;
        if (haveInput_ && CalibrationReadyLocked())
        {
            latestRawTarget_ = ComputeTargetLocked(latestInput_);
            haveTarget_ = true;
        }
    }

    double NormalizeAngle(
        double value,
        double zeroPose,
        double onePose) const
    {
        const double range = WrapToPi(onePose - zeroPose);
        if (std::abs(range) < minimumSourceRange_)
            return 0.0;
        return ClampUnit(WrapToPi(value - zeroPose) / range);
    }

    double FingerFlex(
        const DexCapState &state,
        const DexCapState &onePose,
        std::size_t mcpIndex,
        std::size_t pipIndex,
        std::size_t dipIndex) const
    {
        const double mcp = NormalizeAngle(
            state[mcpIndex],
            (*openCalibration_)[mcpIndex],
            onePose[mcpIndex]);
        const double pip = NormalizeAngle(
            state[pipIndex],
            (*openCalibration_)[pipIndex],
            onePose[pipIndex]);
        const double dip = NormalizeAngle(
            state[dipIndex],
            (*openCalibration_)[dipIndex],
            onePose[dipIndex]);
        return ClampUnit(
            fingerWeights_[0] * mcp +
            fingerWeights_[1] * pip +
            fingerWeights_[2] * dip);
    }

    O6State ComputeTargetLocked(const DexCapState &state) const
    {
        const double thumbMcp = NormalizeAngle(
            state[kThumbFlexSourceIndices[0]],
            (*openCalibration_)[kThumbFlexSourceIndices[0]],
            (*thumbFlexCalibration_)[0]);
        const double thumbPip = NormalizeAngle(
            state[kThumbFlexSourceIndices[1]],
            (*openCalibration_)[kThumbFlexSourceIndices[1]],
            (*thumbFlexCalibration_)[1]);
        const double thumbDip = NormalizeAngle(
            state[kThumbFlexSourceIndices[2]],
            (*openCalibration_)[kThumbFlexSourceIndices[2]],
            (*thumbFlexCalibration_)[2]);
        const double thumbFlex = ClampUnit(
            fingerWeights_[0] * thumbMcp +
            fingerWeights_[1] * thumbPip +
            fingerWeights_[2] * thumbDip);
        // thumbYawWeights parameter order is SWP, ROP. Source storage is ROP, SWP.
        const double thumbSwp = NormalizeAngle(
            state[1],
            (*thumbOutCalibration_)[1],
            (*thumbOppositionCalibration_)[1]);
        const double thumbRop = NormalizeAngle(
            state[0],
            (*thumbOutCalibration_)[0],
            (*thumbOppositionCalibration_)[0]);
        const double thumbYaw = ClampUnit(
            thumbYawWeights_[0] * thumbSwp +
            thumbYawWeights_[1] * thumbRop);

        const O6State synergy = {
            thumbFlex,
            thumbYaw,
            FingerFlex(state, *fourFingerFistCalibration_, 6, 7, 8),
            FingerFlex(state, *fourFingerFistCalibration_, 10, 11, 12),
            FingerFlex(state, *fourFingerFistCalibration_, 14, 15, 16),
            FingerFlex(state, *fourFingerFistCalibration_, 18, 19, 20),
        };

        O6State target{};
        for (std::size_t index = 0; index < target.size(); ++index)
        {
            target[index] =
                targetAtZero_[index] +
                synergy[index] *
                    (targetAtOne_[index] - targetAtZero_[index]);
        }
        return target;
    }

    void PublishTarget()
    {
        const SteadyTime now = std::chrono::steady_clock::now();
        O6State output{};
        bool publishCommand = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!haveTarget_)
                return;

            const double inputAge = std::chrono::duration<double>(
                now - latestInputTime_).count();
            if (inputAge > inputTimeoutSeconds_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "DexCap glove input timed out (%.3f s); holding the last O6 command",
                    inputAge);
                return;
            }

            if (!filterInitialized_)
            {
                if (haveCurrentHandState_)
                    filteredTarget_ = currentHandState_;
                else
                    filteredTarget_ = latestRawTarget_;
                previousFilterTime_ = now;
                filterInitialized_ = true;
            }

            const double dt = std::clamp(
                std::chrono::duration<double>(now - previousFilterTime_).count(),
                1e-4, 0.1);
            previousFilterTime_ = now;
            const double alpha = filterCutoffHz_ <= 0.0
                ? 1.0
                : 1.0 - std::exp(
                    -2.0 * std::numbers::pi_v<double> * filterCutoffHz_ * dt);

            for (std::size_t index = 0; index < output.size(); ++index)
            {
                const double filtered =
                    filteredTarget_[index] +
                    alpha * (latestRawTarget_[index] - filteredTarget_[index]);
                const double maxStep = maxVelocity_[index] * dt;
                const double rateLimited = std::clamp(
                    filtered,
                    filteredTarget_[index] - maxStep,
                    filteredTarget_[index] + maxStep);
                const double lower = std::min(
                    targetAtZero_[index], targetAtOne_[index]);
                const double upper = std::max(
                    targetAtZero_[index], targetAtOne_[index]);
                filteredTarget_[index] = std::clamp(rateLimited, lower, upper);
            }
            output = filteredTarget_;

            publishCommand =
                std::find(
                    allowedFsmStates_.begin(), allowedFsmStates_.end(),
                    static_cast<int64_t>(fsmState_.load())) !=
                    allowedFsmStates_.end() &&
                (!requireCurrentHandState_ || haveCurrentHandState_);
        }

        std_msgs::msg::Float64MultiArray message;
        message.data.assign(output.begin(), output.end());
        previewPublisher_->publish(message);
        if (publishCommand)
            commandPublisher_->publish(message);
    }

    mutable std::mutex mutex_;
    std::string handSide_{"right"};
    std::array<std::string, kDexCapJointCount> dexCapJointNames_{};
    std::array<std::string, kO6JointCount> robotJointNames_{};
    double publishRate_{100.0};
    double calibrationWindowSeconds_{1.0};
    int minimumCalibrationSamples_{15};
    double minimumSourceRange_{0.03};
    double inputTimeoutSeconds_{0.20};
    double filterCutoffHz_{6.0};
    bool requireCurrentHandState_{true};
    std::vector<int64_t> allowedFsmStates_{3, 4};
    std::atomic<int32_t> fsmState_{-1};

    std::array<double, 3> fingerWeights_{0.45, 0.35, 0.20};
    std::array<double, 2> thumbYawWeights_{0.70, 0.30};
    O6State targetAtZero_{};
    O6State targetAtOne_{};
    O6State maxVelocity_{};

    std::optional<DexCapState> openCalibration_;
    std::optional<DexCapState> fourFingerFistCalibration_;
    std::optional<std::array<double, 3>> thumbFlexCalibration_;
    std::optional<std::array<double, 2>> thumbOutCalibration_;
    std::optional<std::array<double, 2>> thumbOppositionCalibration_;

    std::deque<TimedSample> calibrationSamples_;
    DexCapState latestInput_{};
    SteadyTime latestInputTime_{};
    bool haveInput_{false};
    O6State latestRawTarget_{};
    bool haveTarget_{false};
    O6State currentHandState_{};
    bool haveCurrentHandState_{false};
    O6State filteredTarget_{};
    SteadyTime previousFilterTime_{};
    bool filterInitialized_{false};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        previewPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        commandPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        gloveSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        robotJointStateSubscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        fsmStateSubscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrateOpenService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        calibrateFourFingerFistService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        calibrateThumbFlexService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        calibrateThumbOutService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        calibrateThumbOppositionService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        calibrationStatusService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        resetCalibrationService_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DexCapO6HandMapper>());
    rclcpp::shutdown();
    return 0;
}
