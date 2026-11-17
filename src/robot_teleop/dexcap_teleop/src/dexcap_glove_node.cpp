#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

#include "cpp/DexCap.hpp"
#include "dexcap_teleop/dexcap_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{

struct JointSpec
{
    const char *name;
    double lower;
    double upper;
};

constexpr std::array<JointSpec, 21> kLeftJoints = {{
    {"glove_joint_l_1_1", -0.5236, 1.0472},
    {"glove_joint_l_1_2", -1.3439, 1.5882},
    {"glove_joint_l_1_3", -2.1817, 1.3265},
    {"glove_joint_l_1_4", -1.0297, 2.7402},
    {"glove_joint_l_1_5", -2.8623, 0.9948},
    {"glove_joint_l_2_1", -1.0996, 0.7854},
    {"glove_joint_l_2_2", -3.1590, 0.1571},
    {"glove_joint_l_2_3", -0.1745, 3.4034},
    {"glove_joint_l_2_4", -3.1416, 0.8029},
    {"glove_joint_l_3_1", -0.6807, 0.9599},
    {"glove_joint_l_3_2", -3.0020, 0.2967},
    {"glove_joint_l_3_3", -0.2967, 3.3859},
    {"glove_joint_l_3_4", -3.1590, 0.6981},
    {"glove_joint_l_4_1", -0.4014, 1.2217},
    {"glove_joint_l_4_2", -3.1416, 0.1745},
    {"glove_joint_l_4_3", -0.1920, 3.4034},
    {"glove_joint_l_4_4", -3.1765, 0.7679},
    {"glove_joint_l_5_1", -0.0349, 0.8203},
    {"glove_joint_l_5_2", -3.2812, 0.1745},
    {"glove_joint_l_5_3", -0.2269, 3.5605},
    {"glove_joint_l_5_4", -2.3387, 1.2566},
}};

constexpr std::array<JointSpec, 21> kRightJoints = {{
    {"glove_joint_r_1_1", -1.0821, 0.5061},
    {"glove_joint_r_1_2", -1.5882, 1.3788},
    {"glove_joint_r_1_3", -1.3265, 2.1468},
    {"glove_joint_r_1_4", -2.7227, 0.9948},
    {"glove_joint_r_1_5", -1.0472, 2.9322},
    {"glove_joint_r_2_1", -0.8203, 1.0996},
    {"glove_joint_r_2_2", -0.1920, 3.1590},
    {"glove_joint_r_2_3", -3.5256, 0.1571},
    {"glove_joint_r_2_4", -0.7854, 3.1765},
    {"glove_joint_r_3_1", -1.0647, 0.5760},
    {"glove_joint_r_3_2", -0.4014, 3.0718},
    {"glove_joint_r_3_3", -3.3685, 0.3142},
    {"glove_joint_r_3_4", -0.7854, 3.1765},
    {"glove_joint_r_4_1", -1.2217, 0.3491},
    {"glove_joint_r_4_2", -0.2094, 3.1416},
    {"glove_joint_r_4_3", -3.4907, 0.1920},
    {"glove_joint_r_4_4", -0.8029, 3.1590},
    {"glove_joint_r_5_1", -0.8029, 0.0349},
    {"glove_joint_r_5_2", -0.1920, 3.2812},
    {"glove_joint_r_5_3", -3.5430, 0.2094},
    {"glove_joint_r_5_4", -1.2566, 2.5656},
}};

constexpr std::array<std::size_t, 21> kUrdfToSdkIndex = {{
    4, 3, 2, 1, 0,
    8, 7, 6, 5,
    12, 11, 10, 9,
    16, 15, 14, 13,
    20, 19, 18, 17,
}};

std::array<uint16_t, 21> to_sdk_order(const GloveJointAngles &state)
{
    return {{
        state.ThumbDIP, state.ThumbPIP, state.ThumbMCP,
        state.ThumbSWP, state.ThumbROP,
        state.IndexDIP, state.IndexPIP, state.IndexMCP, state.IndexSWP,
        state.MiddleDIP, state.MiddlePIP, state.MiddleMCP, state.MiddleSWP,
        state.RingDIP, state.RingPIP, state.RingMCP, state.RingSWP,
        state.LittleDIP, state.LittlePIP, state.LittleMCP, state.LittleSWP,
    }};
}

double raw_angle_to_radians(
    const uint16_t raw_angle, const JointSpec &joint)
{
    const double angle = static_cast<double>(raw_angle) *
        std::numbers::pi_v<double> / 18000.0;
    constexpr double two_pi = 2.0 * std::numbers::pi_v<double>;
    double best = std::clamp(angle, joint.lower, joint.upper);
    double best_distance = std::abs(angle - best);
    for (int turns = -2; turns <= 2; ++turns)
    {
        const double candidate = angle + static_cast<double>(turns) * two_pi;
        const double limited =
            std::clamp(candidate, joint.lower, joint.upper);
        const double distance = std::abs(candidate - limited);
        if (distance < best_distance)
        {
            best = limited;
            best_distance = distance;
        }
    }
    return best;
}

}  // namespace

class DexCapGloveNode final : public rclcpp::Node
{
public:
    DexCapGloveNode()
        : Node("dexcap_glove")
    {
        side_ = declare_parameter<std::string>("side", "right");
        if (side_ != "left" && side_ != "right")
        {
            throw std::invalid_argument("side must be 'left' or 'right'");
        }
        device_type_ = side_ == "left" ? LGlove : RGlove;
        const auto default_topic =
            side_ == "left" ? "/dexcap/left_hand" : "/dexcap/right_hand";

        std::string serial_port =
            declare_parameter<std::string>("serial_port", "");
        const double publish_rate = std::clamp(
            declare_parameter<double>("publish_rate", 40.0), 1.0, 1000.0);
        const std::string topic =
            declare_parameter<std::string>("topic", default_topic);
        if (serial_port.empty())
        {
            serial_port =
                dexcap_teleop::find_serial_port(device_type_);
        }
        if (serial_port.empty())
        {
            throw std::runtime_error(
                "No DexCap V4 " + side_ +
                " glove found; set serial_port explicitly");
        }

        publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(topic, 10);
        suit_ = std::make_unique<DexRobot::DexCapSuit>(ProductVersion::V4);
        RCLCPP_INFO(
            get_logger(), "Connecting DexCap %s glove on %s",
            side_.c_str(), serial_port.c_str());
        const auto device =
            suit_->ConnectDevice(serial_port, DexRobot::AdapterType::WIREDUSB);
        if (device != device_type_)
        {
            if (device != UnDefn)
            {
                suit_->DisconnectDevice(device);
            }
            throw std::runtime_error(
                "The selected serial device is not the DexCap " +
                side_ + " glove");
        }
        if (!suit_->Start(device_type_))
        {
            const std::string error = suit_->getErrorMessage(device_type_);
            suit_->Close(device_type_);
            throw std::runtime_error(
                "Could not start DexCap " + side_ +
                " glove sampling: " + error);
        }
        connected_ = true;

        const auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DexCapGloveNode::publish_state, this));
        RCLCPP_INFO(
            get_logger(),
            "%s glove polling configured at %.1f Hz; publishing %s",
            side_.c_str(), publish_rate, topic.c_str());
    }

    ~DexCapGloveNode() override
    {
        close_device();
    }

private:
    void close_device()
    {
        if (!connected_ || !suit_)
        {
            return;
        }
        connected_ = false;
        const DEX_RETURN result = suit_->Close(device_type_);
        if (result == DEX_SUCCESS)
        {
            RCLCPP_INFO(
                get_logger(), "DexCap %s glove closed", side_.c_str());
        }
        else if (result == DEX_SUCCESS_WITH_INFO)
        {
            RCLCPP_WARN(
                get_logger(),
                "DexCap %s glove closed with additional SDK information",
                side_.c_str());
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "DexCap %s glove close failed with code %d",
                side_.c_str(), result);
        }
    }

    void publish_state()
    {
        const GloveJointAngles state = side_ == "left"
            ? suit_->GetLeftGloveJointState()
            : suit_->GetRightGloveJointState();
        if (state.timestamp == 0 || state.timestamp == last_timestamp_)
        {
            return;
        }
        last_timestamp_ = state.timestamp;

        const auto sdk_angles = to_sdk_order(state);
        const auto &specs = side_ == "left" ? kLeftJoints : kRightJoints;
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name.reserve(specs.size());
        message.position.reserve(specs.size());

        for (std::size_t urdf_index = 0;
             urdf_index < specs.size(); ++urdf_index)
        {
            const std::size_t sdk_index = kUrdfToSdkIndex[urdf_index];
            const uint32_t error_bit = uint32_t{0x80000000U} >> sdk_index;
            double position = previous_position_[urdf_index].value_or(
                std::clamp(0.0, specs[urdf_index].lower,
                    specs[urdf_index].upper));
            if ((state.ErrorMask & error_bit) == 0U)
            {
                position = raw_angle_to_radians(
                    sdk_angles[sdk_index], specs[urdf_index]);
                previous_position_[urdf_index] = position;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "%s glove reports an error for %s; holding last value",
                    side_.c_str(), specs[urdf_index].name);
            }
            message.name.emplace_back(specs[urdf_index].name);
            message.position.push_back(position);
        }
        publisher_->publish(message);
    }

    std::string side_;
    DEXCAP_DEVICE_TYPE device_type_{RGlove};
    std::unique_ptr<DexRobot::DexCapSuit> suit_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<std::optional<double>, 21> previous_position_{};
    uint64_t last_timestamp_{0};
    bool connected_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<DexCapGloveNode>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("dexcap_teleop"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
