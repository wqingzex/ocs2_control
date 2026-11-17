#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
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

constexpr std::array<JointSpec, 5> kWaistJoints = {{
    {"waist_joint_1", -1.3090, 1.7977},
    {"waist_joint_2", -2.0769, 1.9897},
    {"waist_joint_3", -2.2515, 2.5656},
    {"waist_joint_4", -1.5359, 0.4712},
    {"waist_joint_5", -1.0996, 0.9774},
}};

constexpr std::array<JointSpec, 9> kLeftArmJoints = {{
    {"left_arm_joint_1", -2.6005, 3.0543},
    {"left_arm_joint_2", -2.4435, 0.8727},
    {"left_arm_joint_3", -1.2217, 4.3982},
    {"left_arm_joint_4", -4.3982, 0.8727},
    {"left_arm_joint_5", -4.3982, 1.2043},
    {"left_arm_joint_6", -1.5533, 1.5708},
    {"left_arm_joint_7", -1.3265, 1.9548},
    {"left_arm_joint_8", -2.6005, 2.2864},
    {"left_arm_joint_9", -0.4189, 3.4907},
}};

constexpr std::array<JointSpec, 9> kRightArmJoints = {{
    {"right_arm_joint_1", -3.1241, 2.5482},
    {"right_arm_joint_2", -2.4435, 0.8378},
    {"right_arm_joint_3", -4.3808, 1.2217},
    {"right_arm_joint_4", -2.4435, 0.8378},
    {"right_arm_joint_5", -1.2392, 4.3459},
    {"right_arm_joint_6", -1.5533, 1.5533},
    {"right_arm_joint_7", -1.3090, 1.9722},
    {"right_arm_joint_8", -2.6180, 2.2689},
    {"right_arm_joint_9", -0.4363, 3.4907},
}};

double raw_angle_to_radians(
    const uint16_t raw_angle, const JointSpec &joint)
{
    const double angle = static_cast<double>(raw_angle) *
        std::numbers::pi_v<double> / 18000.0;
    constexpr double two_pi = 2.0 * std::numbers::pi_v<double>;
    double best = angle;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int turns = -2; turns <= 2; ++turns)
    {
        const double candidate = angle + static_cast<double>(turns) * two_pi;
        const double distance = candidate < joint.lower
            ? joint.lower - candidate
            : (candidate > joint.upper ? candidate - joint.upper : 0.0);
        if (distance < best_distance)
        {
            best = candidate;
            best_distance = distance;
        }
    }
    return best;
}

template<std::size_t Size>
sensor_msgs::msg::JointState make_joint_state(
    const rclcpp::Time &stamp,
    const std::array<JointSpec, Size> &specs,
    const std::array<uint16_t, Size> &raw_angles)
{
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name.reserve(Size);
    message.position.reserve(Size);
    for (std::size_t index = 0; index < Size; ++index)
    {
        message.name.emplace_back(specs[index].name);
        message.position.push_back(
            raw_angle_to_radians(raw_angles[index], specs[index]));
    }
    return message;
}

}  // namespace

class DexCapUpperBodyNode final : public rclcpp::Node
{
public:
    DexCapUpperBodyNode()
        : Node("dexcap_upper_body")
    {
        std::string serial_port =
            declare_parameter<std::string>("serial_port", "");
        const double publish_rate = std::clamp(
            declare_parameter<double>("publish_rate", 1000.0), 1.0, 1000.0);
        const std::string waist_topic =
            declare_parameter<std::string>("waist_topic", "/dexcap/waist");
        const std::string left_arm_topic =
            declare_parameter<std::string>(
                "left_arm_topic", "/dexcap/left_arm");
        const std::string right_arm_topic =
            declare_parameter<std::string>(
                "right_arm_topic", "/dexcap/right_arm");

        if (serial_port.empty())
        {
            serial_port =
                dexcap_teleop::find_serial_port(UpBody);
        }
        if (serial_port.empty())
        {
            throw std::runtime_error(
                "No DexCap V4 upper-body device found; set serial_port");
        }

        waist_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(waist_topic, 10);
        left_arm_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(left_arm_topic, 10);
        right_arm_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(right_arm_topic, 10);

        suit_ = std::make_unique<DexRobot::DexCapSuit>(ProductVersion::V4);
        RCLCPP_INFO(
            get_logger(), "Connecting DexCap upper body on %s",
            serial_port.c_str());
        const auto device =
            suit_->ConnectDevice(serial_port, DexRobot::AdapterType::WIREDUSB);
        if (device != UpBody)
        {
            if (device != UnDefn)
            {
                suit_->DisconnectDevice(device);
            }
            throw std::runtime_error(
                "The selected serial device is not a DexCap upper body");
        }
        if (!suit_->Start(UpBody))
        {
            const std::string error = suit_->getErrorMessage(UpBody);
            suit_->Close(UpBody);
            throw std::runtime_error(
                "Could not start DexCap upper-body sampling: " + error);
        }
        connected_ = true;

        const auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DexCapUpperBodyNode::publish_state, this));
        RCLCPP_INFO(
            get_logger(),
            "Upper-body polling configured at %.1f Hz; publishing %s, %s, %s",
            publish_rate, waist_topic.c_str(), left_arm_topic.c_str(),
            right_arm_topic.c_str());
    }

    ~DexCapUpperBodyNode() override
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
        const DEX_RETURN result = suit_->Close(UpBody);
        if (result == DEX_SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "DexCap upper body closed");
        }
        else if (result == DEX_SUCCESS_WITH_INFO)
        {
            RCLCPP_WARN(
                get_logger(),
                "DexCap upper body closed with additional SDK information");
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "DexCap upper-body close failed with code %d",
                result);
        }
    }

    void publish_state()
    {
        const SkeletonJointAngles state = suit_->GetBodyJointState();
        if (state.timestamp == 0 || state.timestamp == last_timestamp_)
        {
            return;
        }
        last_timestamp_ = state.timestamp;
        const auto stamp = now();

        // SDK waist order runs from the shoulder end toward the IMU. Publish
        // from the IMU/base end toward the shoulder to match the V4 URDF.
        const std::array<uint16_t, 5> waist = {
            state.Back5, state.Back4, state.Back3, state.Back2, state.Back1};
        const std::array<uint16_t, 9> left_arm = {
            state.LArm1, state.LArm2, state.LArm3,
            state.LArm4, state.LArm5, state.LArm6,
            state.LArm7, state.LArm8, state.LArm9};
        const std::array<uint16_t, 9> right_arm = {
            state.RArm1, state.RArm2, state.RArm3,
            state.RArm4, state.RArm5, state.RArm6,
            state.RArm7, state.RArm8, state.RArm9};

        waist_publisher_->publish(
            make_joint_state(stamp, kWaistJoints, waist));
        left_arm_publisher_->publish(
            make_joint_state(stamp, kLeftArmJoints, left_arm));
        right_arm_publisher_->publish(
            make_joint_state(stamp, kRightArmJoints, right_arm));
    }

    std::unique_ptr<DexRobot::DexCapSuit> suit_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr waist_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        left_arm_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        right_arm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t last_timestamp_{0};
    bool connected_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<DexCapUpperBodyNode>());
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
