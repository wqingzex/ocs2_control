#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

bool copy_joints(
    const sensor_msgs::msg::JointState &message,
    const std::unordered_map<std::string, std::size_t> &indices,
    const std::vector<std::string> &names,
    KDL::JntArray &result)
{
    if (result.rows() != names.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        const auto found = indices.find(names[index]);
        if (found == indices.end() ||
            found->second >= message.position.size() ||
            !std::isfinite(message.position[found->second]))
        {
            return false;
        }
        result(index) = message.position[found->second];
    }
    return true;
}

}  // namespace

class DexCapGripperMapper final : public rclcpp::Node
{
public:
    DexCapGripperMapper()
        : Node("dexcap_gripper_mapper")
    {
        hand_side_ = declare_parameter<std::string>("hand_side", "right");
        if (hand_side_ != "left" && hand_side_ != "right")
        {
            throw std::invalid_argument(
                "hand_side must be 'left' or 'right'");
        }
        const std::string side_short =
            hand_side_ == "left" ? "l" : "r";
        for (int joint = 1; joint <= 5; ++joint)
        {
            thumb_joint_names_.push_back(
                "glove_joint_" + side_short + "_1_" +
                std::to_string(joint));
        }
        for (int joint = 1; joint <= 4; ++joint)
        {
            index_joint_names_.push_back(
                "glove_joint_" + side_short + "_2_" +
                std::to_string(joint));
        }
        thumb_tip_link_ =
            "glove_link_" + side_short + "_1_5";
        index_tip_link_ =
            "glove_link_" + side_short + "_2_4";

        const auto input_topic = declare_parameter<std::string>(
            "input_topic", "/dexcap/" + hand_side_ + "_hand");
        const auto pinch_topic = declare_parameter<std::string>(
            "pinch_distance_topic",
            "/dexcap/" + hand_side_ + "_hand/pinch_distance");
        const auto target_topic = declare_parameter<std::string>(
            "gripper_target_topic",
            "/" + hand_side_ + "_gripper_controller/target_percent");
        publish_target_ =
            declare_parameter<bool>("publish_gripper_target", false);
        closed_distance_ =
            declare_parameter<double>("pinch_closed_distance", 0.040);
        open_distance_ =
            declare_parameter<double>("pinch_open_distance", 0.110);
        filter_alpha_ = std::clamp(
            declare_parameter<double>("gripper_filter_alpha", 0.25),
            0.0, 1.0);
        minimum_change_ = std::max(
            declare_parameter<double>("gripper_min_change", 0.005), 0.0);
        input_timeout_seconds_ = std::max(
            declare_parameter<double>("input_timeout", 0.20), 0.01);
        if (open_distance_ <= closed_distance_)
        {
            throw std::invalid_argument(
                "pinch_open_distance must exceed pinch_closed_distance");
        }

        const auto thumb_tip = declare_parameter<std::vector<double>>(
            "thumb_tip_offset", {0.0075, 0.0281, 0.0});
        const auto index_tip = declare_parameter<std::vector<double>>(
            "index_tip_offset", {0.0075, 0.0281, 0.0});
        if (thumb_tip.size() != 3 || index_tip.size() != 3)
        {
            throw std::invalid_argument(
                "thumb_tip_offset and index_tip_offset require three values");
        }
        thumb_tip_offset_ =
            KDL::Vector(thumb_tip[0], thumb_tip[1], thumb_tip[2]);
        index_tip_offset_ =
            KDL::Vector(index_tip[0], index_tip[1], index_tip[2]);
        configure_kinematics(
            declare_parameter<std::string>("robot_description", ""));

        pinch_publisher_ =
            create_publisher<std_msgs::msg::Float64>(pinch_topic, 10);
        target_publisher_ =
            create_publisher<std_msgs::msg::Float64>(target_topic, 10);
        glove_subscription_ =
            create_subscription<sensor_msgs::msg::JointState>(
                input_topic, rclcpp::SensorDataQoS(),
                std::bind(
                    &DexCapGripperMapper::glove_callback, this,
                    std::placeholders::_1));
        input_watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DexCapGripperMapper::check_input_timeout, this));

        RCLCPP_INFO(
            get_logger(),
            "%s gripper mapper: input=%s, %.3f m=closed, %.3f m=open, "
            "output=%s",
            hand_side_.c_str(), input_topic.c_str(), closed_distance_,
            open_distance_, target_topic.c_str());
    }

private:
    void configure_kinematics(const std::string &description)
    {
        if (description.empty())
        {
            throw std::invalid_argument(
                "robot_description is required for pinch kinematics");
        }
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(description, tree) ||
            !tree.getChain("base_link", thumb_tip_link_, thumb_chain_) ||
            !tree.getChain("base_link", index_tip_link_, index_chain_))
        {
            throw std::runtime_error(
                "Could not construct DexCap thumb/index KDL chains");
        }
        thumb_solver_ =
            std::make_unique<KDL::ChainFkSolverPos_recursive>(thumb_chain_);
        index_solver_ =
            std::make_unique<KDL::ChainFkSolverPos_recursive>(index_chain_);
    }

    void glove_callback(
        const sensor_msgs::msg::JointState::SharedPtr message)
    {
        latest_input_time_ = std::chrono::steady_clock::now();
        have_input_ = true;
        std::unordered_map<std::string, std::size_t> indices;
        for (std::size_t index = 0; index < message->name.size(); ++index)
        {
            indices.emplace(message->name[index], index);
        }

        KDL::JntArray thumb(thumb_chain_.getNrOfJoints());
        KDL::JntArray index(index_chain_.getNrOfJoints());
        if (!copy_joints(
                *message, indices, thumb_joint_names_, thumb) ||
            !copy_joints(
                *message, indices, index_joint_names_, index))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "%s-hand input is missing finite thumb/index joints",
                hand_side_.c_str());
            return;
        }

        KDL::Frame thumb_frame;
        KDL::Frame index_frame;
        if (thumb_solver_->JntToCart(thumb, thumb_frame) < 0 ||
            index_solver_->JntToCart(index, index_frame) < 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to calculate thumb/index fingertip positions");
            return;
        }
        const double distance = (
            thumb_frame * thumb_tip_offset_ -
            index_frame * index_tip_offset_).Norm();
        std_msgs::msg::Float64 distance_message;
        distance_message.data = distance;
        pinch_publisher_->publish(distance_message);

        const double raw = std::clamp(
            (distance - closed_distance_) /
                (open_distance_ - closed_distance_),
            0.0, 1.0);
        filtered_target_ = filtered_target_.has_value()
            ? filter_alpha_ * raw +
                (1.0 - filter_alpha_) * *filtered_target_
            : raw;
        if (!publish_target_ ||
            (last_target_.has_value() &&
             std::abs(*filtered_target_ - *last_target_) < minimum_change_))
        {
            return;
        }

        std_msgs::msg::Float64 target_message;
        target_message.data = *filtered_target_;
        target_publisher_->publish(target_message);
        last_target_ = *filtered_target_;
    }

    void check_input_timeout()
    {
        if (!have_input_)
        {
            return;
        }
        const double input_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - latest_input_time_).count();
        if (input_age > input_timeout_seconds_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "DexCap %s glove input timed out (%.3f s); "
                "holding the last gripper command",
                hand_side_.c_str(), input_age);
        }
    }

    bool publish_target_{false};
    std::string hand_side_;
    std::string thumb_tip_link_;
    std::string index_tip_link_;
    std::vector<std::string> thumb_joint_names_;
    std::vector<std::string> index_joint_names_;
    double closed_distance_{0.040};
    double open_distance_{0.110};
    double filter_alpha_{0.25};
    double minimum_change_{0.005};
    double input_timeout_seconds_{0.20};
    bool have_input_{false};
    std::chrono::steady_clock::time_point latest_input_time_{};
    std::optional<double> filtered_target_;
    std::optional<double> last_target_;
    KDL::Chain thumb_chain_;
    KDL::Chain index_chain_;
    KDL::Vector thumb_tip_offset_;
    KDL::Vector index_tip_offset_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> thumb_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> index_solver_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pinch_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        glove_subscription_;
    rclcpp::TimerBase::SharedPtr input_watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<DexCapGripperMapper>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("dexcap_gripper_mapper"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
