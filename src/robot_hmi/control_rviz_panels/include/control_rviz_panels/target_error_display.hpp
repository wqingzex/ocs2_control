#pragma once
#include <rviz_common/display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <robot_control_msgs/msg/whole_body_current_state.hpp>
#include <memory>
#include <array>
#include <chrono>
#include <mutex>
#include "control_rviz_panels/error_dashboard.hpp"

namespace control_rviz_panels {
class TargetErrorDisplay : public rviz_common::Display {
    Q_OBJECT
public:
    TargetErrorDisplay();
    ~TargetErrorDisplay() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;
protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
private Q_SLOTS:
    void updateScale();
private:
    struct Sample {
        geometry_msgs::msg::PoseStamped pose{};
        std::chrono::steady_clock::time_point received{};
        bool valid{false};
    };
    struct Data {
        std::mutex mutex;
        std::array<std::array<Sample, 2>, 4> samples{};
        std::array<bool, 4> active{};
        bool mpc_active{false};
        bool whole_body_state_received{false};
        uint64_t fsm_generation{0};
    };
    std::shared_ptr<Data> data_{std::make_shared<Data>()};
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 8> subscriptions_;
    rclcpp::Subscription<robot_control_msgs::msg::WholeBodyCurrentState>::SharedPtr mode_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_subscription_;
    std::unique_ptr<ErrorDashboard> dashboard_;
    rviz_common::properties::FloatProperty* scale_property_{};
    uint64_t fsm_generation_{0};
    std::array<double, 4> thresholds_{5, 20, 1, 5};
    float elapsed_{0};
};
}
