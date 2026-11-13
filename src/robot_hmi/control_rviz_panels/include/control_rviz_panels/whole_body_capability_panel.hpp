#pragma once

#include <memory>

#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_control_msgs/msg/whole_body_capability.hpp"

namespace control_rviz_panels
{

  class WholeBodyCapabilityPanel : public rviz_common::Panel
  {
    Q_OBJECT

public:
    explicit WholeBodyCapabilityPanel(QWidget* parent = nullptr);
    ~WholeBodyCapabilityPanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

  private:
    void onCapabilityReceived(
        const robot_control_msgs::msg::WholeBodyCapability::SharedPtr msg);

    void updateCapabilityLabel(QLabel* label, const QString& name, bool enabled);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_control_msgs::msg::WholeBodyCapability>::SharedPtr capability_subscriber_;

    std::unique_ptr<QVBoxLayout> main_layout_;
    std::unique_ptr<QGroupBox> capability_group_;

    std::unique_ptr<QLabel> status_label_;
    std::unique_ptr<QLabel> mobile_base_label_;
    std::unique_ptr<QLabel> body_relative_label_;
    std::unique_ptr<QLabel> waist_lock_label_;
    std::unique_ptr<QLabel> custom_joint_lock_label_;
    std::unique_ptr<QLabel> bimanual_label_;
    std::unique_ptr<QLabel> body_tracking_label_;
    std::unique_ptr<QLabel> head_tracking_label_;
    std::unique_ptr<QLabel> home_pose_tracking_label_;
  };

}  // namespace control_rviz_panels
