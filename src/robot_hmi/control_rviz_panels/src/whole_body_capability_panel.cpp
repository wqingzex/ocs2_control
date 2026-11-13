#include "control_rviz_panels/whole_body_capability_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace control_rviz_panels
{

WholeBodyCapabilityPanel::WholeBodyCapabilityPanel(QWidget* parent)
    : rviz_common::Panel(parent)
{
    main_layout_ = std::make_unique<QVBoxLayout>(this);

    capability_group_ = std::make_unique<QGroupBox>("WBC Capabilities", this);
    auto* group_layout = new QVBoxLayout(capability_group_.get());

    status_label_ = std::make_unique<QLabel>("Waiting for capability message...", capability_group_.get());
    status_label_->setStyleSheet("QLabel { color: #666666; font-style: italic; }");
    group_layout->addWidget(status_label_.get());

    mobile_base_label_ = std::make_unique<QLabel>("Has Mobile Base: Unknown", capability_group_.get());
    body_relative_label_ = std::make_unique<QLabel>("Has Body Relative Constraint: Unknown", capability_group_.get());
    waist_lock_label_ = std::make_unique<QLabel>("Has Waist Lock: Unknown", capability_group_.get());
    custom_joint_lock_label_ = std::make_unique<QLabel>("Has Custom Joint Lock: Unknown", capability_group_.get());
    bimanual_label_ = std::make_unique<QLabel>("Has Bimanual Coupling: Unknown", capability_group_.get());
    body_tracking_label_ = std::make_unique<QLabel>("Body Tracking EE Enabled: Unknown", capability_group_.get());
    head_tracking_label_ = std::make_unique<QLabel>("Head Tracking EE Enabled: Unknown", capability_group_.get());
    home_pose_tracking_label_ = std::make_unique<QLabel>("Has Home Joint Reference: Unknown", capability_group_.get());

    group_layout->addWidget(mobile_base_label_.get());
    group_layout->addWidget(body_relative_label_.get());
    group_layout->addWidget(waist_lock_label_.get());
    group_layout->addWidget(custom_joint_lock_label_.get());
    group_layout->addWidget(bimanual_label_.get());
    group_layout->addWidget(body_tracking_label_.get());
    group_layout->addWidget(head_tracking_label_.get());
    group_layout->addWidget(home_pose_tracking_label_.get());

    main_layout_->addWidget(capability_group_.get());
}

WholeBodyCapabilityPanel::~WholeBodyCapabilityPanel() = default;

void WholeBodyCapabilityPanel::onInitialize()
{
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    capability_subscriber_ =
        node_->create_subscription<robot_control_msgs::msg::WholeBodyCapability>(
            "/whole_body_controller/whole_body_capabilities",
            rclcpp::QoS(1).transient_local(),
            std::bind(&WholeBodyCapabilityPanel::onCapabilityReceived, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
                "WholeBodyCapabilityPanel initialized, subscribed to /whole_body_controller/whole_body_capabilities");
}

void WholeBodyCapabilityPanel::onCapabilityReceived(
    const robot_control_msgs::msg::WholeBodyCapability::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    status_label_->setText("Capability message received");
    status_label_->setStyleSheet("QLabel { color: #2e7d32; font-weight: bold; }");

    updateCapabilityLabel(mobile_base_label_.get(), "Has Mobile Base", msg->has_mobile_base);
    updateCapabilityLabel(body_relative_label_.get(), "Has Body Relative Constraint", msg->has_body_relative_constraint);
    updateCapabilityLabel(waist_lock_label_.get(), "Has Waist Lock", msg->has_waist_lock);
    updateCapabilityLabel(custom_joint_lock_label_.get(), "Has Custom Joint Lock", msg->has_custom_joint_lock);
    updateCapabilityLabel(bimanual_label_.get(), "Has Bimanual Coupling", msg->has_bimanual_coupling);
    updateCapabilityLabel(body_tracking_label_.get(), "Body Tracking EE Enabled", msg->body_tracking_ee_enabled);
    updateCapabilityLabel(head_tracking_label_.get(), "Head Tracking EE Enabled", msg->head_tracking_ee_enabled);
    updateCapabilityLabel(home_pose_tracking_label_.get(), "Has Home Joint Reference", msg->has_home_joint_reference);
}

void WholeBodyCapabilityPanel::updateCapabilityLabel(QLabel* label, const QString& name, bool enabled)
{
    if (!label)
    {
        return;
    }

    label->setText(QString("%1: %2").arg(name, enabled ? "Yes" : "No"));
    label->setStyleSheet(
        enabled
            ? "QLabel { color: #2e7d32; font-weight: bold; }"
            : "QLabel { color: #c62828; font-weight: bold; }");
}

void WholeBodyCapabilityPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void WholeBodyCapabilityPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

}  // namespace control_rviz_panels

PLUGINLIB_EXPORT_CLASS(control_rviz_panels::WholeBodyCapabilityPanel, rviz_common::Panel)
