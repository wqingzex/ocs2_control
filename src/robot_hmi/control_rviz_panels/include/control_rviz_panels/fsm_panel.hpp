#ifndef CONTROL_RVIZ_PANELS_OCS2_FSM_PANEL_HPP
#define CONTROL_RVIZ_PANELS_OCS2_FSM_PANEL_HPP

#include <memory>
#include <string>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QComboBox>
#include <QFrame>
#include <QMetaObject>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <robot_control_msgs/msg/whole_body_capability.hpp>
#include <robot_control_msgs/msg/whole_body_current_state.hpp>

#include "control_rviz_panels/switch_button.hpp"

namespace control_rviz_panels
{

class FsmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  FsmPanel(QWidget* parent = nullptr);
  ~FsmPanel() override;

  void onInitialize() override;

  // Load and save configuration data
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  // FSM state transition slots
  void onHomeToHold();
  void onHoldToOCS2();
  void onOCS2ToHold();
  void onHoldToHome();
  void onHoldToMoveJ();
  void onMoveJToHold();
  void onSwitchPose();
  void onSwitchPoseReleased();

  // WBC control slots
  void onBaseToggled();
  void onBimanualToggled();
  void onLeftArmToggled();
  void onRightArmToggled();
  void onBodyModeChanged(int index);
  void onHeadModeChanged(int index);
  void onHomePoseTrackingToggled();

private:
  // FSM related methods
  void publishCommand(int32_t command);
  void updateStatusDisplay();
  void updateButtonVisibility();
  void setCurrentState(const std::string& state);
  void onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg);

  // WBC related methods
  bool whole_body_available_ = false;
  void publishModeCommand(const std::string& cmd);
  void onReceiveCapability(const robot_control_msgs::msg::WholeBodyCapability::SharedPtr msg);
  void onReceiveCurrentState(const robot_control_msgs::msg::WholeBodyCurrentState::SharedPtr msg);

  void refreshWholeBodyUi();
  void updateBodyComboBox();
  void updateHeadComboBox();
  void updateSwitchVisualState(SwitchButton* sw, bool capability_available, bool logical_on);

  bool isBaseLocked() const;
  bool isBimanualCoupled() const;
  bool isLeftArmEnabled() const;
  bool isRightArmEnabled() const;

  bool isBodyFree() const;
  bool isBodyVertical() const;
  bool isBodyTracking() const;
  bool isBodyLocked() const;
  bool isBodyCustomLocked() const;

  int getCurrentBodyModeIndex() const;
  QString getBodyModeCommand(int modeIndex) const;

  // WBC mode indices
  enum BodyModeIndex
  {
    BODY_MODE_FREE = 0,
    BODY_MODE_VERTICAL = 1,
    BODY_MODE_TRACKING = 2,
    BODY_MODE_LOCKED = 3,
    BODY_MODE_CUSTOM_LOCKED = 5
  };

  struct CapabilityState
  {
    bool has_mobile_base = false;
    bool has_body_relative_constraint = false;
    bool has_waist_lock = false;
    bool has_head_coupling = false;
    bool has_custom_joint_lock = false;
    bool has_bimanual_coupling = false;
    bool body_tracking_ee_enabled = false;
    bool head_tracking_ee_enabled = false;
    bool head_midpoint_gaze_enabled = false;
    bool has_home_joint_reference = false;
  };

  struct CurrentWholeBodyState
  {
    uint8_t base_state = 0;
    uint8_t body_state = 0;
    uint8_t head_state = 0;
    uint8_t bimanual_state = 0;
    uint8_t left_arm_state = 0;
    uint8_t right_arm_state = 0;
    bool home_joint_reference_enabled = false;
  };

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscriber_;

  // WBC ROS2
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_command_pub_;
  rclcpp::Subscription<robot_control_msgs::msg::WholeBodyCapability>::SharedPtr capability_sub_;
  rclcpp::Subscription<robot_control_msgs::msg::WholeBodyCurrentState>::SharedPtr state_sub_;

  // FSM UI Elements
  std::unique_ptr<QGroupBox> button_group_;
  std::unique_ptr<QPushButton> home_to_hold_btn_;
  std::unique_ptr<QPushButton> hold_to_ocs2_btn_;
  std::unique_ptr<QPushButton> ocs2_to_hold_btn_;
  std::unique_ptr<QPushButton> hold_to_home_btn_;
  std::unique_ptr<QPushButton> hold_to_movej_btn_;
  std::unique_ptr<QPushButton> movej_to_hold_btn_;
  std::unique_ptr<QPushButton> switch_pose_btn_;
  std::unique_ptr<QLabel> current_state_label_;

  // WBC UI Elements
  std::unique_ptr<QWidget> whole_body_container_;  // Container that shows/hides based on mode
  std::unique_ptr<QVBoxLayout> whole_body_layout_;
  std::unique_ptr<QGridLayout> upper_button_layout_;
  std::unique_ptr<QHBoxLayout> body_control_layout_;

  std::unique_ptr<QLabel> base_label_;
  std::unique_ptr<QLabel> bimanual_label_;
  std::unique_ptr<QLabel> left_arm_label_;
  std::unique_ptr<QLabel> right_arm_label_;
  std::unique_ptr<QLabel> home_pose_label_;

  std::unique_ptr<SwitchButton> base_switch_;
  std::unique_ptr<SwitchButton> bimanual_switch_;
  std::unique_ptr<SwitchButton> left_arm_switch_;
  std::unique_ptr<SwitchButton> right_arm_switch_;
  std::unique_ptr<SwitchButton> home_pose_switch_;

  std::unique_ptr<QLabel> body_label_;
  std::unique_ptr<QComboBox> body_combo_box_;
  std::unique_ptr<QLabel> head_label_;
  std::unique_ptr<QComboBox> head_combo_box_;

  // Current state tracking
  int32_t current_command_ = 0;
  std::string current_state_ = "HOLD";

  // WBC state tracking
  CapabilityState capability_state_;
  CurrentWholeBodyState current_whole_body_state_;

  // Switch pose button state
  bool switch_pose_pressed_ = false;
  std::unique_ptr<QTimer> reset_timer_;
};

} // namespace control_rviz_panels

#endif // CONTROL_RVIZ_PANELS_OCS2_FSM_PANEL_HPP
