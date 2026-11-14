//
// Dexterous hand: open/close commands that pick or blend two HOME configurations.
//   target_command: Int32, 0 close / 1 open (HOME configuration close_config / open_config)
//   target_percent: Float64 0..1, blends the two configurations joint by joint (0 close, 1 open)
// Only effective in MOVEJ; the commands set the MOVEJ target without switching states.
//

#include <algorithm>
#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "EndEffector.h"

namespace end_effector_controller
{
    namespace
    {
        using robot_controller_core::FSMStateName;

        class HandEndEffector final : public EndEffector
        {
        public:
            void declareParameters(EndEffectorController& c) override
            {
                auto declare = c.declarer();
                close_config_ = declare("target_command_close_config", static_cast<int32_t>(1));
                open_config_ = declare("target_command_open_config", static_cast<int32_t>(0));
                hold_joint_names_ = declare("target_command_hold_joints", std::vector<std::string>{});
                buildHoldMask(c);
                if (close_config_ < 0 || open_config_ < 0)
                {
                    RCLCPP_ERROR(c.node()->get_logger(),
                                 "target_command_close_config (%d) and target_command_open_config (%d) must be >= 0; "
                                 "target_command and target_percent will be ignored",
                                 close_config_, open_config_);
                }

                RCLCPP_INFO(c.node()->get_logger(), "Target command enabled: close_config=%d, open_config=%d",
                            close_config_, open_config_);
                if (!hold_joint_names_.empty())
                {
                    std::string held;
                    for (const auto& joint_name : hold_joint_names_)
                    {
                        if (!held.empty())
                        {
                            held += ", ";
                        }
                        held += joint_name;
                    }
                    RCLCPP_INFO(c.node()->get_logger(),
                                "Target command hold joints (unchanged by switch/percent): %s", held.c_str());
                }
            }

            void setupCommands(EndEffectorController& c) override { setupHandCommands(c); }

            void onAfterFsmStarted(EndEffectorController&) override { is_active_.store(true); }
            // The interface loans are released after deactivation; the hand callbacks must not read them.
            void onDeactivate() override { is_active_.store(false); }

        private:
            void buildHoldMask(EndEffectorController& c)
            {
                const auto& joints = c.joints();
                hold_mask_.assign(joints.size(), false);
                for (const auto& hold_joint : hold_joint_names_)
                {
                    const auto it = std::find(joints.begin(), joints.end(), hold_joint);
                    if (it == joints.end())
                    {
                        RCLCPP_WARN(c.node()->get_logger(),
                                    "target_command_hold_joints: joint '%s' not in controller joints list",
                                    hold_joint.c_str());
                        continue;
                    }
                    hold_mask_[static_cast<size_t>(std::distance(joints.begin(), it))] = true;
                }
            }

            // Joints listed in target_command_hold_joints keep their current position when a command arrives.
            void applyHoldOverrides(EndEffectorController& c, std::vector<double>& target) const
            {
                if (hold_mask_.empty())
                {
                    return;
                }
                if (target.size() != hold_mask_.size())
                {
                    RCLCPP_WARN_THROTTLE(c.node()->get_logger(), *c.node()->get_clock(), 5000,
                                         "target_command_hold_joints ignored: configuration has %zu values but the "
                                         "controller has %zu joints",
                                         target.size(), hold_mask_.size());
                    return;
                }
                auto& interfaces = c.interfaces();
                for (size_t i = 0; i < target.size(); ++i)
                {
                    if (!hold_mask_[i])
                    {
                        continue;
                    }
                    if (i < interfaces.joint_position_state_interface_.size())
                    {
                        const auto value = interfaces.joint_position_state_interface_[i].get().get_optional();
                        if (value.has_value())
                        {
                            target[i] = value.value();
                            continue;
                        }
                    }
                    if (i < interfaces.last_sent_joint_positions_.size())
                    {
                        target[i] = interfaces.last_sent_joint_positions_[i];
                    }
                }
            }

            // Only effective in MOVEJ: sets the MOVEJ target without switching states.
            void setupHandCommands(EndEffectorController& c)
            {
                EndEffectorController* controller = &c;

                const std::string command_topic = c.name() + "/target_command";
                target_command_subscription_ = c.node()->create_subscription<std_msgs::msg::Int32>(
                    command_topic, rclcpp::QoS(10),
                    [this, controller](const std_msgs::msg::Int32::SharedPtr msg)
                    {
                        auto& c = *controller;
                        auto logger = c.node()->get_logger();
                        if (!is_active_.load() || !c.currentState() || c.currentState()->state_name != FSMStateName::MOVEJ)
                        {
                            RCLCPP_DEBUG(logger, "Target command received but ignored (current state is not MOVEJ)");
                            return;
                        }

                        // Map the command value to a HOME configuration index
                        int32_t config_index = -1;
                        if (msg->data == 0)
                        {
                            config_index = close_config_;
                        }
                        else if (msg->data == 1)
                        {
                            config_index = open_config_;
                        }
                        else
                        {
                            RCLCPP_WARN(logger, "Invalid target_command value: %d (expected 0 or 1)", msg->data);
                            return;
                        }
                        if (config_index < 0)
                        {
                            RCLCPP_WARN(logger, "Invalid configuration index: %d (must be >= 0)", config_index);
                            return;
                        }

                        if (c.states().home && c.states().movej)
                        {
                            std::vector<double> target_config =
                                c.states().home->getConfiguration(static_cast<size_t>(config_index));
                            if (!target_config.empty())
                            {
                                applyHoldOverrides(c, target_config);
                                c.states().movej->setTargetPosition(target_config);
                                RCLCPP_DEBUG(logger,
                                             "Target command received: %d, setting MOVEJ target to configuration %d",
                                             msg->data, config_index);
                            }
                            else
                            {
                                RCLCPP_WARN(logger, "Configuration %d not available or invalid", config_index);
                            }
                        }
                    });
                RCLCPP_DEBUG(c.node()->get_logger(),
                             "Subscribed to target_command topic: %s (close_config=%d, open_config=%d)",
                             command_topic.c_str(), close_config_, open_config_);

                // Percentage: blend close and open configuration joint by joint, 0.0 = close, 1.0 = open.
                const std::string percent_topic = c.name() + "/target_percent";
                target_percent_subscription_ = c.node()->create_subscription<std_msgs::msg::Float64>(
                    percent_topic, rclcpp::QoS(10),
                    [this, controller](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        auto& c = *controller;
                        auto logger = c.node()->get_logger();
                        if (!is_active_.load() || !c.currentState() || c.currentState()->state_name != FSMStateName::MOVEJ)
                        {
                            RCLCPP_DEBUG(logger, "target_percent received but ignored (current state is not MOVEJ)");
                            return;
                        }

                        double percent = msg->data;
                        if (percent < 0.0 || percent > 1.0)
                        {
                            RCLCPP_WARN(logger, "target_percent %.3f out of range [0.0, 1.0], clamping", percent);
                            percent = std::clamp(percent, 0.0, 1.0);
                        }
                        if (!c.states().home || !c.states().movej || close_config_ < 0 || open_config_ < 0)
                        {
                            return;
                        }

                        const std::vector<double> close_config =
                            c.states().home->getConfiguration(static_cast<size_t>(close_config_));
                        const std::vector<double> open_config =
                            c.states().home->getConfiguration(static_cast<size_t>(open_config_));
                        if (close_config.empty() || open_config.empty())
                        {
                            RCLCPP_WARN_THROTTLE(logger, *c.node()->get_clock(), 5000,
                                                 "target_percent: close_config(%d) or open_config(%d) not available",
                                                 close_config_, open_config_);
                            return;
                        }
                        if (close_config.size() != open_config.size())
                        {
                            RCLCPP_WARN(logger,
                                        "target_percent: close_config size(%zu) != open_config size(%zu), ignoring",
                                        close_config.size(), open_config.size());
                            return;
                        }

                        std::vector<double> target(close_config.size());
                        for (size_t i = 0; i < target.size(); ++i)
                        {
                            target[i] = close_config[i] + percent * (open_config[i] - close_config[i]);
                        }
                        applyHoldOverrides(c, target);
                        c.states().movej->setTargetPosition(target);
                        RCLCPP_DEBUG(logger, "target_percent %.1f%% -> MOVEJ target set (%zu joints)",
                                     percent * 100.0, target.size());
                    });
                RCLCPP_DEBUG(c.node()->get_logger(),
                             "Subscribed to target_percent topic: %s (0.0=close_config[%d], 1.0=open_config[%d])",
                             percent_topic.c_str(), close_config_, open_config_);
            }

            std::atomic<bool> is_active_{false};
            int32_t close_config_{1};
            int32_t open_config_{0};
            std::vector<std::string> hold_joint_names_;
            std::vector<bool> hold_mask_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_command_subscription_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_percent_subscription_;
        };
    } // namespace

    std::unique_ptr<EndEffector> makeHandEndEffector() { return std::make_unique<HandEndEffector>(); }
} // namespace end_effector_controller
