//
// Parallel gripper with optional force feedback (GripperController, one joint).
//   <joint>/position_command    Float64 absolute position, no force feedback (relative name)
//   target_command              Int32 0 close (force feedback) / 1 open
//   target_percent              Float64 0..1, 0 closed, 1 open; force feedback in the closing direction
// The closed / open positions come from the joint limits and the initial_value in robot_description: the limit
// nearest to initial_value is "closed". Only effective in MOVEJ; the commands set the MOVEJ target, so HOLD / HOME
// and the interpolation (movej_interpolation_type) come from the state machine.
//
// Force feedback: while closing, once |effort| exceeds force_threshold the target becomes
//   current + (original target - current) * force_feedback_ratio
// (0 stops where it is, 1 keeps going to the original target). Needs `effort` in state_interfaces.
//

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "EndEffector.h"

namespace end_effector_controller
{
    namespace
    {
        using robot_controller_core::FSMStateName;

        // The text of every <joint name="..."> ... </joint> element for this joint (URDF joint and ros2_control joint).
        std::vector<std::string> jointBlocks(const std::string& description, const std::string& joint)
        {
            std::vector<std::string> blocks;
            const std::string open_tag = "<joint name=\"" + joint + "\"";
            size_t pos = description.find(open_tag);
            while (pos != std::string::npos)
            {
                size_t end = description.find("</joint>", pos);
                end = (end == std::string::npos) ? description.size() : end;
                blocks.push_back(description.substr(pos, end - pos));
                pos = description.find(open_tag, end);
            }
            return blocks;
        }

        bool parseLimits(const std::string& description, const std::string& joint, double& lower, double& upper)
        {
            static const std::regex limit_tag(R"(<limit\b[^>]*>)");
            static const std::regex lower_attr(R"re(lower\s*=\s*"([^"]+)")re");
            static const std::regex upper_attr(R"re(upper\s*=\s*"([^"]+)")re");
            for (const auto& block : jointBlocks(description, joint))
            {
                std::smatch tag, lo, up;
                if (!std::regex_search(block, tag, limit_tag))
                {
                    continue;
                }
                const std::string text = tag.str();
                if (std::regex_search(text, lo, lower_attr) && std::regex_search(text, up, upper_attr))
                {
                    lower = std::stod(lo[1]);
                    upper = std::stod(up[1]);
                    return true;
                }
            }
            return false;
        }

        bool parseInitialValue(const std::string& description, const std::string& joint, double& initial)
        {
            static const std::regex initial_param(R"re(<param\s+name\s*=\s*"initial_value"\s*>\s*([^<\s]+)\s*</param>)re");
            for (const auto& block : jointBlocks(description, joint))
            {
                std::smatch match;
                if (std::regex_search(block, match, initial_param))
                {
                    initial = std::stod(match[1]);
                    return true;
                }
            }
            return false;
        }

        class GripperEndEffector final : public EndEffector
        {
        public:
            void declareParameters(joint_controller::JointController& c) override
            {
                if (c.joints().size() != 1)
                {
                    throw std::runtime_error("GripperController: the gripper controls exactly one joint, got " +
                                             std::to_string(c.joints().size()));
                }
                joint_ = c.joints().front();

                auto declare = c.declarer();
                use_effort_ = declare("use_effort_interface", true);
                force_threshold_ = declare("force_threshold", 0.1);
                force_ratio_ = declare("force_feedback_ratio", 0.5);

                const auto& description = c.robotDescription();
                double initial = 0.0;
                const bool has_initial = parseInitialValue(description, joint_, initial);
                // HOME is the initial position of the hardware unless the yaml lists its own home_N.
                if (c.states().home && c.states().home->configurationCount() == 0 && has_initial)
                {
                    c.states().home->setHomePosition({initial});
                    RCLCPP_INFO(c.node()->get_logger(), "No home_1 configured, HOME uses initial_value %.6f", initial);
                }
                if (!parseLimits(description, joint_, lower_, upper_))
                {
                    RCLCPP_ERROR(c.node()->get_logger(),
                                 "Gripper joint '%s': no <limit lower/upper> in robot_description, "
                                 "open/close commands will be ignored", joint_.c_str());
                    return;
                }
                const bool closed_is_upper = std::abs(upper_ - initial) < std::abs(lower_ - initial);
                closed_ = closed_is_upper ? upper_ : lower_;
                open_ = closed_is_upper ? lower_ : upper_;
                is_ready_ = true;
                RCLCPP_INFO(c.node()->get_logger(),
                            "Gripper '%s': closed %.6f, open %.6f (initial_value %.6f, limits [%.6f, %.6f]), "
                            "effort feedback %s (threshold %.3f, ratio %.3f)",
                            joint_.c_str(), closed_, open_, initial, lower_, upper_,
                            use_effort_ ? "requested" : "off", force_threshold_, force_ratio_);
            }

            void setupCommands(joint_controller::JointController& c) override
            {
                joint_controller::JointController* controller = &c;
                const auto node = c.node();

                const std::string position_topic = joint_ + "/position_command";   // relative: follows the namespace
                position_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
                    position_topic, 10,
                    [this, controller](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        if (!std::isfinite(msg->data))
                        {
                            RCLCPP_WARN(controller->node()->get_logger(), "Position command is NaN/Inf, ignoring");
                            return;
                        }
                        applyTarget(*controller, msg->data, false);
                    });

                command_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
                    c.name() + "/target_command", 10,
                    [this, controller](const std_msgs::msg::Int32::SharedPtr msg)
                    {
                        if (msg->data != 0 && msg->data != 1)
                        {
                            RCLCPP_WARN(controller->node()->get_logger(),
                                        "Invalid target command: %d (expected 0=close or 1=open), ignoring",
                                        msg->data);
                            return;
                        }
                        applyTarget(*controller, msg->data == 1 ? open_ : closed_, msg->data == 0);
                    });

                percent_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
                    c.name() + "/target_percent", 10,
                    [this, controller](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        double percent = msg->data;
                        if (!std::isfinite(percent))
                        {
                            RCLCPP_WARN(controller->node()->get_logger(), "target_percent is NaN/Inf, ignoring");
                            return;
                        }
                        if (percent < 0.0 || percent > 1.0)
                        {
                            RCLCPP_WARN(controller->node()->get_logger(),
                                        "target_percent %.3f out of range [0.0, 1.0], clamping", percent);
                            percent = std::clamp(percent, 0.0, 1.0);
                        }
                        const double target = closed_ + percent * (open_ - closed_);
                        const double current = currentPosition(*controller);
                        const double closing_sign = (closed_ > open_) ? 1.0 : -1.0;
                        const bool closing = std::isfinite(current) && (target - current) * closing_sign > 1e-6;
                        applyTarget(*controller, target, closing);
                    });
                RCLCPP_DEBUG(node->get_logger(), "Gripper commands: %s, %s/target_command, %s/target_percent",
                             position_topic.c_str(), c.name().c_str(), c.name().c_str());
            }

            void onAfterFsmStarted(joint_controller::JointController& c) override
            {
                has_effort_ = use_effort_ && !c.interfaces().joint_force_state_interface_.empty();
                if (use_effort_ && !has_effort_)
                {
                    RCLCPP_ERROR(c.node()->get_logger(),
                                 "use_effort_interface is true but there is no effort state interface for '%s' "
                                 "(add effort to state_interfaces); force feedback is off", joint_.c_str());
                }
                force_armed_.store(false);
                is_active_.store(true);
            }

            void onDeactivate() override
            {
                is_active_.store(false);
                force_armed_.store(false);
            }

            // Real-time thread, after MOVEJ wrote its command: apply the force feedback.
            void onCommandWritten(joint_controller::JointController& c) override
            {
                if (!force_armed_.load() || force_triggered_.load() || !has_effort_)
                {
                    return;
                }
                if (!c.currentState() || c.currentState()->state_name != FSMStateName::MOVEJ)
                {
                    return;
                }
                const double effort =
                    c.interfaces().joint_force_state_interface_.front().get().get_optional().value_or(0.0);
                if (std::abs(effort) <= force_threshold_)
                {
                    return;
                }
                const double current = currentPosition(c);
                if (!std::isfinite(current))
                {
                    return;
                }
                const double original = original_target_.load();
                const double target = current + (original - current) * force_ratio_;
                force_triggered_.store(true);
                c.states().movej->setTargetPosition({target});
                RCLCPP_INFO(c.node()->get_logger(),
                            "Force threshold triggered (closing), moving %.1f%% toward target: %.6f "
                            "(current %.6f, original target %.6f)",
                            force_ratio_ * 100.0, target, current, original);
            }

        private:
            static double currentPosition(joint_controller::JointController& c)
            {
                auto& interfaces = c.interfaces();
                if (interfaces.joint_position_state_interface_.empty())
                {
                    return std::numeric_limits<double>::quiet_NaN();
                }
                return interfaces.joint_position_state_interface_.front().get().get_optional().value_or(
                    std::numeric_limits<double>::quiet_NaN());
            }

            // Sets the MOVEJ target; ignored unless the controller is active in MOVEJ.
            void applyTarget(joint_controller::JointController& c, double position, bool with_force_feedback)
            {
                auto logger = c.node()->get_logger();
                if (!is_ready_)
                {
                    RCLCPP_WARN(logger, "Gripper limits not available, ignoring command %.6f", position);
                    return;
                }
                if (!is_active_.load() || !c.currentState() || c.currentState()->state_name != FSMStateName::MOVEJ)
                {
                    RCLCPP_DEBUG(logger, "Gripper command received but ignored (current state is not MOVEJ)");
                    return;
                }
                const double clamped = std::clamp(position, lower_, upper_);
                if (clamped != position)
                {
                    RCLCPP_WARN(logger, "Gripper command %.6f clamped to [%.6f, %.6f], using %.6f", position, lower_,
                                upper_, clamped);
                }
                original_target_.store(clamped);
                force_triggered_.store(false);
                force_armed_.store(with_force_feedback && has_effort_);
                c.states().movej->setTargetPosition({clamped});
                RCLCPP_INFO(logger, "Gripper target %.6f (%s)", clamped,
                            (with_force_feedback && has_effort_) ? "force feedback" : "no force feedback");
            }

            std::string joint_;
            bool use_effort_{true};
            double force_threshold_{0.1};
            double force_ratio_{0.5};
            double lower_{0.0};
            double upper_{0.0};
            double closed_{0.0};
            double open_{0.0};
            bool is_ready_{false};
            bool has_effort_{false};

            std::atomic<bool> is_active_{false};
            std::atomic<bool> force_armed_{false};
            std::atomic<bool> force_triggered_{false};
            std::atomic<double> original_target_{0.0};

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscription_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr percent_subscription_;
        };
    } // namespace

    std::unique_ptr<EndEffector> makeGripperEndEffector() { return std::make_unique<GripperEndEffector>(); }
} // namespace end_effector_controller
