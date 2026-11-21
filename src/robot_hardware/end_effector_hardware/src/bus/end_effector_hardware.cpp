#include "end_effector_hardware/bus/end_effector_hardware.h"

#include <algorithm>
#include <cmath>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "end_effector_hardware/bus/params.h"
#include "robot_comm_serial/serial_bus.h"

namespace end_effector_hardware
{
    using hardware_interface::CallbackReturn;

    namespace
    {
        bool declares(const std::vector<hardware_interface::InterfaceInfo>& interfaces, const std::string& name)
        {
            return std::any_of(interfaces.begin(), interfaces.end(),
                               [&](const hardware_interface::InterfaceInfo& i) { return i.name == name; });
        }
    }  // namespace

    CallbackReturn EndEffectorHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
        const Params& p = info_.hardware_parameters;
        const auto fail = [this](const std::string& why) {
            RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), why.c_str());
            return CallbackReturn::ERROR;
        };
        const auto value = [&p](const char* key, const std::string& fallback = "") {
            const auto it = p.find(key);
            return it == p.end() ? fallback : it->second;
        };

        const std::string device = value("device");
        bus_name_ = value("bus");
        if (device.empty()) return fail("parameter 'device' is required (jodell_rg75, linkerhand_l6, xhand1)");
        if (bus_name_.empty()) return fail("parameter 'bus' is required (e.g. usb_rs485:///dev/serial/by-id/<adapter>)");
        std::string error;
        if (!robot_comm::parseBusName(bus_name_, error)) return fail(error);

        const std::string mode = value("io_mode", "async");
        if (mode != "async" && mode != "sync") return fail("io_mode must be async or sync, got '" + mode + "'");
        io_mode_ = mode == "sync" ? robot_comm::IoMode::Sync : robot_comm::IoMode::Async;
        double torque = 1.0;
        double velocity = 1.0;
        if (!params::ratio(p, "torque", torque, error) || !params::ratio(p, "velocity", velocity, error))
        {
            return fail(error);
        }
        torque_ = torque;
        velocity_ = velocity;

        device_ = makeBusDevice(device, get_logger(), error);
        if (!device_) return fail(error);
        std::vector<std::string> joints;
        for (const auto& joint : info_.joints) joints.push_back(joint.name);
        if (!device_->configure(joints, p, error)) return fail(error);
        for (const auto& joint : device_->joints())
        {
            const auto it = std::find_if(info_.joints.begin(), info_.joints.end(),
                                         [&](const auto& j) { return j.name == joint.name; });
            if (!declares(it->command_interfaces, hardware_interface::HW_IF_POSITION))
            {
                return fail("joint " + joint.name + " needs a position command interface");
            }
        }
        declareDynamicsParameters();
        RCLCPP_INFO(get_logger(), "%s: %s on %s (%s)", info_.name.c_str(), device.c_str(), bus_name_.c_str(),
                    mode.c_str());
        return CallbackReturn::SUCCESS;
    }

    void EndEffectorHardware::declareDynamicsParameters()
    {
        const auto node = get_node();
        if (!node) return;  // unit tests without a node
        if (!node->has_parameter("torque")) node->declare_parameter<double>("torque", torque_.load());
        if (!node->has_parameter("velocity")) node->declare_parameter<double>("velocity", velocity_.load());
        parameter_callback_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            double torque = torque_.load();
            double velocity = velocity_.load();
            for (const auto& parameter : ps)
            {
                if (parameter.get_name() != "torque" && parameter.get_name() != "velocity") continue;
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || parameter.as_double() < 0.0 ||
                    parameter.as_double() > 1.0)
                {
                    result.successful = false;
                    result.reason = parameter.get_name() + " must be a double in [0, 1]";
                    return result;
                }
                (parameter.get_name() == "torque" ? torque : velocity) = parameter.as_double();
            }
            torque_ = torque;
            velocity_ = velocity;
            dynamics_changed_ = true;  // applied by write(), on the control thread
            return result;
        });
    }

    CallbackReturn EndEffectorHardware::on_configure(const rclcpp_lifecycle::State&)
    {
        robot_comm::registerSerialBuses();
        std::string error;
        auto bus = robot_comm::BusHub::instance().connect(bus_name_, error);
        if (!bus)
        {
            RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), error.c_str());
            return CallbackReturn::ERROR;
        }
        device_->attach(std::move(bus), io_mode_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EndEffectorHardware::on_activate(const rclcpp_lifecycle::State&)
    {
        // An arm's bus becomes ready when the arm has connected, in the arm's on_activate, which ros2_control may run
        // after this one (in the same thread, so waiting here cannot help). The device starts initializing once it
        // is ready; until then nothing is sent and the state interfaces keep their initial values.
        const auto state = device_->bus()->state();
        if (state == robot_comm::BusState::Failed)
        {
            RCLCPP_ERROR(get_logger(), "%s: bus %s failed", info_.name.c_str(), bus_name_.c_str());
            return CallbackReturn::ERROR;
        }
        if (state == robot_comm::BusState::NotReady)
        {
            RCLCPP_INFO(get_logger(), "%s: bus %s not ready yet%s; initializing once it is", info_.name.c_str(),
                        bus_name_.c_str(), device_->bus()->name().isArm() ? " (arm not connected yet)" : "");
        }
        device_->setDynamics(torque_.load(), velocity_.load());
        dynamics_changed_ = false;
        device_->start();
        failure_reported_ = false;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EndEffectorHardware::on_deactivate(const rclcpp_lifecycle::State&)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EndEffectorHardware::on_cleanup(const rclcpp_lifecycle::State&)
    {
        device_->attach(nullptr, io_mode_);  // the last client of a USB bus closes the port
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> EndEffectorHardware::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> out;
        for (auto& joint : device_->joints())
        {
            const auto& info = *std::find_if(info_.joints.begin(), info_.joints.end(),
                                             [&](const auto& j) { return j.name == joint.name; });
            const std::pair<const char*, double*> states[] = {{hardware_interface::HW_IF_POSITION, &joint.position},
                                                              {hardware_interface::HW_IF_VELOCITY, &joint.velocity},
                                                              {hardware_interface::HW_IF_EFFORT, &joint.effort}};
            for (const auto& [name, value] : states)
            {
                if (declares(info.state_interfaces, name))
                {
                    out.push_back(std::make_shared<hardware_interface::StateInterface>(joint.name, name, value));
                }
            }
        }
        return out;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> EndEffectorHardware::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> out;
        for (auto& joint : device_->joints())
        {
            out.push_back(std::make_shared<hardware_interface::CommandInterface>(
                joint.name, hardware_interface::HW_IF_POSITION, &joint.command));
        }
        return out;
    }

    hardware_interface::return_type EndEffectorHardware::read(const rclcpp::Time&, const rclcpp::Duration&)
    {
        device_->collect();
        if (device_->failed())
        {
            if (!failure_reported_)
            {
                RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), device_->failure().c_str());
                failure_reported_ = true;
            }
            return hardware_interface::return_type::ERROR;
        }
        // A lost line (a USB adapter pulled out): the bus reopens the port by itself, the device goes offline meanwhile
        // (HW-004) and probes again when the line is back. Returning ERROR here would make ros2_control deactivate this
        // component and every controller using its interfaces (the joint state broadcaster of the whole robot), and
        // nothing activates them again.
        if (device_->bus()->state() == robot_comm::BusState::Failed)
        {
            if (!bus_failure_reported_)
            {
                RCLCPP_ERROR(get_logger(), "%s: bus %s failed: %s; the port is reopened automatically", info_.name.c_str(),
                             bus_name_.c_str(), device_->bus()->stats().last_error.c_str());
                bus_failure_reported_ = true;
            }
        }
        else if (bus_failure_reported_)
        {
            RCLCPP_INFO(get_logger(), "%s: bus %s is back", info_.name.c_str(), bus_name_.c_str());
            bus_failure_reported_ = false;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type EndEffectorHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        if (dynamics_changed_.exchange(false)) device_->setDynamics(torque_.load(), velocity_.load());
        device_->step();
        return hardware_interface::return_type::OK;
    }
}  // namespace end_effector_hardware

PLUGINLIB_EXPORT_CLASS(end_effector_hardware::EndEffectorHardware, hardware_interface::SystemInterface)
