#include "end_effector_hardware/bus/kwr75_sensor.h"

#include <cmath>
#include <cstring>

#include <pluginlib/class_list_macros.hpp>

#include "end_effector_hardware/bus/params.h"
#include "robot_comm_serial/serial_bus.h"

namespace end_effector_hardware
{
    using hardware_interface::CallbackReturn;
    using hardware_interface::return_type;

    namespace kwr75
    {
        namespace
        {
            constexpr uint8_t kPoll = 0x48;
            constexpr uint8_t kStream = 0x49;  ///< a sensor left in stream mode by another program still frames like this
            constexpr uint8_t kMarker = 0xAA;
            constexpr double kMaxForce = 2.0e3;   ///< kgf; larger is a corrupted frame
            constexpr double kMaxTorque = 2.0e2;  ///< kgf*m
            constexpr std::array<const char*, kAxes> kInterfaces{"force.x",  "force.y",  "force.z",
                                                                 "torque.x", "torque.y", "torque.z"};
        }  // namespace

        std::vector<uint8_t> pollRequest() { return {kPoll, kMarker, 0x0D, 0x0A}; }

        bool parseFrame(const std::vector<uint8_t>& frame, std::array<double, kAxes>& out)
        {
            if (frame.size() != kFrameLength || (frame[0] != kPoll && frame[0] != kStream) || frame[1] != kMarker ||
                frame[kFrameLength - 2] != 0x0D || frame[kFrameLength - 1] != 0x0A)
            {
                return false;
            }
            std::array<double, kAxes> values{};
            for (std::size_t axis = 0; axis < kAxes; ++axis)
            {
                // Low byte first, as the host (x86 / aarch64 are little endian).
                float v = 0.0F;
                std::memcpy(&v, frame.data() + 2 + 4 * axis, sizeof(v));
                if (!std::isfinite(v) || std::fabs(v) > (axis < 3 ? kMaxForce : kMaxTorque)) return false;
                values[axis] = v;
            }
            out = values;
            return true;
        }
    }  // namespace kwr75

    CallbackReturn Kwr75Sensor::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
        const auto fail = [this](const std::string& why) {
            RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), why.c_str());
            return CallbackReturn::ERROR;
        };
        if (info_.sensors.size() != 1) return fail("expects exactly one <sensor>");
        const auto& sensor = info_.sensors.front();
        if (sensor.state_interfaces.size() != kwr75::kAxes) return fail("the sensor must declare 6 state interfaces");
        for (std::size_t i = 0; i < kwr75::kAxes; ++i)
        {
            if (sensor.state_interfaces[i].name != kwr75::kInterfaces[i])
            {
                return fail("state interface " + std::to_string(i) + " must be " + kwr75::kInterfaces[i]);
            }
        }
        sensor_name_ = sensor.name;

        const Params& p = info_.hardware_parameters;
        const auto value = [&p](const char* key, const std::string& fallback = "") {
            const auto it = p.find(key);
            return it == p.end() ? fallback : it->second;
        };
        bus_name_ = value("bus");
        if (bus_name_.empty()) return fail("parameter 'bus' is required (e.g. usb_rs485:///dev/ttyUSB4?baud=2500000)");
        std::string error;
        if (!robot_comm::parseBusName(bus_name_, error)) return fail(error);
        wrench_topic_ = value("wrench_topic");
        frame_id_ = value("frame_id", frame_id_);
        const std::string si = value("convert_to_si", "true");
        if (si != "true" && si != "false") return fail("convert_to_si must be true or false, got '" + si + "'");
        double gravity = 9.80665;
        long offline_ms = offline_after_.count();
        if (!params::number(p, "gravity", gravity, error) ||
            !params::integer(p, "offline_after_ms", 1, 60000, offline_ms, error))
        {
            return fail(error);
        }
        scale_ = si == "true" ? gravity : 1.0;
        offline_after_ = std::chrono::milliseconds(offline_ms);

        RCLCPP_INFO(get_logger(), "%s: KWR75 on %s", info_.name.c_str(), bus_name_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Kwr75Sensor::on_configure(const rclcpp_lifecycle::State&)
    {
        robot_comm::registerSerialBuses();
        std::string error;
        bus_ = robot_comm::BusHub::instance().connect(bus_name_, error);
        if (!bus_)
        {
            RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), error.c_str());
            return CallbackReturn::ERROR;
        }
        const auto node = get_node();
        if (node && !wrench_topic_.empty() && !publisher_)
        {
            publisher_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic_, rclcpp::SensorDataQoS());
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Kwr75Sensor::on_activate(const rclcpp_lifecycle::State&)
    {
        // Like EndEffectorHardware: an arm's bus may only become ready after this component is active.
        if (bus_->state() == robot_comm::BusState::Failed)
        {
            RCLCPP_ERROR(get_logger(), "%s: bus %s failed", info_.name.c_str(), bus_name_.c_str());
            return CallbackReturn::ERROR;
        }
        wrench_.fill(0.0);
        online_ = false;
        reported_offline_ = false;
        last_good_ = std::chrono::steady_clock::now();  // grace period for the first frame
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Kwr75Sensor::on_deactivate(const rclcpp_lifecycle::State&)
    {
        wrench_.fill(0.0);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Kwr75Sensor::on_cleanup(const rclcpp_lifecycle::State&)
    {
        bus_.reset();  // the last client of a USB bus closes the port
        outstanding_ = false;
        publisher_.reset();
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> Kwr75Sensor::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
        for (std::size_t i = 0; i < kwr75::kAxes; ++i)
        {
            interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(sensor_name_, kwr75::kInterfaces[i], &wrench_[i]));
        }
        return interfaces;
    }

    void Kwr75Sensor::take(const std::vector<uint8_t>& frame)
    {
        std::array<double, kwr75::kAxes> raw{};
        if (!kwr75::parseFrame(frame, raw))
        {
            ++bad_frames_;
            return;
        }
        for (std::size_t i = 0; i < kwr75::kAxes; ++i) wrench_[i] = raw[i] * scale_;
        last_good_ = std::chrono::steady_clock::now();
        if (!online_)
        {
            RCLCPP_INFO(get_logger(), "%s: KWR75 on %s is %s", info_.name.c_str(), bus_name_.c_str(),
                        reported_offline_ ? "back" : "online");
        }
        online_ = true;
        reported_offline_ = false;
    }

    return_type Kwr75Sensor::read(const rclcpp::Time& time, const rclcpp::Duration&)
    {
        while (auto done = bus_->poll())
        {
            outstanding_ = false;
            if (done->status == robot_comm::CompletionStatus::Ok) take(done->reply);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_good_ > offline_after_)
        {
            wrench_.fill(0.0);  // a stale wrench is worse than none
            // A bus that is not ready yet (arm not connected) is not the sensor's fault: report once it is.
            if (!reported_offline_ && bus_->state() != robot_comm::BusState::NotReady)
            {
                RCLCPP_WARN(get_logger(), "%s: no valid KWR75 frame on %s for %lld ms (%llu bad frames); output 0",
                            info_.name.c_str(), bus_name_.c_str(), static_cast<long long>(offline_after_.count()),
                            static_cast<unsigned long long>(bad_frames_));
                reported_offline_ = true;
            }
            online_ = false;
        }

        if (!outstanding_)
        {
            robot_comm::ReplySpec reply;
            reply.frame_length = [](const std::vector<uint8_t>&) { return kwr75::kFrameLength; };
            reply.max_size = kwr75::kFrameLength;
            reply.timeout = std::chrono::milliseconds(20);
            std::string error;
            outstanding_ = bus_->submit(kwr75::pollRequest(), reply, 0, error);  // refused while the arm is away
        }

        publish(time);
        return return_type::OK;
    }

    void Kwr75Sensor::publish(const rclcpp::Time& time)
    {
        if (!publisher_) return;
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = time;
        msg.header.frame_id = frame_id_;
        msg.wrench.force.x = wrench_[0];
        msg.wrench.force.y = wrench_[1];
        msg.wrench.force.z = wrench_[2];
        msg.wrench.torque.x = wrench_[3];
        msg.wrench.torque.y = wrench_[4];
        msg.wrench.torque.z = wrench_[5];
        publisher_->publish(msg);
    }
}  // namespace end_effector_hardware

PLUGINLIB_EXPORT_CLASS(end_effector_hardware::Kwr75Sensor, hardware_interface::SensorInterface)
