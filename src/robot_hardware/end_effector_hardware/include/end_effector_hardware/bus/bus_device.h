#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/logger.hpp>

#include "robot_comm/bus.h"
#include "robot_comm/bus_hub.h"

namespace end_effector_hardware
{
    using Params = std::unordered_map<std::string, std::string>;

    /** One joint of a device, as exported to ros2_control. */
    struct DeviceJoint
    {
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
        double command = std::numeric_limits<double>::quiet_NaN();
    };

    /**
     * An end effector that talks over a robot_comm bus (USB-RS485, an arm's flange port, ...), written as a state
     * machine so it never waits in the control loop: each write() it may put one request on the bus, each read() it
     * takes the replies. Initialization runs the same way (an arm's bus only moves while the arm's read() / write()
     * run, so a device cannot wait for it in on_activate). Until the device has read its first state it sends no
     * commands and its state interfaces keep their initial values.
     *
     * A subclass says what to send (next()) and what a reply means (handle()); it calls markReady() once its state is
     * known, markFailed() when initialization cannot succeed, and noteReply() for every reply so the base can report
     * the device offline.
     */
    class BusDevice
    {
    public:
        BusDevice(std::string model, rclcpp::Logger logger);
        virtual ~BusDevice() = default;

        const std::string& model() const { return model_; }

        /** Take the URDF joints and hardware parameters. False (and `error`) if they do not fit this device. */
        virtual bool configure(const std::vector<std::string>& joint_names, const Params& params, std::string& error) = 0;
        /** Runtime torque / velocity ratios in [0, 1] (devices without them ignore this). */
        virtual void setDynamics(double /*torque*/, double /*velocity*/) {}

        void attach(std::shared_ptr<robot_comm::BusClient> bus, robot_comm::IoMode mode);
        /** Start (again) from the beginning of initialization. */
        void start();
        /** In read(): take finished requests. */
        void collect();
        /** In write(): put the next request on the bus. */
        void step();

        std::vector<DeviceJoint>& joints() { return joints_; }
        bool ready() const { return ready_; }
        bool failed() const { return !failure_.empty(); }
        const std::string& failure() const { return failure_; }
        /** False after `offline_after` without a good reply (the device was unplugged / powered off). */
        bool online() const { return online_; }
        robot_comm::BusClient* bus() const { return bus_.get(); }

    protected:
        struct Request
        {
            std::vector<uint8_t> frame;
            robot_comm::ReplySpec reply;
            uint32_t kind = 0;  ///< the subclass's own request kind, handed back to handle()
        };

        /** What to send now, or nothing. Called only when no request of this device is outstanding. */
        virtual std::optional<Request> next() = 0;
        /** The outcome of a request of the given kind. */
        virtual void handle(uint32_t kind, const robot_comm::Completion& completion) = 0;
        /** Back to the initial state (start()). */
        virtual void reset() = 0;

        void markReady();
        void markFailed(const std::string& why);
        void noteReply(bool good);
        /** The optional parameter offline_after_ms (how long without a good reply before the device is offline). */
        bool configureOffline(const Params& params, std::string& error);

        rclcpp::Logger logger_;
        std::vector<DeviceJoint> joints_;
        std::chrono::milliseconds offline_after_{2000};

    private:
        void deliver(uint32_t kind, const robot_comm::Completion& completion);

        const std::string model_;
        std::shared_ptr<robot_comm::BusClient> bus_;
        robot_comm::IoMode mode_ = robot_comm::IoMode::Async;
        bool outstanding_ = false;
        uint32_t outstanding_kind_ = 0;
        bool ready_ = false;
        std::string failure_;
        bool online_ = true;
        std::chrono::steady_clock::time_point last_good_;
    };

    /** The devices EndEffectorHardware can drive, by the URDF `device` parameter. */
    std::unique_ptr<BusDevice> makeBusDevice(const std::string& device, rclcpp::Logger logger, std::string& error);
}  // namespace end_effector_hardware
