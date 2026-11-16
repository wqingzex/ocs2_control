#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "marvin_hardware/tool/modbus_io.h"
#include "robot_comm/bus_hub.h"
#include "robot_comm/queued_bus.h"

namespace marvin_hardware
{
    /**
     * The flange ports of one arm as robot_comm buses, so end effector components (end_effector_hardware) can talk
     * to devices plugged into the arm:
     *
     *   arm_rs485://<ip>/<left|right>/port1   SDK channel COM1
     *   arm_rs485://<ip>/<left|right>/port2   SDK channel COM2
     *   arm_canfd://<ip>/<left|right>         SDK channel CANFD
     *
     * The SDK has one send / receive buffer per arm, so all ports of an arm share one request in flight. Driven
     * from MarvinHardware: push() at the end of write() puts the next request on the wire, pull() at the start of
     * read() collects the reply (it reads the SDK buffer only while a request is in flight). Ready while the arm is
     * connected; asynchronous use only from the control thread (see QueuedBus::transact).
     */
    class ArmBuses
    {
    public:
        ArmBuses(std::string host, std::string side, Send485Func send, GetChDataFunc get);
        ~ArmBuses();

        /** Register the three buses in BusHub. */
        bool provide(std::string& error);
        /** Connected arm: ready. Disconnected: everything pending fails. */
        void setConnected(bool connected);
        void push();
        void pull();

        const std::string& side() const { return side_; }

    private:
        class PortBus;
        struct Port
        {
            std::string name;
            long channel;
            std::shared_ptr<PortBus> bus;
        };
        struct InFlight
        {
            Port* port = nullptr;
            robot_comm::ReplySpec reply;
            std::chrono::steady_clock::time_point sent;
            std::chrono::steady_clock::time_point last_byte;
            std::vector<uint8_t> received;
        };

        void complete(robot_comm::CompletionStatus status, const std::string& error = {});

        const std::string side_;
        Send485Func send_;
        GetChDataFunc get_;
        std::array<Port, 3> ports_;
        std::vector<std::unique_ptr<robot_comm::BusHub::Provision>> provisions_;
        std::optional<InFlight> in_flight_;
        std::size_t next_port_ = 0;
        bool pump_thread_set_ = false;
    };
}  // namespace marvin_hardware
