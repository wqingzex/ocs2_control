#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "robot_comm/bus.h"
#include "robot_comm/bus_name.h"

namespace robot_comm
{
    namespace detail
    {
        /** Where the bus of one physical line lives; shared by its clients (and its provider, for an arm's bus). */
        struct BusSlot
        {
            std::mutex mutex;
            std::shared_ptr<Bus> bus;  ///< null: not opened / not provided (yet)
            std::string options;       ///< usb_*: the options it was opened with
        };
    }  // namespace detail

    /**
     * What a device holds: its own view of a bus. Only this client's completions come back. For an arm's bus the
     * client exists before the arm connects (state() NotReady) and starts working once the arm provides the bus.
     */
    class BusClient
    {
    public:
        BusClient(BusName name, std::shared_ptr<detail::BusSlot> slot);
        ~BusClient();
        BusClient(const BusClient&) = delete;
        BusClient& operator=(const BusClient&) = delete;

        const BusName& name() const { return name_; }
        BusState state() const;
        BusStats stats() const;

        bool submit(std::vector<uint8_t> frame, ReplySpec reply, uint32_t tag, std::string& error);
        std::optional<Completion> poll();
        Completion transact(std::vector<uint8_t> frame, ReplySpec reply,
                            std::chrono::milliseconds queue_timeout = std::chrono::milliseconds(1000));

    private:
        /** The bus currently in the slot, attached; null when there is none. */
        std::shared_ptr<Bus> current() const;

        const BusName name_;
        const std::shared_ptr<detail::BusSlot> slot_;
        mutable std::mutex mutex_;
        mutable std::shared_ptr<Bus> bus_;  ///< the bus this client is attached to
        mutable ClientId id_ = 0;
        // An async request is pending on bus_ (submitted, its completion not polled yet). When bus_ goes away or is
        // replaced meanwhile, its completion would never come: poll() hands out this Error completion instead.
        mutable bool pending_ = false;
        mutable uint32_t pending_tag_ = 0;
        mutable std::optional<Completion> orphaned_;
    };

    /**
     * The process-wide registry of buses, by name. A device asks for a bus by name (connect); the same physical line
     * gives the same Bus. USB buses are opened here on first use, by the factory registered for their kind
     * (robot_comm_serial registers usb_rs485 / usb_serial). An arm's flange buses exist only while the arm driver
     * provides them. Lives in the shared library robot_comm_bus, so every plugin in the process sees the same one.
     */
    class BusHub
    {
    public:
        using Factory = std::function<std::shared_ptr<Bus>(const BusName& name, std::string& error)>;

        /** Withdraws a provided bus when destroyed. */
        class Provision
        {
        public:
            explicit Provision(std::shared_ptr<detail::BusSlot> slot) : slot_(std::move(slot)) {}
            ~Provision();
            Provision(const Provision&) = delete;
            Provision& operator=(const Provision&) = delete;

        private:
            std::shared_ptr<detail::BusSlot> slot_;
        };

        static BusHub& instance();

        void registerFactory(const std::string& kind, Factory factory);
        bool hasFactory(const std::string& kind) const;

        /** A client of the named bus. Null (and `error`) when the name is malformed or the bus cannot be opened. */
        std::shared_ptr<BusClient> connect(const std::string& name, std::string& error);

        /** Make `bus` the bus of `name` until the returned Provision is destroyed. Null if it is already provided. */
        std::unique_ptr<Provision> provide(const std::string& name, std::shared_ptr<Bus> bus, std::string& error);

    private:
        std::shared_ptr<detail::BusSlot> slot(const std::string& key);

        mutable std::mutex mutex_;
        std::map<std::string, Factory> factories_;
        std::map<std::string, std::weak_ptr<detail::BusSlot>> slots_;
    };
}  // namespace robot_comm
