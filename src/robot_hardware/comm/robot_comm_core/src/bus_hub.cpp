#include "robot_comm/bus_hub.h"

#include <utility>

namespace robot_comm
{
    namespace
    {
        /** "baud=115200&low_latency=1": what a USB bus was opened with (two names for one port must agree on it). */
        std::string optionsOf(const BusName& name)
        {
            const std::string text = name.text();
            const std::size_t q = text.find('?');
            return q == std::string::npos ? std::string() : text.substr(q + 1);
        }

        std::string notReady(const BusName& name)
        {
            return name.text() + ": not ready" + (name.isArm() ? " (arm not connected)" : "");
        }
    }  // namespace

    // ---- BusClient -------------------------------------------------------------------------------------------------

    BusClient::BusClient(BusName name, std::shared_ptr<detail::BusSlot> slot)
        : name_(std::move(name)), slot_(std::move(slot))
    {
    }

    BusClient::~BusClient()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (bus_) bus_->detach(id_);
    }

    std::shared_ptr<Bus> BusClient::current() const
    {
        std::shared_ptr<Bus> bus;
        {
            std::lock_guard<std::mutex> slot_lock(slot_->mutex);
            bus = slot_->bus;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (bus != bus_)  // provided, withdrawn or replaced (the arm reconnected) since the last call
        {
            if (bus_ && pending_)
            {
                orphaned_ = Completion{pending_tag_, CompletionStatus::Error, {},
                                       name_.text() + ": the bus went away while the request was pending"};
                pending_ = false;
            }
            if (bus_) bus_->detach(id_);
            bus_ = bus;
            id_ = bus_ ? bus_->attach() : 0;
        }
        return bus_;
    }

    BusState BusClient::state() const
    {
        const auto bus = current();
        return bus ? bus->state() : BusState::NotReady;
    }

    BusStats BusClient::stats() const
    {
        const auto bus = current();
        return bus ? bus->stats() : BusStats{};
    }

    bool BusClient::submit(std::vector<uint8_t> frame, ReplySpec reply, uint32_t tag, std::string& error)
    {
        const auto bus = current();
        if (!bus)
        {
            error = notReady(name_);
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (bus != bus_)
        {
            error = name_.text() + ": the bus changed, try again";
            return false;
        }
        if (!bus->submit(id_, std::move(frame), std::move(reply), tag, error)) return false;
        pending_ = true;
        pending_tag_ = tag;
        return true;
    }

    std::optional<Completion> BusClient::poll()
    {
        const auto bus = current();
        std::lock_guard<std::mutex> lock(mutex_);
        if (orphaned_)
        {
            auto completion = std::move(orphaned_);
            orphaned_.reset();
            return completion;
        }
        if (!bus || bus != bus_) return std::nullopt;
        auto completion = bus->poll(id_);
        if (completion) pending_ = false;
        return completion;
    }

    Completion BusClient::transact(std::vector<uint8_t> frame, ReplySpec reply, std::chrono::milliseconds queue_timeout)
    {
        const auto bus = current();
        if (!bus)
        {
            Completion completion;
            completion.error = notReady(name_);
            return completion;
        }
        ClientId id;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            id = id_;
        }
        return bus->transact(id, std::move(frame), std::move(reply), queue_timeout);
    }

    // ---- BusHub ----------------------------------------------------------------------------------------------------

    BusHub::Provision::~Provision()
    {
        std::lock_guard<std::mutex> lock(slot_->mutex);
        slot_->bus.reset();
    }

    BusHub& BusHub::instance()
    {
        static BusHub hub;
        return hub;
    }

    void BusHub::registerFactory(const std::string& kind, Factory factory)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        factories_[kind] = std::move(factory);
    }

    bool BusHub::hasFactory(const std::string& kind) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return factories_.count(kind) != 0;
    }

    std::shared_ptr<detail::BusSlot> BusHub::slot(const std::string& key)
    {
        // The caller holds mutex_.
        auto& weak = slots_[key];
        auto slot = weak.lock();
        if (!slot)
        {
            slot = std::make_shared<detail::BusSlot>();
            weak = slot;
        }
        return slot;
    }

    std::shared_ptr<BusClient> BusHub::connect(const std::string& name, std::string& error)
    {
        const auto parsed = parseBusName(name, error);
        if (!parsed) return nullptr;
        if (parsed->scheme == "usb_can")
        {
            error = "bus name '" + name + "': usb_can interfaces are opened by the CAN hand driver, not through BusHub";
            return nullptr;
        }
        std::shared_ptr<detail::BusSlot> slot;
        Factory factory;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            slot = this->slot(parsed->key());
            if (const auto it = factories_.find(parsed->scheme); it != factories_.end()) factory = it->second;
        }
        if (parsed->isUsbSerial())
        {
            std::lock_guard<std::mutex> lock(slot->mutex);
            const std::string options = optionsOf(*parsed);
            if (slot->bus)
            {
                if (options != slot->options)
                {
                    error = "bus name '" + name + "': " + parsed->device + " is already open with " + slot->options;
                    return nullptr;
                }
            }
            else
            {
                if (!factory)
                {
                    error = "bus name '" + name + "': no driver registered for " + parsed->scheme +
                            " (call robot_comm_serial::registerSerialBuses())";
                    return nullptr;
                }
                std::string why;
                auto bus = factory(*parsed, why);
                if (!bus)
                {
                    error = "bus name '" + name + "': " + why;
                    return nullptr;
                }
                slot->bus = std::move(bus);
                slot->options = options;
            }
        }
        return std::make_shared<BusClient>(*parsed, slot);
    }

    std::unique_ptr<BusHub::Provision> BusHub::provide(const std::string& name, std::shared_ptr<Bus> bus,
                                                       std::string& error)
    {
        const auto parsed = parseBusName(name, error);
        if (!parsed) return nullptr;
        if (!bus)
        {
            error = "bus name '" + name + "': nothing to provide";
            return nullptr;
        }
        std::shared_ptr<detail::BusSlot> slot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            slot = this->slot(parsed->key());
        }
        std::lock_guard<std::mutex> lock(slot->mutex);
        if (slot->bus)
        {
            error = "bus name '" + name + "': already provided";
            return nullptr;
        }
        slot->bus = std::move(bus);
        return std::make_unique<Provision>(slot);
    }
}  // namespace robot_comm
