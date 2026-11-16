#include "end_effector_hardware/bus/bus_device.h"

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "end_effector_hardware/bus/params.h"

#include "end_effector_hardware/bus/jodell_rg75.h"
#include "end_effector_hardware/bus/linkerhand_l6.h"
#include "end_effector_hardware/bus/xhand1.h"

namespace end_effector_hardware
{
    namespace
    {
        rclcpp::Clock& throttleClock()
        {
            static rclcpp::Clock clock(RCL_STEADY_TIME);
            return clock;
        }
    }  // namespace

    BusDevice::BusDevice(std::string model, rclcpp::Logger logger)
        : logger_(std::move(logger)), model_(std::move(model))
    {
    }

    void BusDevice::attach(std::shared_ptr<robot_comm::BusClient> bus, robot_comm::IoMode mode)
    {
        bus_ = std::move(bus);
        mode_ = mode;
    }

    void BusDevice::start()
    {
        outstanding_ = false;
        ready_ = false;
        failure_.clear();
        online_ = true;
        last_good_ = std::chrono::steady_clock::now();
        if (bus_)
        {
            while (bus_->poll())
            {
                // drop what an earlier run left behind
            }
        }
        reset();
    }

    void BusDevice::collect()
    {
        if (!bus_) return;
        if (mode_ == robot_comm::IoMode::Async)
        {
            while (auto completion = bus_->poll())
            {
                outstanding_ = false;
                deliver(completion->tag, *completion);
            }
        }
        if (online_ && std::chrono::steady_clock::now() - last_good_ > offline_after_)
        {
            online_ = false;
            RCLCPP_ERROR(logger_, "%s on %s: no good reply for %lld ms (HW-004 offline)", model_.c_str(),
                         bus_->name().text().c_str(), static_cast<long long>(offline_after_.count()));
        }
    }

    void BusDevice::step()
    {
        if (!bus_ || failed() || outstanding_) return;
        const auto state = bus_->state();
        if (state == robot_comm::BusState::NotReady || state == robot_comm::BusState::Failed) return;
        auto request = next();
        if (!request) return;
        if (mode_ == robot_comm::IoMode::Sync)
        {
            const auto completion = bus_->transact(std::move(request->frame), std::move(request->reply));
            deliver(request->kind, completion);
            return;
        }
        std::string error;
        if (bus_->submit(std::move(request->frame), std::move(request->reply), request->kind, error))
        {
            outstanding_ = true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(logger_, throttleClock(), 2000, "%s: %s", model_.c_str(), error.c_str());
        }
    }

    void BusDevice::deliver(uint32_t kind, const robot_comm::Completion& completion)
    {
        if (failed()) return;
        handle(kind, completion);
    }

    void BusDevice::markReady()
    {
        if (!ready_)
        {
            RCLCPP_INFO(logger_, "%s on %s: ready", model_.c_str(), bus_ ? bus_->name().text().c_str() : "?");
        }
        ready_ = true;
    }

    void BusDevice::markFailed(const std::string& why)
    {
        failure_ = why;
        RCLCPP_ERROR(logger_, "%s on %s: %s", model_.c_str(), bus_ ? bus_->name().text().c_str() : "?", why.c_str());
    }

    void BusDevice::noteReply(bool good)
    {
        if (!good) return;
        last_good_ = std::chrono::steady_clock::now();
        if (!online_)
        {
            online_ = true;
            RCLCPP_INFO(logger_, "%s on %s: back online", model_.c_str(), bus_ ? bus_->name().text().c_str() : "?");
        }
    }

    bool BusDevice::configureOffline(const Params& params, std::string& error)
    {
        long ms = offline_after_.count();
        if (!params::integer(params, "offline_after_ms", 1, 60000, ms, error)) return false;
        offline_after_ = std::chrono::milliseconds(ms);
        return true;
    }

    std::unique_ptr<BusDevice> makeBusDevice(const std::string& device, rclcpp::Logger logger, std::string& error)
    {
        if (device == "jodell_rg75") return std::make_unique<JodellRg75>(logger);
        if (device == "linkerhand_l6") return std::make_unique<LinkerHandL6>(logger);
        if (device == "xhand1") return std::make_unique<XHand1>(logger);
        error = "unknown device '" + device + "' (known: jodell_rg75, linkerhand_l6, xhand1)";
        return nullptr;
    }
}  // namespace end_effector_hardware
