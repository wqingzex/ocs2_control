#include "marvin_hardware/tool/arm_bus.h"

#include <utility>

namespace marvin_hardware
{
    namespace
    {
        constexpr int kMaxChunksPerPull = 8;  ///< SDK reads per read(): enough for one reply, bounded in time
    }  // namespace

    /** A QueuedBus whose queue ArmBuses drives. */
    class ArmBuses::PortBus final : public robot_comm::QueuedBus
    {
    public:
        explicit PortBus(std::string name) : QueuedBus(std::move(name), false) {}
        using QueuedBus::finish;
        using QueuedBus::setPumpThread;
        using QueuedBus::setReady;
        using QueuedBus::takeNext;
    };

    ArmBuses::ArmBuses(std::string host, std::string side, Send485Func send, GetChDataFunc get)
        : side_(std::move(side)), send_(send), get_(get)
    {
        const std::string arm = host + "/" + side_;
        ports_ = {{
            {"arm_rs485://" + arm + "/port1", COM1_CHANNEL, nullptr},
            {"arm_rs485://" + arm + "/port2", COM2_CHANNEL, nullptr},
            {"arm_canfd://" + arm, CAN_CHANNEL, nullptr},
        }};
        for (auto& port : ports_) port.bus = std::make_shared<PortBus>(port.name);
    }

    ArmBuses::~ArmBuses()
    {
        setConnected(false);
        provisions_.clear();
    }

    bool ArmBuses::provide(std::string& error)
    {
        provisions_.clear();
        for (auto& port : ports_)
        {
            auto provision = robot_comm::BusHub::instance().provide(port.name, port.bus, error);
            if (!provision)
            {
                provisions_.clear();
                return false;
            }
            provisions_.push_back(std::move(provision));
        }
        return true;
    }

    void ArmBuses::setConnected(bool connected)
    {
        in_flight_.reset();  // when disconnecting, the bus fails it together with the queue
        for (auto& port : ports_) port.bus->setReady(connected, "arm disconnected");
    }

    void ArmBuses::complete(robot_comm::CompletionStatus status, const std::string& error)
    {
        InFlight done = std::move(*in_flight_);
        in_flight_.reset();
        done.port->bus->finish(status, std::move(done.received), error);
    }

    void ArmBuses::push()
    {
        if (!pump_thread_set_)
        {
            for (auto& port : ports_) port.bus->setPumpThread(std::this_thread::get_id());
            pump_thread_set_ = true;
        }
        if (in_flight_) return;
        for (std::size_t i = 0; i < ports_.size(); ++i)
        {
            Port& port = ports_[(next_port_ + i) % ports_.size()];
            auto request = port.bus->takeNext();
            if (!request) continue;
            next_port_ = (next_port_ + i + 1) % ports_.size();  // the other ports go first next time
            std::vector<uint8_t> frame = request->frame;
            const auto now = std::chrono::steady_clock::now();
            in_flight_ = InFlight{&port, request->reply, now, now, {}};
            if (!send_ || !send_(frame.data(), static_cast<long>(frame.size()), port.channel))
            {
                complete(robot_comm::CompletionStatus::Error, port.name + ": OnSetChData refused the frame");
                return;
            }
            if (!request->reply.expect_reply) complete(robot_comm::CompletionStatus::Ok);
            return;
        }
    }

    void ArmBuses::pull()
    {
        if (!in_flight_) return;  // only read the SDK buffer for our own request
        unsigned char buffer[256];
        for (int i = 0; i < kMaxChunksPerPull && in_flight_; ++i)
        {
            long channel = 0;
            const long n = get_ ? get_(buffer, &channel) : 0;
            if (n <= 0) break;
            if (channel != in_flight_->port->channel) continue;  // another port of this arm: not ours
            auto& received = in_flight_->received;
            received.insert(received.end(), buffer, buffer + n);
            in_flight_->last_byte = std::chrono::steady_clock::now();
            const auto& spec = in_flight_->reply;
            const std::size_t expected = spec.frame_length ? spec.frame_length(received) : 0;
            if (expected > 0 && received.size() >= expected)
            {
                received.resize(expected);
                complete(robot_comm::CompletionStatus::Ok);
                return;
            }
            if (received.size() >= spec.max_size)
            {
                complete(robot_comm::CompletionStatus::Ok);
                return;
            }
        }
        if (!in_flight_) return;
        const auto now = std::chrono::steady_clock::now();
        const auto& spec = in_flight_->reply;
        if (!spec.frame_length && !in_flight_->received.empty() && now - in_flight_->last_byte >= spec.inter_byte_gap)
        {
            complete(robot_comm::CompletionStatus::Ok);
            return;
        }
        if (now - in_flight_->sent >= spec.timeout)
        {
            complete(robot_comm::CompletionStatus::Timeout, in_flight_->port->name + ": no complete reply within " +
                                                                std::to_string(spec.timeout.count()) + " ms");
        }
    }
}  // namespace marvin_hardware
