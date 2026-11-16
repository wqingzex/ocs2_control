#include "robot_comm_serial/serial_bus.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "robot_comm/bus_hub.h"
#include "robot_comm_serial/posix_serial_port.h"

namespace robot_comm
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        constexpr auto kIdlePoll = std::chrono::microseconds(200);  ///< sleep step when a port does not block
    }  // namespace

    SerialBus::SerialBus(std::string name, std::unique_ptr<SerialPort> port, SerialConfig config)
        : QueuedBus(std::move(name), false), port_(std::move(port)), config_(std::move(config))
    {
    }

    SerialBus::~SerialBus()
    {
        {
            std::lock_guard<std::mutex> lock(wake_mutex_);
            stop_ = true;
        }
        wake_cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        setReady(false, "bus closed");
        port_->close();
    }

    bool SerialBus::start(std::string& error)
    {
        if (!port_->open(config_))
        {
            error = "cannot open " + config_.device + ": " + port_->lastError();
            return false;
        }
        setReady(true);
        thread_ = std::thread(&SerialBus::run, this);
        return true;
    }

    void SerialBus::wake()
    {
        {
            std::lock_guard<std::mutex> lock(wake_mutex_);
            pending_ = true;
        }
        wake_cv_.notify_one();
    }

    void SerialBus::run()
    {
        setPumpThread(std::this_thread::get_id());
        while (true)
        {
            {
                std::unique_lock<std::mutex> lock(wake_mutex_);
                const auto woken = [this] { return pending_ || stop_; };
                if (broken_)
                {
                    wake_cv_.wait_for(lock, reopen_interval_, [this] { return stop_; });
                }
                else
                {
                    wake_cv_.wait(lock, woken);
                }
                if (stop_) return;
                pending_ = false;
            }
            if (broken_ && !reopen()) continue;
            while (auto request = takeNext())
            {
                if (!transfer(*request)) broken_ = true;
                std::lock_guard<std::mutex> lock(wake_mutex_);
                if (stop_ || broken_) break;
            }
        }
    }

    bool SerialBus::reopen()
    {
        port_->close();
        if (!port_->open(config_)) return false;
        broken_ = false;
        setReady(true);  // clears Failed; clients can submit again
        return true;
    }

    bool SerialBus::transfer(const Request& request)
    {
        port_->flushInput();  // a late reply to an earlier request must not be taken for this one
        std::string error;
        if (!writeAll(*port_, request.frame.data(), request.frame.size(), request.reply.timeout, error))
        {
            setFailed(error);  // also completes the request in flight; the state is Failed before anyone is woken
            return false;
        }
        if (!request.reply.expect_reply)
        {
            finish(CompletionStatus::Ok);
            return true;
        }

        const ReplySpec& spec = request.reply;
        const auto deadline = Clock::now() + spec.timeout;
        std::vector<uint8_t> reply;
        auto last_byte = Clock::now();
        uint8_t chunk[256];
        while (true)
        {
            const auto now = Clock::now();
            if (now >= deadline)
            {
                finish(CompletionStatus::Timeout, std::move(reply),
                       name() + ": no complete reply within " + std::to_string(spec.timeout.count()) + " ms");
                return true;
            }
            // Without a length rule the reply ends at a silence of inter_byte_gap after its first byte.
            if (!spec.frame_length && !reply.empty() && now - last_byte >= spec.inter_byte_gap)
            {
                finish(CompletionStatus::Ok, std::move(reply));
                return true;
            }
            const auto wait = std::chrono::duration_cast<std::chrono::microseconds>(
                std::min<Clock::duration>(deadline - now, spec.inter_byte_gap));
            const IoStatus ready = port_->waitReadable(wait);
            if (ready == IoStatus::Error)
            {
                error = "serial read failed: " + port_->lastError();
                setFailed(error);
                return false;
            }
            if (ready == IoStatus::WouldBlock)
            {
                std::this_thread::sleep_for(std::min<Clock::duration>(kIdlePoll, deadline - Clock::now()));
                continue;
            }
            std::size_t n = 0;
            const IoStatus got = port_->read(chunk, std::min(sizeof(chunk), spec.max_size - reply.size()), n);
            if (got == IoStatus::Error)
            {
                error = "serial read failed: " + port_->lastError();
                setFailed(error);
                return false;
            }
            if (n == 0) continue;
            reply.insert(reply.end(), chunk, chunk + n);
            last_byte = Clock::now();
            const std::size_t expected = spec.frame_length ? spec.frame_length(reply) : 0;
            if (expected > 0 && reply.size() >= expected)
            {
                reply.resize(expected);
                finish(CompletionStatus::Ok, std::move(reply));
                return true;
            }
            if (reply.size() >= spec.max_size)
            {
                finish(CompletionStatus::Ok, std::move(reply));
                return true;
            }
        }
    }

    void registerSerialBuses()
    {
        const auto factory = [](const BusName& name, std::string& error) -> std::shared_ptr<Bus> {
            SerialConfig config;
            config.device = name.device;
            config.baudrate = name.baud();
            config.low_latency = name.lowLatency();
            config.strict_baudrate = true;
            auto bus = std::make_shared<SerialBus>(name.text(), std::make_unique<PosixSerialPort>(), config);
            if (!bus->start(error)) return nullptr;
            return bus;
        };
        BusHub::instance().registerFactory("usb_rs485", factory);
        BusHub::instance().registerFactory("usb_serial", factory);
    }
}  // namespace robot_comm
