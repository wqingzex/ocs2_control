#include "robot_comm/queued_bus.h"

#include <utility>

namespace robot_comm
{
    QueuedBus::QueuedBus(std::string name, bool ready) : name_(std::move(name)), ready_(ready) {}

    BusState QueuedBus::state() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (failed_) return BusState::Failed;
        if (!ready_) return BusState::NotReady;
        return (in_flight_ || !queue_.empty()) ? BusState::Busy : BusState::Idle;
    }

    BusStats QueuedBus::stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    ClientId QueuedBus::attach()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const ClientId id = next_client_++;
        clients_[id];
        return id;
    }

    void QueuedBus::detach(ClientId client)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        clients_.erase(client);
        for (auto it = queue_.begin(); it != queue_.end();)
        {
            it = (it->client == client) ? queue_.erase(it) : it + 1;
        }
        // A request of this client still in flight finishes normally; its completion is dropped.
        cv_.notify_all();
    }

    bool QueuedBus::submit(ClientId client, std::vector<uint8_t> frame, ReplySpec reply, uint32_t tag,
                           std::string& error)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = clients_.find(client);
            if (it == clients_.end())
            {
                error = name_ + ": unknown client";
                return false;
            }
            if (failed_)
            {
                error = name_ + ": " + stats_.last_error;
                return false;
            }
            if (!ready_)
            {
                error = name_ + ": not ready";
                return false;
            }
            if (it->second.outstanding)
            {
                error = name_ + ": previous request still pending";
                return false;
            }
            it->second.outstanding = true;
            queue_.push_back(Request{client, tag, std::move(frame), std::move(reply), false});
        }
        wake();
        return true;
    }

    std::optional<Completion> QueuedBus::poll(ClientId client)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = clients_.find(client);
        if (it == clients_.end() || it->second.done.empty()) return std::nullopt;
        Completion completion = std::move(it->second.done.front());
        it->second.done.pop_front();
        return completion;
    }

    Completion QueuedBus::transact(ClientId client, std::vector<uint8_t> frame, ReplySpec reply,
                                   std::chrono::milliseconds queue_timeout)
    {
        const auto error = [](std::string text) {
            Completion completion;
            completion.status = CompletionStatus::Error;
            completion.error = std::move(text);
            return completion;
        };
        const auto reply_timeout = reply.timeout;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (pump_thread_ != std::thread::id() && pump_thread_ == std::this_thread::get_id())
            {
                return error(name_ + ": synchronous call from the thread that pumps this bus; its reply could only "
                                     "arrive on a later cycle of this thread (use io_mode async, or run the device "
                                     "component in its own thread)");
            }
            const auto deadline = std::chrono::steady_clock::now() + queue_timeout;
            const auto idle = [&] {
                const auto it = clients_.find(client);
                return it == clients_.end() || !it->second.outstanding || failed_ || !ready_;
            };
            if (!cv_.wait_until(lock, deadline, idle)) return error(name_ + ": previous request still pending");
            const auto it = clients_.find(client);
            if (it == clients_.end()) return error(name_ + ": unknown client");
            if (failed_) return error(name_ + ": " + stats_.last_error);
            if (!ready_) return error(name_ + ": not ready");
            it->second.outstanding = true;
            it->second.sync_result.reset();
            queue_.push_back(Request{client, 0, std::move(frame), std::move(reply), true});
        }
        wake();
        std::unique_lock<std::mutex> lock(mutex_);
        // Earlier requests of other clients go first; each is bounded by its own reply timeout.
        const auto deadline = std::chrono::steady_clock::now() + queue_timeout + reply_timeout;
        const auto done = [&] {
            const auto it = clients_.find(client);
            return it == clients_.end() || it->second.sync_result.has_value();
        };
        if (!cv_.wait_until(lock, deadline, done)) return error(name_ + ": no outcome within the queue and reply timeouts");
        const auto it = clients_.find(client);
        if (it == clients_.end()) return error(name_ + ": client detached");
        Completion completion = std::move(*it->second.sync_result);
        it->second.sync_result.reset();
        return completion;
    }

    std::optional<QueuedBus::Request> QueuedBus::takeNext()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (in_flight_ || queue_.empty() || !ready_ || failed_) return std::nullopt;
        in_flight_ = std::move(queue_.front());
        queue_.pop_front();
        ++stats_.sent;
        return *in_flight_;
    }

    void QueuedBus::finish(CompletionStatus status, std::vector<uint8_t> reply, std::string error)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!in_flight_) return;
            const Request request = std::move(*in_flight_);
            in_flight_.reset();
            switch (status)
            {
                case CompletionStatus::Ok: ++stats_.ok; break;
                case CompletionStatus::Timeout: ++stats_.timeouts; break;
                case CompletionStatus::Error:
                    ++stats_.errors;
                    stats_.last_error = error;
                    break;
            }
            deliverLocked(request, Completion{request.tag, status, std::move(reply), std::move(error)});
            cv_.notify_all();
        }
        wake();  // the next queued request can go
    }

    bool QueuedBus::hasInFlight() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return in_flight_.has_value();
    }

    void QueuedBus::setReady(bool ready, const std::string& reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = ready;
        if (ready)
        {
            failed_ = false;
        }
        else
        {
            failAllLocked(reason);
        }
        cv_.notify_all();
    }

    void QueuedBus::setFailed(const std::string& reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true;
        stats_.last_error = reason;
        failAllLocked(reason);
        cv_.notify_all();
    }

    void QueuedBus::setPumpThread(std::thread::id id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pump_thread_ = id;
    }

    void QueuedBus::failAllLocked(const std::string& reason)
    {
        if (in_flight_)
        {
            const Request request = std::move(*in_flight_);
            in_flight_.reset();
            ++stats_.errors;
            deliverLocked(request, Completion{request.tag, CompletionStatus::Error, {}, name_ + ": " + reason});
        }
        while (!queue_.empty())
        {
            const Request request = std::move(queue_.front());
            queue_.pop_front();
            ++stats_.errors;
            deliverLocked(request, Completion{request.tag, CompletionStatus::Error, {}, name_ + ": " + reason});
        }
    }

    void QueuedBus::deliverLocked(const Request& request, Completion completion)
    {
        const auto it = clients_.find(request.client);
        if (it == clients_.end()) return;  // detached meanwhile
        it->second.outstanding = false;
        if (request.sync)
        {
            it->second.sync_result = std::move(completion);
        }
        else
        {
            it->second.done.push_back(std::move(completion));
        }
    }
}  // namespace robot_comm
