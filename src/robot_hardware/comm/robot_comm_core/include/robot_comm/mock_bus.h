#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "robot_comm/queued_bus.h"

namespace robot_comm
{
    /**
     * A bus for tests. Each request is answered by the responder (nullopt: no reply, the request times out). With
     * auto pump (the default) a request completes as soon as it is queued; otherwise the test calls pumpOne().
     */
    class MockBus : public QueuedBus
    {
    public:
        using Responder = std::function<std::optional<std::vector<uint8_t>>(const std::vector<uint8_t>& request)>;

        explicit MockBus(std::string name = "mock://bus", bool ready = true) : QueuedBus(std::move(name), ready) {}

        void setResponder(Responder responder)
        {
            std::lock_guard<std::mutex> lock(mock_mutex_);
            responder_ = std::move(responder);
        }
        void setAutoPump(bool on) { auto_pump_ = on; }

        /** Put the next queued request on the "line" and answer it. False: nothing to do. */
        bool pumpOne()
        {
            auto request = takeNext();
            if (!request) return false;
            Responder responder;
            {
                std::lock_guard<std::mutex> lock(mock_mutex_);
                sent_.push_back(request->frame);
                responder = responder_;
            }
            if (!request->reply.expect_reply)
            {
                finish(CompletionStatus::Ok);
                return true;
            }
            std::optional<std::vector<uint8_t>> reply = responder ? responder(request->frame) : std::nullopt;
            if (reply)
            {
                finish(CompletionStatus::Ok, std::move(*reply));
            }
            else
            {
                finish(CompletionStatus::Timeout, {}, name() + ": no reply");
            }
            return true;
        }

        std::vector<std::vector<uint8_t>> sent() const
        {
            std::lock_guard<std::mutex> lock(mock_mutex_);
            return sent_;
        }

        using QueuedBus::setFailed;
        using QueuedBus::setPumpThread;
        using QueuedBus::setReady;

    protected:
        void wake() override
        {
            if (!auto_pump_) return;
            // finish() calls wake() again on the same thread (re-entry is fine); another thread waits its turn.
            std::lock_guard<std::recursive_mutex> lock(pump_mutex_);
            while (pumpOne())
            {
            }
        }

    private:
        mutable std::mutex mock_mutex_;
        std::recursive_mutex pump_mutex_;
        Responder responder_;
        std::vector<std::vector<uint8_t>> sent_;
        std::atomic<bool> auto_pump_{true};
    };
}  // namespace robot_comm
