#pragma once

#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "robot_comm/bus.h"

namespace robot_comm
{
    /**
     * The client bookkeeping every bus needs, so the rules live in one place:
     *  - one request in flight on the line at a time; requests of different clients go out in the order submitted;
     *  - each client has at most one request queued or in flight, and only sees its own completions;
     *  - transact() waits for the outcome, and is refused on the thread that pumps the bus (it would wait forever);
     *  - when the bus stops being ready (arm disconnected) or fails, everything queued or in flight completes with
     *    CompletionStatus::Error.
     *
     * A subclass moves the bytes. Its pump (an IO thread, or the arm driver's read() / write()) calls takeNext() for
     * the next request and finish() with its outcome. wake() is called whenever a request has been queued.
     */
    class QueuedBus : public Bus
    {
    public:
        explicit QueuedBus(std::string name, bool ready = false);
        ~QueuedBus() override = default;

        const std::string& name() const override { return name_; }
        BusState state() const override;
        BusStats stats() const override;

        ClientId attach() override;
        void detach(ClientId client) override;

        bool submit(ClientId client, std::vector<uint8_t> frame, ReplySpec reply, uint32_t tag,
                    std::string& error) override;
        std::optional<Completion> poll(ClientId client) override;
        Completion transact(ClientId client, std::vector<uint8_t> frame, ReplySpec reply,
                            std::chrono::milliseconds queue_timeout) override;

    protected:
        struct Request
        {
            ClientId client = 0;
            uint32_t tag = 0;
            std::vector<uint8_t> frame;
            ReplySpec reply;
            bool sync = false;
        };

        /** The next request to put on the line; it is in flight until finish(). nullopt: one is in flight, or none queued. */
        std::optional<Request> takeNext();
        /** The outcome of the request in flight (its tag is filled in here). */
        void finish(CompletionStatus status, std::vector<uint8_t> reply = {}, std::string error = {});
        bool hasInFlight() const;
        /** Ready: Idle / Busy. Not ready: submit() is refused and everything pending completes with Error. */
        void setReady(bool ready, const std::string& reason = "bus not ready");
        /** The line is broken; like setReady(false) but the state is Failed. */
        void setFailed(const std::string& reason);
        /** transact() from this thread is refused (the bus is pumped by it). */
        void setPumpThread(std::thread::id id);

        /** A request was queued. Called without the lock held. */
        virtual void wake() {}

    private:
        struct ClientState
        {
            bool outstanding = false;
            std::deque<Completion> done;
            std::optional<Completion> sync_result;
        };

        void failAllLocked(const std::string& reason);
        void deliverLocked(const Request& request, Completion completion);

        const std::string name_;
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::map<ClientId, ClientState> clients_;
        std::deque<Request> queue_;
        std::optional<Request> in_flight_;
        bool ready_ = false;
        bool failed_ = false;
        BusStats stats_;
        ClientId next_client_ = 1;
        std::thread::id pump_thread_;
    };
}  // namespace robot_comm
