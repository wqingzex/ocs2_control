#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace robot_comm
{
    /**
     * A bus is one physical half-duplex line (a USB-RS485 adapter, the RS485 port at an arm's flange, ...): one
     * request frame at a time, each answered by at most one reply frame. Devices do not use a Bus directly; they hold
     * a BusClient (bus_hub.h), which only sees its own replies. Frames are opaque bytes: protocol framing (Modbus RTU
     * CRC, a vendor protocol) is built above the bus.
     */

    /** How to recognise the reply to a request. */
    struct ReplySpec
    {
        bool expect_reply = true;  ///< false: the request completes as soon as it is sent
        /**
         * Given the bytes received so far, the total length of the reply, or 0 while that is not known yet. Without
         * it the reply ends at max_size bytes or after inter_byte_gap of silence.
         */
        std::function<std::size_t(const std::vector<uint8_t>&)> frame_length;
        std::size_t max_size = 256;
        std::chrono::milliseconds timeout{50};            ///< from the request being sent to the whole reply
        std::chrono::microseconds inter_byte_gap{2000};  ///< end of a reply of unknown length
    };

    enum class BusState
    {
        NotReady,  ///< not open yet, or (an arm's bus) the arm is not connected
        Idle,      ///< nothing in flight
        Busy,      ///< a request is in flight or queued
        Failed     ///< the line failed (port lost, ...); see BusStats::last_error
    };

    enum class CompletionStatus
    {
        Ok,       ///< reply received (or sent, for expect_reply = false)
        Timeout,  ///< no complete reply within ReplySpec::timeout; Completion::reply holds what did arrive
        Error,    ///< the request could not be sent, or the bus failed / went away while it was in flight
    };

    struct Completion
    {
        uint32_t tag = 0;  ///< the tag passed to submit()
        CompletionStatus status = CompletionStatus::Error;
        std::vector<uint8_t> reply;
        std::string error;
    };

    struct BusStats
    {
        uint64_t sent = 0;
        uint64_t ok = 0;
        uint64_t timeouts = 0;
        uint64_t errors = 0;
        std::string last_error;
    };

    /** Whether a device waits for its replies (Sync) or collects them on a later cycle (Async, the default). */
    enum class IoMode
    {
        Async,
        Sync
    };

    using ClientId = uint32_t;

    /**
     * One physical line. Implementations: SerialBus (USB adapters, own IO thread), the arm-flange buses of the arm
     * drivers (pumped from the arm's read() / write()), MockBus (tests). They share the queueing and per-client
     * bookkeeping in QueuedBus (queued_bus.h).
     */
    class Bus
    {
    public:
        virtual ~Bus() = default;

        virtual const std::string& name() const = 0;
        virtual BusState state() const = 0;
        virtual BusStats stats() const = 0;

        virtual ClientId attach() = 0;
        virtual void detach(ClientId client) = 0;

        /**
         * Queue a request. Never blocks. Returns false (and `error`) when this client already has a request queued or
         * in flight, or the bus is not ready / failed. Requests of different clients go out one after another.
         */
        virtual bool submit(ClientId client, std::vector<uint8_t> frame, ReplySpec reply, uint32_t tag,
                            std::string& error) = 0;
        /** This client's next finished request, if any. Never blocks. */
        virtual std::optional<Completion> poll(ClientId client) = 0;
        /**
         * Send and wait for the outcome. Refused (CompletionStatus::Error) on a bus that is pumped by the calling
         * thread: the reply could never arrive. `queue_timeout` bounds the wait for earlier requests of other clients.
         */
        virtual Completion transact(ClientId client, std::vector<uint8_t> frame, ReplySpec reply,
                                    std::chrono::milliseconds queue_timeout) = 0;
    };
}  // namespace robot_comm
