#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "robot_comm/queued_bus.h"
#include "robot_comm/serial.h"

namespace robot_comm
{
    /**
     * A bus over a serial port (usb_rs485:// and usb_serial://): its own IO thread writes one request at a time and
     * collects the reply, so both submit() / poll() and transact() work from any thread. A read or write error marks
     * the bus Failed (the adapter was unplugged, ...); the IO thread then tries to reopen the port every reopen
     * interval (default 1 s) and the bus is ready again once that works (the adapter is back).
     */
    class SerialBus final : public QueuedBus
    {
    public:
        SerialBus(std::string name, std::unique_ptr<SerialPort> port, SerialConfig config);
        ~SerialBus() override;
        SerialBus(const SerialBus&) = delete;
        SerialBus& operator=(const SerialBus&) = delete;

        /** Open the port and start the IO thread; the bus is ready afterwards. */
        bool start(std::string& error);
        /** How often a failed bus tries to reopen its port. Call before start(). */
        void setReopenInterval(std::chrono::milliseconds interval) { reopen_interval_ = interval; }

    protected:
        void wake() override;

    private:
        void run();
        /** False if the port broke (the bus is Failed then). */
        bool transfer(const Request& request);
        bool reopen();

        const std::unique_ptr<SerialPort> port_;
        const SerialConfig config_;
        std::mutex wake_mutex_;
        std::condition_variable wake_cv_;
        bool pending_ = false;
        bool stop_ = false;
        bool broken_ = false;  ///< IO thread only: the port failed and is to be reopened
        std::chrono::milliseconds reopen_interval_{1000};
        std::thread thread_;
    };

    /** Register usb_rs485:// and usb_serial:// with BusHub (PosixSerialPort). Safe to call more than once. */
    void registerSerialBuses();
}  // namespace robot_comm
