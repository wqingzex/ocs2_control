#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot_comm/serial.h"  // Notice

namespace robot_comm
{
    /**
     * A blocking Modbus master talking to one slave. Drivers use this interface; robot_comm_modbus has the libmodbus RTU
     * implementation (LibmodbusRtuMaster) and a mock for tests. Not thread-safe: one transaction at a time.
     */
    class ModbusMaster
    {
    public:
        virtual ~ModbusMaster() = default;

        virtual bool connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

        /// Function 0x03. Returns the number of registers read, or -1.
        virtual int readHoldingRegisters(uint16_t addr, int count, uint16_t* dest) = 0;
        /// Function 0x04. Returns the number of registers read, or -1.
        virtual int readInputRegisters(uint16_t addr, int count, uint16_t* dest) = 0;
        /// Function 0x06.
        virtual bool writeRegister(uint16_t addr, uint16_t value) = 0;
        /// Function 0x10. Returns the number of registers written, or -1.
        virtual int writeRegisters(uint16_t addr, int count, const uint16_t* src) = 0;
        /**
         * Function 0x10 without waiting for the acknowledgement: a frame that went out but got no answer counts as
         * success (some slaves, e.g. the Jodell RG75, never acknowledge a write).
         */
        virtual bool writeRegistersNoAck(uint16_t addr, int count, const uint16_t* src) = 0;

        /// Drop bytes left over from a failed transaction.
        virtual void flush() = 0;
        /// Takes effect on the next connect(); a value <= 0 means no timeout.
        virtual void setTimeouts(int response_timeout_ms, int byte_timeout_ms) = 0;
        virtual void setDebug(bool debug) = 0;

        virtual std::string getLastError() const = 0;
        /// Hints collected while connecting (USB latency setting, ...), handed over once.
        virtual std::vector<Notice> takeNotices() = 0;

        // For error messages.
        virtual const std::string& getSerialPort() const = 0;
        virtual uint32_t getBaudrate() const = 0;
        virtual int getSlaveId() const = 0;
    };
} // namespace robot_comm
