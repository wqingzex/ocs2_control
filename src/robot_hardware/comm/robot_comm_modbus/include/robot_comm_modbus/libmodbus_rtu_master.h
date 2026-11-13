#pragma once

#include <string>
#include <vector>

#include "robot_comm/modbus.h"

typedef struct _modbus modbus_t;

namespace robot_comm
{
    /// Modbus RTU over a serial port, on libmodbus.
    class LibmodbusRtuMaster final : public ModbusMaster
    {
    public:
        /**
         * @param parity 'N', 'E' or 'O'
         * @param configure_low_latency try to set the USB adapter's latency_timer to 1 ms after connecting
         */
        LibmodbusRtuMaster(const std::string& serial_port, uint32_t baudrate, int slave_id, char parity = 'N',
                           int data_bits = 8, int stop_bits = 1, bool configure_low_latency = true);
        ~LibmodbusRtuMaster() override;
        LibmodbusRtuMaster(const LibmodbusRtuMaster&) = delete;
        LibmodbusRtuMaster& operator=(const LibmodbusRtuMaster&) = delete;

        bool connect() override;
        void disconnect() override;
        bool isConnected() const override { return modbus_ctx_ != nullptr; }

        int readHoldingRegisters(uint16_t addr, int count, uint16_t* dest) override;
        int readInputRegisters(uint16_t addr, int count, uint16_t* dest) override;
        bool writeRegister(uint16_t addr, uint16_t value) override;
        int writeRegisters(uint16_t addr, int count, const uint16_t* src) override;
        bool writeRegistersNoAck(uint16_t addr, int count, const uint16_t* src) override;

        void flush() override;
        void setTimeouts(int response_timeout_ms, int byte_timeout_ms) override;
        void setDebug(bool debug) override;

        std::string getLastError() const override;
        std::vector<Notice> takeNotices() override;

        const std::string& getSerialPort() const override { return serial_port_; }
        uint32_t getBaudrate() const override { return baudrate_; }
        int getSlaveId() const override { return slave_id_; }

    private:
        std::string serial_port_;
        uint32_t baudrate_;
        int slave_id_;
        char parity_;
        int data_bits_;
        int stop_bits_;
        bool configure_low_latency_;
        int response_timeout_ms_ = 500;
        int byte_timeout_ms_ = 100;

        modbus_t* modbus_ctx_ = nullptr;
        mutable std::string last_error_;
        std::vector<Notice> notices_;
    };
} // namespace robot_comm
