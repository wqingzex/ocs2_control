#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "robot_comm/modbus.h"

namespace robot_comm
{
    /**
     * An in-memory slave for tests. Holding and input registers share one map (0 where nothing was set), writes land in
     * it and are logged, and every call can be made to fail.
     */
    class MockModbusMaster final : public ModbusMaster
    {
    public:
        struct Write
        {
            uint16_t addr;
            std::vector<uint16_t> values;
            bool no_ack;
        };

        explicit MockModbusMaster(int slave_id = 1) : slave_id_(slave_id) {}

        bool connect() override;
        void disconnect() override { connected_ = false; }
        bool isConnected() const override { return connected_; }

        int readHoldingRegisters(uint16_t addr, int count, uint16_t* dest) override;
        int readInputRegisters(uint16_t addr, int count, uint16_t* dest) override;
        bool writeRegister(uint16_t addr, uint16_t value) override;
        int writeRegisters(uint16_t addr, int count, const uint16_t* src) override;
        bool writeRegistersNoAck(uint16_t addr, int count, const uint16_t* src) override;

        void flush() override { ++flush_count_; }
        void setTimeouts(int, int) override {}
        void setDebug(bool) override {}

        std::string getLastError() const override { return last_error_; }
        std::vector<Notice> takeNotices() override;

        const std::string& getSerialPort() const override { return port_; }
        uint32_t getBaudrate() const override { return 115200; }
        int getSlaveId() const override { return slave_id_; }

        // --- test controls ---
        void setRegister(uint16_t addr, uint16_t value) { registers_[addr] = value; }
        uint16_t reg(uint16_t addr) const;
        const std::vector<Write>& writes() const { return writes_; }
        int flushCount() const { return flush_count_; }
        void failConnect(bool fail) { fail_connect_ = fail; }
        void failIo(bool fail) { fail_io_ = fail; }
        void addNotice(const Notice& notice) { pending_notices_.push_back(notice); }

    private:
        int read(uint16_t addr, int count, uint16_t* dest);
        void store(uint16_t addr, int count, const uint16_t* src, bool no_ack);

        int slave_id_;
        std::string port_ = "mock";
        bool connected_ = false;
        bool fail_connect_ = false;
        bool fail_io_ = false;
        int flush_count_ = 0;
        std::string last_error_;
        std::map<uint16_t, uint16_t> registers_;
        std::vector<Write> writes_;
        std::vector<Notice> pending_notices_;
    };
} // namespace robot_comm
