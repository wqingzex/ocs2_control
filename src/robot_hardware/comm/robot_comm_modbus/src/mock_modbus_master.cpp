#include "robot_comm_modbus/mock_modbus_master.h"

namespace robot_comm
{
    bool MockModbusMaster::connect()
    {
        if (fail_connect_)
        {
            last_error_ = "mock connect failure";
            return false;
        }
        connected_ = true;
        last_error_.clear();
        return true;
    }

    uint16_t MockModbusMaster::reg(uint16_t addr) const
    {
        const auto it = registers_.find(addr);
        return it == registers_.end() ? 0 : it->second;
    }

    int MockModbusMaster::read(uint16_t addr, int count, uint16_t* dest)
    {
        if (!connected_ || fail_io_ || count <= 0)
        {
            last_error_ = connected_ ? "mock io failure" : "Modbus context not initialized";
            return -1;
        }
        for (int i = 0; i < count; ++i)
        {
            dest[i] = reg(static_cast<uint16_t>(addr + i));
        }
        return count;
    }

    int MockModbusMaster::readHoldingRegisters(uint16_t addr, int count, uint16_t* dest)
    {
        return read(addr, count, dest);
    }

    int MockModbusMaster::readInputRegisters(uint16_t addr, int count, uint16_t* dest)
    {
        return read(addr, count, dest);
    }

    void MockModbusMaster::store(uint16_t addr, int count, const uint16_t* src, bool no_ack)
    {
        for (int i = 0; i < count; ++i)
        {
            registers_[static_cast<uint16_t>(addr + i)] = src[i];
        }
        writes_.push_back({addr, std::vector<uint16_t>(src, src + count), no_ack});
    }

    bool MockModbusMaster::writeRegister(uint16_t addr, uint16_t value)
    {
        if (!connected_ || fail_io_)
        {
            last_error_ = connected_ ? "mock io failure" : "Modbus context not initialized";
            return false;
        }
        store(addr, 1, &value, false);
        return true;
    }

    int MockModbusMaster::writeRegisters(uint16_t addr, int count, const uint16_t* src)
    {
        if (!connected_ || fail_io_ || count <= 0)
        {
            last_error_ = connected_ ? "mock io failure" : "Modbus context not initialized";
            return -1;
        }
        store(addr, count, src, false);
        return count;
    }

    bool MockModbusMaster::writeRegistersNoAck(uint16_t addr, int count, const uint16_t* src)
    {
        if (!connected_ || fail_io_ || count <= 0)
        {
            last_error_ = connected_ ? "mock io failure" : "Modbus context not initialized";
            return false;
        }
        store(addr, count, src, true);
        return true;
    }

    std::vector<Notice> MockModbusMaster::takeNotices()
    {
        std::vector<Notice> out;
        out.swap(pending_notices_);
        return out;
    }
} // namespace robot_comm
