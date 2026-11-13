// LibmodbusRtuMaster against a small RTU slave on the other end of a pty.
#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include "robot_comm_modbus/libmodbus_rtu_master.h"

namespace
{
    uint16_t crc16(const uint8_t* data, std::size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (std::size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
            }
        }
        return crc;
    }

    // A one-slave Modbus RTU device: registers 0..0x1FF, FC03/04/06/10.
    class PtySlave
    {
    public:
        PtySlave(uint8_t slave_id, bool acknowledge_writes) : id_(slave_id), ack_writes_(acknowledge_writes)
        {
            master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
            EXPECT_GE(master_fd_, 0);
            EXPECT_EQ(grantpt(master_fd_), 0);
            EXPECT_EQ(unlockpt(master_fd_), 0);
            device_ = ptsname(master_fd_);
            thread_ = std::thread([this] { run(); });
        }
        ~PtySlave()
        {
            stop_ = true;
            thread_.join();
            ::close(master_fd_);
        }
        const std::string& device() const { return device_; }
        std::map<uint16_t, uint16_t> registers;
        std::atomic<int> requests{0};

    private:
        void run()
        {
            std::vector<uint8_t> buffer;
            while (!stop_)
            {
                pollfd pfd{master_fd_, POLLIN, 0};
                if (poll(&pfd, 1, 5) <= 0)
                {
                    continue;
                }
                uint8_t chunk[64];
                const ssize_t n = ::read(master_fd_, chunk, sizeof(chunk));
                if (n <= 0)
                {
                    continue;
                }
                buffer.insert(buffer.end(), chunk, chunk + n);
                while (handleOne(buffer))
                {
                }
            }
        }

        std::size_t frameLength(const std::vector<uint8_t>& b) const
        {
            if (b.size() < 2)
            {
                return 0;
            }
            if (b[1] == 0x10)
            {
                return b.size() < 7 ? 0 : 9 + b[6];
            }
            return 8;
        }

        bool handleOne(std::vector<uint8_t>& b)
        {
            const std::size_t length = frameLength(b);
            if (length == 0 || b.size() < length)
            {
                return false;
            }
            const std::vector<uint8_t> req(b.begin(), b.begin() + length);
            b.erase(b.begin(), b.begin() + length);
            ++requests;
            if (req[0] != id_)
            {
                return true;
            }

            const uint16_t addr = static_cast<uint16_t>((req[2] << 8) | req[3]);
            std::vector<uint8_t> rsp{id_, req[1]};
            switch (req[1])
            {
            case 0x03:
            case 0x04:
            {
                const int count = (req[4] << 8) | req[5];
                rsp.push_back(static_cast<uint8_t>(count * 2));
                for (int i = 0; i < count; ++i)
                {
                    const uint16_t v = registers[static_cast<uint16_t>(addr + i)];
                    rsp.push_back(static_cast<uint8_t>(v >> 8));
                    rsp.push_back(static_cast<uint8_t>(v & 0xFF));
                }
                break;
            }
            case 0x06:
                registers[addr] = static_cast<uint16_t>((req[4] << 8) | req[5]);
                rsp.assign(req.begin(), req.begin() + 6);
                break;
            case 0x10:
            {
                const int count = (req[4] << 8) | req[5];
                for (int i = 0; i < count; ++i)
                {
                    registers[static_cast<uint16_t>(addr + i)] =
                        static_cast<uint16_t>((req[7 + 2 * i] << 8) | req[8 + 2 * i]);
                }
                if (!ack_writes_)
                {
                    return true;
                }
                rsp.assign(req.begin() + 1, req.begin() + 6);
                rsp.insert(rsp.begin(), id_);
                break;
            }
            default:
                return true;
            }
            const uint16_t crc = crc16(rsp.data(), rsp.size());
            rsp.push_back(static_cast<uint8_t>(crc & 0xFF));
            rsp.push_back(static_cast<uint8_t>(crc >> 8));
            const ssize_t written = ::write(master_fd_, rsp.data(), rsp.size());
            (void)written;
            return true;
        }

        uint8_t id_;
        bool ack_writes_;
        int master_fd_ = -1;
        std::string device_;
        std::atomic<bool> stop_{false};
        std::thread thread_;
    };

    std::unique_ptr<robot_comm::LibmodbusRtuMaster> makeMaster(const PtySlave& slave, int slave_id)
    {
        auto master = std::make_unique<robot_comm::LibmodbusRtuMaster>(slave.device(), 115200, slave_id, 'N', 8, 1,
                                                                        false);
        master->setTimeouts(200, 100);
        return master;
    }
}  // namespace

TEST(LibmodbusRtuMaster, reads_holding_and_input_registers)
{
    PtySlave slave(3, true);
    slave.registers = {{0x10, 0x1234}, {0x11, 0xBEEF}};
    auto master_ptr = makeMaster(slave, 3);
    auto& master = *master_ptr;
    ASSERT_TRUE(master.connect()) << master.getLastError();

    uint16_t out[2] = {};
    ASSERT_EQ(master.readHoldingRegisters(0x10, 2, out), 2) << master.getLastError();
    EXPECT_EQ(out[0], 0x1234);
    EXPECT_EQ(out[1], 0xBEEF);

    uint16_t in[1] = {};
    ASSERT_EQ(master.readInputRegisters(0x11, 1, in), 1) << master.getLastError();
    EXPECT_EQ(in[0], 0xBEEF);
}

TEST(LibmodbusRtuMaster, writes_single_and_multiple_registers)
{
    PtySlave slave(5, true);
    auto master_ptr = makeMaster(slave, 5);
    auto& master = *master_ptr;
    ASSERT_TRUE(master.connect()) << master.getLastError();

    EXPECT_TRUE(master.writeRegister(0x20, 0x00FF));
    const uint16_t values[3] = {1, 2, 3};
    EXPECT_EQ(master.writeRegisters(0x30, 3, values), 3) << master.getLastError();
    EXPECT_EQ(slave.registers[0x20], 0x00FF);
    EXPECT_EQ(slave.registers[0x31], 2);
    EXPECT_EQ(slave.registers[0x32], 3);
}

TEST(LibmodbusRtuMaster, a_write_without_acknowledge_fails_normally_but_succeeds_as_no_ack)
{
    PtySlave slave(9, false);
    auto master_ptr = makeMaster(slave, 9);
    auto& master = *master_ptr;
    ASSERT_TRUE(master.connect()) << master.getLastError();

    const uint16_t values[2] = {0x0100, 0x00B1};
    EXPECT_EQ(master.writeRegisters(0x03E8, 2, values), -1);
    master.flush();

    const auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(master.writeRegistersNoAck(0x03E8, 2, values)) << master.getLastError();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, std::chrono::milliseconds(150));  // waits ~30 ms, not the 200 ms response timeout
    EXPECT_EQ(slave.registers[0x03E8], 0x0100);
}

TEST(LibmodbusRtuMaster, a_silent_slave_is_an_error_with_a_reason)
{
    PtySlave slave(1, true);
    auto master_ptr = makeMaster(slave, 2);
    auto& master = *master_ptr;  // another slave id: nobody answers
    ASSERT_TRUE(master.connect()) << master.getLastError();
    uint16_t out = 0;
    EXPECT_EQ(master.readHoldingRegisters(0, 1, &out), -1);
    EXPECT_FALSE(master.getLastError().empty());
}

TEST(LibmodbusRtuMaster, refuses_io_when_not_connected_and_reports_why_connect_failed)
{
    robot_comm::LibmodbusRtuMaster master("/dev/does-not-exist", 115200, 1, 'N', 8, 1, false);
    uint16_t out = 0;
    EXPECT_FALSE(master.isConnected());
    EXPECT_EQ(master.readHoldingRegisters(0, 1, &out), -1);
    EXPECT_FALSE(master.writeRegister(0, 1));
    EXPECT_FALSE(master.connect());
    EXPECT_NE(master.getLastError().find("/dev/does-not-exist"), std::string::npos);
}

TEST(LibmodbusRtuMaster, disconnect_and_reconnect_work)
{
    PtySlave slave(4, true);
    slave.registers = {{0, 42}};
    auto master_ptr = makeMaster(slave, 4);
    auto& master = *master_ptr;
    ASSERT_TRUE(master.connect());
    master.disconnect();
    EXPECT_FALSE(master.isConnected());
    ASSERT_TRUE(master.connect()) << master.getLastError();
    uint16_t out = 0;
    EXPECT_EQ(master.readHoldingRegisters(0, 1, &out), 1);
    EXPECT_EQ(out, 42);
}
