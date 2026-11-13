#include <gtest/gtest.h>

#include "robot_comm_modbus/mock_modbus_master.h"

using robot_comm::MockModbusMaster;

TEST(MockModbusMaster, reads_nothing_until_connected)
{
    MockModbusMaster master;
    uint16_t out[2] = {};
    EXPECT_EQ(master.readHoldingRegisters(0, 2, out), -1);
    EXPECT_FALSE(master.writeRegister(0, 1));
    EXPECT_FALSE(master.isConnected());
}

TEST(MockModbusMaster, returns_the_registers_that_were_set_and_zero_elsewhere)
{
    MockModbusMaster master;
    ASSERT_TRUE(master.connect());
    master.setRegister(0x10, 0xABCD);
    uint16_t out[3] = {1, 1, 1};
    ASSERT_EQ(master.readInputRegisters(0x10, 3, out), 3);
    EXPECT_EQ(out[0], 0xABCD);
    EXPECT_EQ(out[1], 0);
    EXPECT_EQ(out[2], 0);
}

TEST(MockModbusMaster, logs_writes_and_makes_them_readable)
{
    MockModbusMaster master;
    ASSERT_TRUE(master.connect());
    const uint16_t values[2] = {7, 8};
    ASSERT_EQ(master.writeRegisters(4, 2, values), 2);
    ASSERT_TRUE(master.writeRegistersNoAck(9, 2, values));
    ASSERT_TRUE(master.writeRegister(20, 5));

    ASSERT_EQ(master.writes().size(), 3u);
    EXPECT_FALSE(master.writes()[0].no_ack);
    EXPECT_TRUE(master.writes()[1].no_ack);
    EXPECT_EQ(master.writes()[1].values, (std::vector<uint16_t>{7, 8}));
    EXPECT_EQ(master.reg(5), 8);
    EXPECT_EQ(master.reg(20), 5);
}

TEST(MockModbusMaster, can_be_made_to_fail)
{
    MockModbusMaster master;
    master.failConnect(true);
    EXPECT_FALSE(master.connect());
    EXPECT_FALSE(master.getLastError().empty());

    master.failConnect(false);
    ASSERT_TRUE(master.connect());
    master.failIo(true);
    uint16_t out = 0;
    EXPECT_EQ(master.readHoldingRegisters(0, 1, &out), -1);
    EXPECT_FALSE(master.writeRegistersNoAck(0, 1, &out));
    EXPECT_TRUE(master.writes().empty());
}

TEST(MockModbusMaster, hands_notices_over_once)
{
    MockModbusMaster master;
    master.addNotice({robot_comm::Notice::Level::Warn, "latency not set"});
    EXPECT_EQ(master.takeNotices().size(), 1u);
    EXPECT_TRUE(master.takeNotices().empty());
}
