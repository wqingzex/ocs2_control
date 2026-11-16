// JodellGripper against a mock Modbus slave: the wiring (addresses, no-ack writes, change detection, link check).
#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gripper_hardware_common/utils/JodellCommandBuilder.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/TorqueConverter.h"
#include "gripper_hardware_common/utils/VelocityConverter.h"
#include "end_effector_hardware/grippers/jodell_gripper.h"
#include "robot_comm_modbus/mock_modbus_master.h"

namespace
{
using gripper_hardware_common::JodellCommandBuilder;
using gripper_hardware_common::PositionConverter;
using gripper_hardware_common::TorqueConverter;
using gripper_hardware_common::VelocityConverter;
namespace Jodell = gripper_hardware_common::ModbusConfig::Jodell;

class JodellGripperTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    gripper_ = std::make_unique<end_effector_hardware::JodellGripper>(
      rclcpp::get_logger("test_jodell"), clock_, "gripper_joint");
    ASSERT_TRUE(master_.connect());
    // position byte 0x40 in the high byte of register 1; torque 0x80 / speed 0x20 in register 2
    master_.setRegister(Jodell::STATUS_REG_ADDR + 1, 0x4000);
    master_.setRegister(Jodell::STATUS_REG_ADDR + 2, 0x8020);
  }

  rclcpp::Clock::SharedPtr clock_;
  robot_comm::MockModbusMaster master_{Jodell::SLAVE_ADDRESS};
  std::unique_ptr<end_effector_hardware::JodellGripper> gripper_;
};
}  // namespace

TEST_F(JodellGripperTest, initialization_sends_the_init_value_without_waiting_and_reads_the_status)
{
  ASSERT_TRUE(gripper_->initialize(&master_, {}));

  ASSERT_FALSE(master_.writes().empty());
  const auto& init = master_.writes().front();
  EXPECT_EQ(init.addr, Jodell::INIT_REG_ADDR);
  EXPECT_TRUE(init.no_ack);
  EXPECT_EQ(init.values, std::vector<uint16_t>{Jodell::INIT_VALUE});
  EXPECT_DOUBLE_EQ(*gripper_->getPositionPtr(), PositionConverter::Jodell::jodellToNormalized(0x40));
}

TEST_F(JodellGripperTest, initialization_fails_when_the_gripper_does_not_answer_the_status_read)
{
  master_.failIo(true);
  EXPECT_FALSE(gripper_->initialize(&master_, {}));
}

TEST_F(JodellGripperTest, status_is_decoded_into_position_velocity_and_effort)
{
  ASSERT_TRUE(gripper_->initialize(&master_, {}));
  master_.setRegister(Jodell::STATUS_REG_ADDR + 1, 0xC000);
  master_.setRegister(Jodell::STATUS_REG_ADDR + 2, 0xFF00);

  ASSERT_TRUE(gripper_->readStatus());
  EXPECT_DOUBLE_EQ(*gripper_->getPositionPtr(), PositionConverter::Jodell::jodellToNormalized(0xC0));
  EXPECT_DOUBLE_EQ(*gripper_->getVelocityPtr(), 0.0);
  EXPECT_DOUBLE_EQ(*gripper_->getEffortPtr(), 1.0);

  master_.setRegister(Jodell::STATUS_REG_ADDR + 2, 0x80FF);
  ASSERT_TRUE(gripper_->readStatus());
  EXPECT_DOUBLE_EQ(*gripper_->getVelocityPtr(), 1.0);
  EXPECT_DOUBLE_EQ(*gripper_->getEffortPtr(), 128.0 / 255.0);
}

TEST_F(JodellGripperTest, a_read_failure_is_reported)
{
  ASSERT_TRUE(gripper_->initialize(&master_, {}));
  master_.failIo(true);
  EXPECT_FALSE(gripper_->readStatus());
}

TEST_F(JodellGripperTest, a_changed_command_is_written_once_as_a_no_ack_block)
{
  ASSERT_TRUE(gripper_->initialize(&master_, {{"torque", "0.5"}, {"velocity", "0.25"}}));
  const auto writes_after_init = master_.writes().size();

  *gripper_->getPositionCommandPtr() = 0.8;
  ASSERT_TRUE(gripper_->writeCommand());
  ASSERT_EQ(master_.writes().size(), writes_after_init + 1);

  const auto expected = JodellCommandBuilder::buildCommand(
    PositionConverter::Jodell::normalizedToJodell(0.8),
    TorqueConverter::Jodell::normalizedToJodell(0.5),
    VelocityConverter::Jodell::normalizedToLowByte(0.25));
  const auto& write = master_.writes().back();
  EXPECT_EQ(write.addr, Jodell::POSITION_REG_ADDR);
  EXPECT_TRUE(write.no_ack);
  EXPECT_EQ(write.values, expected);

  // the same command again: nothing goes out
  EXPECT_FALSE(gripper_->writeCommand());
  EXPECT_EQ(master_.writes().size(), writes_after_init + 1);
}

TEST_F(JodellGripperTest, a_failed_send_is_reported_and_retried_on_the_next_cycle)
{
  ASSERT_TRUE(gripper_->initialize(&master_, {}));
  *gripper_->getPositionCommandPtr() = 0.2;

  master_.failIo(true);
  EXPECT_FALSE(gripper_->writeCommand());
  master_.failIo(false);
  EXPECT_TRUE(gripper_->writeCommand());
}

TEST_F(JodellGripperTest, nothing_works_before_initialize_or_after_shutdown)
{
  EXPECT_FALSE(gripper_->readStatus());
  EXPECT_FALSE(gripper_->writeCommand());

  ASSERT_TRUE(gripper_->initialize(&master_, {}));
  gripper_->shutdown();
  EXPECT_FALSE(gripper_->readStatus());
}

TEST_F(JodellGripperTest, the_measured_speed_does_not_replace_the_configured_command_speed)
{
  // A status read reports speed 0x20, but commands keep using the configured 0.25 (this used to be overwritten).
  ASSERT_TRUE(gripper_->initialize(&master_, {{"velocity", "0.25"}}));
  ASSERT_TRUE(gripper_->readStatus());
  EXPECT_DOUBLE_EQ(*gripper_->getVelocityPtr(), 0x20 / 255.0);

  *gripper_->getPositionCommandPtr() = 0.6;
  ASSERT_TRUE(gripper_->writeCommand());
  const uint16_t speed_and_force = master_.writes().back().values.at(2);
  EXPECT_EQ(speed_and_force & 0xFF, VelocityConverter::Jodell::normalizedToLowByte(0.25));
}
