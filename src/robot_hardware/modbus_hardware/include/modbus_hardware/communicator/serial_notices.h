#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "robot_comm/modbus.h"
#include "robot_comm/serial.h"

namespace modbus_hardware
{
// The communication layer is ROS-free and hands back what it wants logged (the USB latency timer was changed, ...).
inline void log_serial_notices(const rclcpp::Logger& logger, const std::vector<robot_comm::Notice>& notices)
{
  for (const auto& notice : notices)
  {
    if (notice.level == robot_comm::Notice::Level::Warn)
    {
      RCLCPP_WARN(logger, "%s", notice.text.c_str());
    }
    else
    {
      RCLCPP_INFO(logger, "%s", notice.text.c_str());
    }
  }
}
/// Connect a Modbus master and log what it noticed while doing so (USB latency setting and the like).
inline bool connect_master_and_log(robot_comm::ModbusMaster& master)
{
  const bool connected = master.connect();
  log_serial_notices(rclcpp::get_logger("UsbSerialLatency"), master.takeNotices());
  return connected;
}

}  // namespace modbus_hardware
