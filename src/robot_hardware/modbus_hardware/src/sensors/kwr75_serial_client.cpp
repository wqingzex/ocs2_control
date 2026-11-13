#include "modbus_hardware/sensors/kwr75_serial_client.h"
#include "modbus_hardware/communicator/serial_notices.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>


namespace modbus_hardware
{
namespace
{
constexpr uint8_t kFrameMarker = 0xAA;
constexpr uint8_t kFrameEnd0 = 0x0D;
constexpr uint8_t kFrameEnd1 = 0x0A;
constexpr std::size_t kPayloadOffset = 2;
constexpr std::size_t kFloatBytes = 4;

bool frame_matches(
  const std::vector<uint8_t>& buffer,
  std::size_t start,
  uint8_t command_code)
{
  if (start + Kwr75SerialClient::kFrameLength > buffer.size())
  {
    return false;
  }
  const uint8_t header = buffer[start];
  if (header != command_code && header != 0x48 && header != 0x49)
  {
    return false;
  }
  if (buffer[start + 1] != kFrameMarker)
  {
    return false;
  }
  if (buffer[start + Kwr75SerialClient::kFrameLength - 2] != kFrameEnd0 ||
      buffer[start + Kwr75SerialClient::kFrameLength - 1] != kFrameEnd1)
  {
    return false;
  }
  return true;
}

}  // namespace

Kwr75SerialClient::Kwr75SerialClient(
  std::string serial_port,
  int baudrate,
  uint8_t command_code,
  bool convert_to_si,
  double gravity,
  int response_timeout_ms,
  int read_timeout_ms,
  int startup_delay_ms,
  int warmup_attempts)
: serial_port_(std::move(serial_port))
, baudrate_(baudrate)
, command_code_(command_code)
, convert_to_si_(convert_to_si)
, gravity_(gravity)
, response_timeout_ms_(response_timeout_ms)
, read_timeout_ms_(read_timeout_ms)
, startup_delay_ms_(startup_delay_ms)
, warmup_attempts_(warmup_attempts)
{
}

bool Kwr75SerialClient::connect()
{
  disconnect();

  robot_comm::SerialConfig config;
  config.device = serial_port_;
  config.baudrate = baudrate_;
  config.strict_baudrate = true;  // an unsupported baudrate has always been a connect failure here
  const bool opened = port_->open(config);
  log_serial_notices(rclcpp::get_logger("Kwr75SerialClient"), port_->takeNotices());
  if (!opened)
  {
    return false;
  }

  streaming_started_ = false;
  return true;
}

void Kwr75SerialClient::disconnect()
{
  port_->close();
  streaming_started_ = false;
}

bool Kwr75SerialClient::start_capture()
{
  if (!is_connected())
  {
    return false;
  }
  if (streaming_started_)
  {
    return true;
  }

  port_->flushAll();
  if (!send_start_command())
  {
    return false;
  }

  streaming_started_ = true;
  return true;
}

void Kwr75SerialClient::record_io_sample(const std::vector<uint8_t>& buffer)
{
  if (buffer.empty())
  {
    last_io_sample_hex_.clear();
    return;
  }
  const std::size_t sample_len = std::min(buffer.size(), std::size_t{32});
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < sample_len; ++i)
  {
    if (i > 0)
    {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<int>(buffer[i]);
  }
  if (buffer.size() > sample_len)
  {
    oss << " ...";
  }
  last_io_sample_hex_ = oss.str();
}

bool Kwr75SerialClient::warmup()
{
  if (!is_connected())
  {
    return false;
  }
  if (!start_capture())
  {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));

  std::array<double, kAxisCount> wrench {};
  for (int attempt = 0; attempt < warmup_attempts_; ++attempt)
  {
    if (read_wrench(wrench, response_timeout_ms_))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool Kwr75SerialClient::read_wrench(std::array<double, kAxisCount>& wrench_si, int timeout_ms)
{
  if (!is_connected())
  {
    return false;
  }
  if (!streaming_started_ && !start_capture())
  {
    return false;
  }

  const int effective_timeout = timeout_ms >= 0 ? timeout_ms : read_timeout_ms_;

  std::array<uint8_t, kFrameLength> frame {};
  if (!read_latest_frame(frame, effective_timeout))
  {
    return false;
  }

  std::array<float, kAxisCount> raw_values {};
  if (!parse_frame(frame, raw_values, command_code_))
  {
    return false;
  }

  const double force_scale = convert_to_si_ ? gravity_ : 1.0;
  const double torque_scale = convert_to_si_ ? gravity_ : 1.0;
  wrench_si[0] = static_cast<double>(raw_values[0]) * force_scale;
  wrench_si[1] = static_cast<double>(raw_values[1]) * force_scale;
  wrench_si[2] = static_cast<double>(raw_values[2]) * force_scale;
  wrench_si[3] = static_cast<double>(raw_values[3]) * torque_scale;
  wrench_si[4] = static_cast<double>(raw_values[4]) * torque_scale;
  wrench_si[5] = static_cast<double>(raw_values[5]) * torque_scale;
  return true;
}

bool Kwr75SerialClient::send_start_command()
{
  const uint8_t request[4] = {command_code_, kFrameMarker, kFrameEnd0, kFrameEnd1};
  std::string error;
  return robot_comm::writeAll(*port_, request, sizeof(request), std::chrono::milliseconds(20), error);
}

bool Kwr75SerialClient::read_latest_frame(
  std::array<uint8_t, kFrameLength>& frame,
  int timeout_ms)
{
  std::vector<uint8_t> buffer;
  buffer.reserve(kFrameLength * 8);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  bool found = false;
  while (std::chrono::steady_clock::now() <= deadline)
  {
    const auto ready = port_->waitReadable(std::chrono::microseconds(1000));
    if (ready == robot_comm::IoStatus::Error)
    {
      return false;
    }
    if (ready == robot_comm::IoStatus::WouldBlock)
    {
      if (found)
      {
        return true;
      }
      continue;
    }

    uint8_t chunk[256];
    std::size_t bytes_read = 0;
    const auto read_status = port_->read(chunk, sizeof(chunk), bytes_read);
    if (read_status == robot_comm::IoStatus::Error)
    {
      return false;
    }
    if (read_status != robot_comm::IoStatus::Ok || bytes_read == 0)
    {
      continue;
    }

    buffer.insert(buffer.end(), chunk, chunk + bytes_read);

    for (std::size_t start = 0; start + kFrameLength <= buffer.size(); ++start)
    {
      if (!frame_matches(buffer, start, command_code_))
      {
        continue;
      }
      std::copy_n(
        buffer.begin() + static_cast<std::ptrdiff_t>(start),
        kFrameLength,
        frame.begin());
      found = true;
    }

    if (buffer.size() > kFrameLength * 8)
    {
      buffer.erase(buffer.begin(), buffer.end() - static_cast<std::ptrdiff_t>(kFrameLength * 2));
    }
  }

  if (!found)
  {
    record_io_sample(buffer);
  }

  return found;
}

float Kwr75SerialClient::decode_wire_float(const uint8_t* wire_bytes)
{
  float value = 0.0F;
  std::memcpy(&value, wire_bytes, sizeof(value));
  return value;
}

bool Kwr75SerialClient::parse_frame(
  const std::array<uint8_t, kFrameLength>& frame,
  std::array<float, kAxisCount>& values,
  uint8_t command_code)
{
  const uint8_t header = frame[0];
  if (header != command_code && header != 0x48 && header != 0x49)
  {
    return false;
  }
  if (frame[1] != kFrameMarker)
  {
    return false;
  }
  if (frame[kFrameLength - 2] != kFrameEnd0 || frame[kFrameLength - 1] != kFrameEnd1)
  {
    return false;
  }

  for (std::size_t axis = 0; axis < kAxisCount; ++axis)
  {
    const std::size_t offset = kPayloadOffset + axis * kFloatBytes;
    values[axis] = decode_wire_float(frame.data() + offset);
  }
  return true;
}

}  // namespace modbus_hardware
