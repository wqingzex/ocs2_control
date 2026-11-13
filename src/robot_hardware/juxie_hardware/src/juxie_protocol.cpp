#include "juxie_hardware/juxie_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace juxie_hardware::protocol
{
    int16_t angleToRaw(double angle_deg)
    {
        const double clamped_angle = std::clamp(angle_deg, kAngleMin, kAngleMax);
        const double ratio = (clamped_angle - kAngleMin) / (kAngleMax - kAngleMin);
        const int32_t raw_value = static_cast<int32_t>(kRawMin + ratio * (kRawMax - kRawMin));
        const int32_t clamped_raw = std::clamp(raw_value, static_cast<int32_t>(kRawMin), static_cast<int32_t>(kRawMax));
        return static_cast<int16_t>(clamped_raw);
    }

    double rawToAngle(int16_t raw)
    {
        const double ratio = (raw - kRawMin) / static_cast<double>(kRawMax - kRawMin);
        const double angle = kAngleMin + ratio * (kAngleMax - kAngleMin);
        return std::round(angle * 1000) / 1000;
    }

    robot_comm::CanFrame makeSyncFrame()
    {
        robot_comm::CanFrame frame;
        frame.id = kSyncFrameId;
        frame.fd = true;
        frame.len = 1;
        frame.data[0] = 0x00;
        return frame;
    }

    robot_comm::CanFrame makeEnableFrame(uint8_t motor_id)
    {
        static constexpr uint8_t enable_cmd[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
        robot_comm::CanFrame frame;
        frame.id = kEnableBase + motor_id;
        frame.fd = true;
        frame.len = 8;
        std::memcpy(frame.data.data(), enable_cmd, sizeof(enable_cmd));
        return frame;
    }

    uint8_t controlByte(uint8_t feedback_len, bool first_command)
    {
        if (feedback_len == 7)
        {
            return first_command ? 0xD8 : 0xD0;
        }
        return first_command ? kCtrlWordCspClear : kCtrlWordCsp;
    }

    robot_comm::CanFrame makeMultiMotorFrame(uint8_t control_byte, const std::vector<int16_t>& raw,
                                             const std::vector<uint8_t>& ids)
    {
        robot_comm::CanFrame frame;
        frame.id = kMultiControlId;
        frame.fd = true;
        frame.len = static_cast<uint8_t>(kMultiControlLen);
        const std::size_t count = std::min({raw.size(), ids.size(), kMaxMotors});
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t offset = i * 7;
            frame.data[offset + 0] = control_byte;
            frame.data[offset + 1] = static_cast<uint8_t>((raw[i] >> 8) & 0xFF); // position, high byte
            frame.data[offset + 2] = static_cast<uint8_t>(raw[i] & 0xFF);        // position, low byte
            // bytes 3..6: target parameter 2 and feed-forward, not used in CSP mode
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            frame.data[56 + i] = ids[i];
        }
        return frame;
    }

    StepResult limitStep(int32_t target, int32_t last, int32_t last_delta, const StepLimits& limits)
    {
        const int32_t remaining = target - last;

        int32_t desired_delta = remaining;
        if (desired_delta > limits.max_step)
        {
            desired_delta = limits.max_step;
        }
        else if (desired_delta < -limits.max_step)
        {
            desired_delta = -limits.max_step;
        }

        int32_t delta = desired_delta;
        if (limits.max_accel > 0)
        {
            const int32_t min_delta = last_delta - limits.max_accel;
            const int32_t max_delta = last_delta + limits.max_accel;
            delta = std::clamp(desired_delta, min_delta, max_delta);
            delta = std::clamp(delta, -limits.max_step, limits.max_step);
        }

        // Do not step past the target; if that makes the increment jump, not overshooting wins.
        if ((remaining >= 0 && delta > remaining) || (remaining <= 0 && delta < remaining))
        {
            delta = remaining;
        }

        const int32_t sent = std::clamp(last + delta, static_cast<int32_t>(kRawMin), static_cast<int32_t>(kRawMax));
        return StepResult{static_cast<int16_t>(sent), sent - last};
    }

    Classified classify(const robot_comm::CanFrame& frame, const std::vector<uint8_t>& motor_ids)
    {
        if (!frame.isStandardData())
        {
            return {};
        }
        const auto is_ours = [&](uint8_t id) { return std::find(motor_ids.begin(), motor_ids.end(), id) != motor_ids.end(); };
        if (frame.id >= kFeedbackBase && frame.id < kFeedbackBase + 256)
        {
            const auto id = static_cast<uint8_t>(frame.id - kFeedbackBase);
            if (is_ours(id))
            {
                return {FrameKind::Feedback, id};
            }
        }
        else if (frame.id >= kEmergencyBase && frame.id < kEmergencyBase + 256)
        {
            const auto id = static_cast<uint8_t>(frame.id - kEmergencyBase);
            if (is_ours(id))
            {
                return {FrameKind::Emergency, id};
            }
        }
        return {};
    }

    DecodeResult decodeFeedback(const robot_comm::CanFrame& frame, uint8_t protocol_len, MotorFeedback& out)
    {
        if (protocol_len != 12 && protocol_len != 7)
        {
            return DecodeResult::UnsupportedLen;
        }
        if (frame.len < protocol_len)
        {
            return DecodeResult::TooShort;
        }
        const auto word = [&](int hi) { return static_cast<int16_t>((frame.data[hi] << 8) | frame.data[hi + 1]); };

        out = MotorFeedback{};
        out.position_rad = rawToAngle(word(0)) * M_PI / 180.0;
        out.velocity_rad_s = word(2) * 2 * M_PI / 60.0;
        if (protocol_len == 12)
        {
            out.effort = static_cast<double>(word(4));
            out.error_code = static_cast<uint16_t>(frame.data[6] | (static_cast<uint16_t>(frame.data[7]) << 8));
            out.mode = frame.data[10];
            out.status = frame.data[11];
            out.enabled = (out.status & 0x80) != 0;
            out.fault = (out.status & 0x20) != 0 || out.error_code != 0;
        }
        else
        {
            out.effort = 0.0;
            out.status = frame.data[6];
            out.enabled = (out.status & 0x20) != 0;
            out.fault = (out.status & 0x08) != 0;
        }
        return DecodeResult::Ok;
    }

    std::vector<FaultInfo> decodeErrorWord(uint16_t error_code_raw)
    {
        struct Entry
        {
            uint16_t bit;
            FaultInfo info;
        };
        static const Entry table[] = {
            {0x0001, {0x0001, "过压"}},
            {0x0002, {0x0002, "欠压"}},
            {0x0004, {0x0003, "过温"}},
            {0x0008, {0x0004, "堵转"}},
            {0x0010, {0x0005, "过载"}},
            {0x0020, {0x0006, "速度跟踪误差过大"}},
            {0x0040, {0x0007, "正限位保护"}},
            {0x0080, {0x0008, "负限位保护"}},
            {0x0100, {0x0009, "输出端编码器异常"}},
            {0x0200, {0x000A, "位置跃迁过大"}},
            {0x0400, {0x000B, "电流采样错误"}},
            {0x0800, {0x000C, "电流跟踪误差超差"}},
            {0x1000, {0x000D, "位置跟踪误差过大"}},
        };
        std::vector<FaultInfo> faults;
        for (const auto& entry : table)
        {
            if (error_code_raw & entry.bit)
            {
                faults.push_back(entry.info);
            }
        }
        return faults;
    }

    std::vector<FaultInfo> decodeEmergencyBytes(uint8_t byte0, uint8_t byte1)
    {
        struct Entry
        {
            int byte;
            uint8_t bit;
            FaultInfo info;
        };
        static const Entry table[] = {
            {0, 0x80, {0x0001, "负限位保护"}},      {0, 0x40, {0x0002, "正限位保护"}},
            {0, 0x20, {0x0003, "超过最大速度"}},    {0, 0x10, {0x0004, "过载"}},
            {0, 0x08, {0x0005, "堵转"}},            {0, 0x04, {0x0006, "过温"}},
            {0, 0x02, {0x0007, "欠压"}},            {0, 0x01, {0x0008, "过压"}},
            {1, 0x08, {0x0009, "CAN通信故障"}},     {1, 0x04, {0x000A, "相电流采样故障"}},
            {1, 0x02, {0x000B, "位置跃迁过大故障"}}, {1, 0x01, {0x000C, "绝对值编码器故障"}},
        };
        std::vector<FaultInfo> faults;
        for (const auto& entry : table)
        {
            const uint8_t byte = entry.byte == 0 ? byte0 : byte1;
            if (byte & entry.bit)
            {
                faults.push_back(entry.info);
            }
        }
        return faults;
    }
} // namespace juxie_hardware::protocol
