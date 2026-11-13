#include "juxie_hardware/juxie_motor_sim.h"

#include <algorithm>
#include <cmath>

namespace juxie_hardware
{
    MotorSim::MotorSim(const Options& options) : options_(options)
    {
        for (const uint8_t id : options_.ids)
        {
            Motor motor;
            motor.id = id;
            motor.enabled = options_.enabled_at_start;
            motors_.push_back(motor);
        }
    }

    MotorSim::Motor* MotorSim::find(uint8_t id)
    {
        for (auto& motor : motors_)
        {
            if (motor.id == id)
            {
                return &motor;
            }
        }
        return nullptr;
    }

    const MotorSim::Motor* MotorSim::find(uint8_t id) const
    {
        for (const auto& motor : motors_)
        {
            if (motor.id == id)
            {
                return &motor;
            }
        }
        return nullptr;
    }

    robot_comm::CanFrame MotorSim::feedbackOf(const Motor& motor) const
    {
        const int16_t raw = static_cast<int16_t>(std::lround(motor.position));
        robot_comm::CanFrame frame;
        frame.id = protocol::kFeedbackBase + motor.id;
        frame.fd = true;
        frame.len = options_.feedback_len;
        frame.data[0] = static_cast<uint8_t>(raw >> 8);
        frame.data[1] = static_cast<uint8_t>(raw & 0xFF);
        frame.data[2] = static_cast<uint8_t>(motor.velocity >> 8);
        frame.data[3] = static_cast<uint8_t>(motor.velocity & 0xFF);
        if (options_.feedback_len == 12)
        {
            frame.data[6] = static_cast<uint8_t>(motor.error_word & 0xFF);
            frame.data[7] = static_cast<uint8_t>(motor.error_word >> 8);
            frame.data[10] = 0x03; // CSP
            frame.data[11] = motor.enabled ? 0x80 : 0x00;
        }
        else
        {
            frame.data[6] = static_cast<uint8_t>((motor.enabled ? 0x20 : 0x00) | (motor.error_word ? 0x08 : 0x00));
        }
        ++feedback_frames_;
        return frame;
    }

    void MotorSim::answerAll(std::vector<robot_comm::CanFrame>& out)
    {
        for (const auto& motor : motors_)
        {
            out.push_back(feedbackOf(motor));
        }
    }

    std::vector<robot_comm::CanFrame> MotorSim::onFrame(const robot_comm::CanFrame& frame)
    {
        std::vector<robot_comm::CanFrame> out;
        if (!frame.isStandardData())
        {
            return out;
        }

        if (frame.id == protocol::kSyncFrameId && frame.len == 1)
        {
            ++sync_frames_;
            answerAll(out);
        }
        else if (frame.id == protocol::kMultiControlId && frame.len == protocol::kMultiControlLen)
        {
            ++control_frames_;
            for (std::size_t i = 0; i < protocol::kMaxMotors; ++i)
            {
                Motor* motor = find(frame.data[56 + i]);
                if (motor == nullptr || frame.data[56 + i] == 0)
                {
                    continue;
                }
                const uint8_t control = frame.data[i * 7];
                const bool clear_error = (control & protocol::kCtrlClearError) != 0 && options_.feedback_len == 12;
                if (clear_error)
                {
                    motor->error_word = 0;
                }
                motor->enabled = true;
                const int16_t command = static_cast<int16_t>((frame.data[i * 7 + 1] << 8) | frame.data[i * 7 + 2]);
                const double before = motor->position;
                motor->position += (command - motor->position) * options_.follow;
                motor->velocity = static_cast<int16_t>(std::lround(motor->position - before));
            }
            answerAll(out);
        }
        else if (frame.id >= protocol::kEnableBase && frame.id < protocol::kEnableBase + 256 && frame.len == 8)
        {
            ++enable_frames_;
            if (Motor* motor = find(static_cast<uint8_t>(frame.id - protocol::kEnableBase)))
            {
                motor->enabled = true;
            }
        }
        return out;
    }

    void MotorSim::injectFault(uint8_t motor_id, uint16_t error_word)
    {
        if (Motor* motor = find(motor_id))
        {
            motor->error_word = error_word;
        }
    }

    void MotorSim::setRawPosition(uint8_t motor_id, int16_t raw)
    {
        if (Motor* motor = find(motor_id))
        {
            motor->position = raw;
        }
    }

    int16_t MotorSim::rawPosition(uint8_t motor_id) const
    {
        const Motor* motor = find(motor_id);
        return motor ? static_cast<int16_t>(std::lround(motor->position)) : 0;
    }
} // namespace juxie_hardware
