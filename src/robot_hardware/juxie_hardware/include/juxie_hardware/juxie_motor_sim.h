#pragma once

#include <cstdint>
#include <vector>

#include "juxie_hardware/juxie_protocol.h"
#include "robot_comm/can.h"

namespace juxie_hardware
{
    /**
     * A bench model of Juxie joint modules, for the closed-loop unit tests: feed it the frames the driver sends,
     * it returns the frames the motors would answer with. Pure logic, no socket, no clock.
     *
     *  - sync frame (0x80, 1 byte): every motor answers with its state
     *  - multi-motor control frame (0x200, 64 bytes): every addressed motor moves towards the commanded position
     *    (`follow` = fraction of the remaining distance covered per frame; 1.0 = it is there at once) and every
     *    motor answers with its state
     *  - enable frames (0x600 + id): remembered, no answer (the driver does not wait for one)
     *
     * A fault injected with injectFault() shows up in the feedback until a control frame with the "clear error"
     * bit arrives, like a real module.
     */
    class MotorSim
    {
    public:
        struct Options
        {
            std::vector<uint8_t> ids = {1, 2};
            uint8_t feedback_len = 12; ///< 12 (current protocol) or 7 (older firmware)
            double follow = 1.0;
            bool enabled_at_start = true;
        };

        explicit MotorSim(const Options& options);

        /// Frames the motors send in answer to `frame` (empty if they stay silent).
        std::vector<robot_comm::CanFrame> onFrame(const robot_comm::CanFrame& frame);

        void injectFault(uint8_t motor_id, uint16_t error_word);
        void setRawPosition(uint8_t motor_id, int16_t raw);

        int16_t rawPosition(uint8_t motor_id) const;
        uint64_t controlFrames() const { return control_frames_; }
        uint64_t syncFrames() const { return sync_frames_; }
        uint64_t enableFrames() const { return enable_frames_; }
        uint64_t feedbackFrames() const { return feedback_frames_; }

    private:
        struct Motor
        {
            uint8_t id;
            double position = 0.0; ///< raw counts
            int16_t last_raw = 0;
            int16_t velocity = 0;
            uint16_t error_word = 0;
            bool enabled = true;
        };

        Motor* find(uint8_t id);
        const Motor* find(uint8_t id) const;
        robot_comm::CanFrame feedbackOf(const Motor& motor) const;
        void answerAll(std::vector<robot_comm::CanFrame>& out);

        Options options_;
        std::vector<Motor> motors_;
        uint64_t control_frames_ = 0;
        uint64_t sync_frames_ = 0;
        uint64_t enable_frames_ = 0;
        mutable uint64_t feedback_frames_ = 0;
    };
} // namespace juxie_hardware
