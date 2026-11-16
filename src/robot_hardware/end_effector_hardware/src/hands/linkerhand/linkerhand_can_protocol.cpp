#include "end_effector_hardware/hands/linkerhand/linkerhand_can_protocol.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace end_effector_hardware::linkerhand
{
    namespace
    {
        // Only L6 is wired into the robot descriptions; O6 / O7 / L10 use the same protocol with their own entry.
        constexpr std::array<ModelConfig, 1> kModels{{
            {"linkerhand_l6", "l6", 6, {0, 1, 2, 3, 4, 5, 6}, 0xC6, 12, 6, {0.99, 1.39, 1.26, 1.26, 1.26, 1.26, 0.0}},
        }};

        robot_comm::CanFrame dataFrame(uint32_t can_id)
        {
            robot_comm::CanFrame frame;
            frame.id = can_id;
            return frame;
        }
    } // namespace

    const ModelConfig* findModel(const std::string& hand_type)
    {
        for (const auto& model : kModels)
        {
            if (hand_type == model.hand_type)
            {
                return &model;
            }
        }
        return nullptr;
    }

    uint32_t canIdForSide(const std::string& side) { return side == "left" ? kLeftHandCanId : kRightHandCanId; }

    const char* fingerName(Finger finger)
    {
        switch (finger)
        {
        case Finger::Thumb: return "thumb";
        case Finger::Index: return "index";
        case Finger::Middle: return "middle";
        case Finger::Ring: return "ring";
        case Finger::Pinky: return "pinky";
        }
        return "unknown";
    }

    bool fingerFromTactileCommand(uint8_t command, Finger& finger)
    {
        const auto it = std::find(kTactileCommands.begin(), kTactileCommands.end(), command);
        if (it == kTactileCommands.end())
        {
            return false;
        }
        finger = static_cast<Finger>(std::distance(kTactileCommands.begin(), it));
        return true;
    }

    robot_comm::CanFrame makePositionRequest(uint32_t can_id)
    {
        auto frame = dataFrame(can_id);
        frame.len = 1;
        frame.data[0] = kPositionCommand;
        return frame;
    }

    robot_comm::CanFrame makeJointFrame(uint32_t can_id, uint8_t command, const RawJoints& values,
                                        std::size_t joint_count)
    {
        auto frame = dataFrame(can_id);
        const std::size_t count = std::min(joint_count, kMaxJointCount);
        frame.len = static_cast<uint8_t>(count + 1);
        frame.data[0] = command;
        std::copy_n(values.begin(), count, frame.data.begin() + 1);
        return frame;
    }

    bool parsePositionFeedback(const robot_comm::CanFrame& frame, uint32_t can_id, std::size_t joint_count,
                               RawJoints& raw)
    {
        if (!frame.isStandardData() || frame.fd || frame.id != can_id || joint_count > kMaxJointCount ||
            frame.len < joint_count + 1 || frame.data[0] != kPositionCommand)
        {
            return false;
        }
        std::copy_n(frame.data.begin() + 1, joint_count, raw.begin());
        return true;
    }

    robot_comm::CanFrame makeTactileRequest(uint32_t can_id, Finger finger, uint8_t layout)
    {
        auto frame = dataFrame(can_id);
        frame.len = kTactileRequestLen;
        frame.data[0] = kTactileCommands[static_cast<std::size_t>(finger)];
        frame.data[1] = layout;
        return frame;
    }

    bool parseTactileFrame(const robot_comm::CanFrame& frame, uint32_t can_id, const ModelConfig& model,
                           TactileChunk& chunk)
    {
        if (!frame.isStandardData() || frame.fd || frame.id != can_id || frame.len < 1)
        {
            return false;
        }
        Finger finger;
        if (!fingerFromTactileCommand(frame.data[0], finger))
        {
            return false;
        }
        if (frame.len != model.tactile_values_per_frame + 2)
        {
            return false;
        }
        const std::size_t row = (frame.data[1] >> 4) & 0x0F;
        const std::size_t column = frame.data[1] & 0x0F;
        if (row >= model.tactile_frame_count || row >= kMaxTactileRows ||
            column + model.tactile_values_per_frame > kMaxTactileColumns)
        {
            return false;
        }
        chunk.finger = finger;
        chunk.row = row;
        chunk.column = column;
        chunk.values = frame.data.data() + 2;
        return true;
    }

    uint8_t radiansToRaw(double radians, double lower, double upper)
    {
        if (upper <= lower)
        {
            return 255;
        }
        const double normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
        return static_cast<uint8_t>(std::clamp<long>(std::lround(255.0 * (1.0 - normalized)), 0, 255));
    }

    double rawToRadians(uint8_t raw, double lower, double upper)
    {
        if (upper <= lower)
        {
            return lower;
        }
        return lower + (1.0 - static_cast<double>(raw) / 255.0) * (upper - lower);
    }

    uint8_t scaleToRaw(double scale) { return static_cast<uint8_t>(std::lround(std::clamp(scale, 0.0, 1.0) * 255.0)); }
} // namespace end_effector_hardware::linkerhand
