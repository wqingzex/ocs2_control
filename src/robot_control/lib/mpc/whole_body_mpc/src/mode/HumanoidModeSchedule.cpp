/******************************************************************************
 * OCS2 Wheel Humanoid - Mode Schedule Implementation
 ******************************************************************************/

#include "whole_body_mpc/mode/HumanoidModeSchedule.h"
#include <rclcpp/logging.hpp>
#include <sstream>

#include <cctype>
#include <cstring>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <stdexcept>
#include <unordered_map>

namespace ocs2::whole_body_mpc {

//=============================================================================
// String Conversion Functions
//=============================================================================

std::string bodyModeToString(BodyConstraintMode mode) {
    switch (mode) {
        case BodyConstraintMode::FREE:
            return "FREE";
        case BodyConstraintMode::RELATIVE:
            return "RELATIVE";
        case BodyConstraintMode::TRACKING:
            return "TRACKING";
        case BodyConstraintMode::LOCKED:
            return "LOCKED";
        case BodyConstraintMode::HEAD_COUPLED:
            return "HEAD_COUPLED";
        case BodyConstraintMode::CUSTOM_LOCKED:
            return "CUSTOM_LOCKED";
        case BodyConstraintMode::HEAD_TRACKING:
            return "HEAD_TRACKING";
        default:
            return "UNKNOWN";
    }
}

std::string headModeToString(HeadMode mode) {
    switch (mode) {
        case HeadMode::DISABLED: return "DISABLED";
        case HeadMode::TRACKING: return "TRACKING";
        case HeadMode::GAZE: return "GAZE";
        case HeadMode::FORWARD: return "FORWARD";
        default: return "UNKNOWN";
    }
}

std::string bimanualModeToString(BimanualMode mode) {
    switch (mode) {
        case BimanualMode::INDEPENDENT:
            return "INDEPENDENT";
        case BimanualMode::COUPLED:
            return "COUPLED";
        default:
            return "UNKNOWN";
    }
}

std::string modeToString(size_t mode) {
    // Default preset names (stringToMode); must match stringToMode map.
    if (mode == Modes::LEFT_ARM_ONLY) return "HM_LEFT_ARM";
    if (mode == Modes::RIGHT_ARM_ONLY) return "HM_RIGHT_ARM";
    if (mode == Modes::FIXED_DUAL_INDEPENDENT_LOCKED) return "HM_DUAL_IDLE";
    if (mode == Modes::BODY_FREE_DUAL) return "HM_DUAL_BODY_FREE";
    if (mode == Modes::FIXED_DUAL_INDEPENDENT) return "HM_DUAL_RELATIVE_FIXED_BASE";
    if (mode == Modes::WHOLE_BODY) return "HM_DUAL_BODY_TRACKING";
    if (mode == Modes::DUAL_INDEPENDENT) return "HM_DUAL_RELATIVE_MOBILE_BASE";
    if (mode == Modes::FIXED_LEFT_ARM) return "HM_LEFT_ARM";
    if (mode == Modes::FIXED_RIGHT_ARM) return "HM_RIGHT_ARM";
    if (mode == Modes::DUAL_CUSTOM_LOCK) return "HM_DUAL_CUSTOM_LOCK";
    if (mode == Modes::FIXED_DUAL_CUSTOM_LOCK) return "HM_DUAL_CUSTOM_LOCK_FIXED_BASE";
    
    // Decode and format
    HumanoidMode m = HumanoidMode::decode(mode);
    return "Mode{body=" + bodyModeToString(m.bodyMode) + 
           ", head=" + headModeToString(m.headMode) +
           ", left=" + (m.leftArmActive ? "on" : "off") +
           ", right=" + (m.rightArmActive ? "on" : "off") +
           ", bimanual=" + bimanualModeToString(m.bimanualMode) +
           ", base=" + (m.baseLocked ? "locked" : "mobile") + "}";
}

size_t stringToMode(const std::string& name) {
    // Nine body/arms/base presets (see README). Other combinations use Modes::* in C++ or partial/topic commands.
    static const std::unordered_map<std::string, size_t> modeMap = {
        {"HM_LEFT_ARM", Modes::LEFT_ARM_ONLY},
        {"HM_RIGHT_ARM", Modes::RIGHT_ARM_ONLY},
        {"HM_DUAL_IDLE", Modes::FIXED_DUAL_INDEPENDENT_LOCKED},
        {"HM_DUAL_BODY_FREE", Modes::BODY_FREE_DUAL},
        {"HM_DUAL_RELATIVE_FIXED_BASE", Modes::FIXED_DUAL_INDEPENDENT},
        {"HM_DUAL_BODY_TRACKING", Modes::WHOLE_BODY},
        {"HM_DUAL_RELATIVE_MOBILE_BASE", Modes::DUAL_INDEPENDENT},
        {"HM_DUAL_CUSTOM_LOCK", Modes::DUAL_CUSTOM_LOCK},
        {"HM_DUAL_CUSTOM_LOCK_FIXED_BASE", Modes::FIXED_DUAL_CUSTOM_LOCK},
    };

    auto it = modeMap.find(name);
    if (it != modeMap.end()) {
        return it->second;
    }

    throw std::runtime_error(
        "[stringToMode] Unknown preset '" + name +
        "' (use HM_LEFT_ARM, HM_RIGHT_ARM, HM_DUAL_IDLE, HM_DUAL_BODY_FREE, HM_DUAL_RELATIVE_FIXED_BASE, "
        "HM_DUAL_BODY_TRACKING, HM_DUAL_RELATIVE_MOBILE_BASE, "
        "HM_DUAL_CUSTOM_LOCK, HM_DUAL_CUSTOM_LOCK_FIXED_BASE)");
}

std::string normalizeHumanoidModeCommandName(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
        s.erase(0, 1);
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
        s.pop_back();
    }
    for (char& c : s) {
        if (c == ' ') {
            c = '_';
        }
    }
    return s;
}

namespace {

bool startsWithIgnoreCase(const std::string& s, const char* prefix) {
    const size_t n = std::strlen(prefix);
    if (s.size() < n) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        if (std::tolower(static_cast<unsigned char>(s[i])) != std::tolower(static_cast<unsigned char>(prefix[i]))) {
            return false;
        }
    }
    return true;
}

} // namespace

size_t applyPartialHumanoidModeString(size_t currentMode, const std::string& name) {
    if (name.empty()) {
        return currentMode;
    }

    HumanoidMode m = HumanoidMode::decode(currentMode);

    if (name == "HEAD_TRACKING") {
        m.headMode = HeadMode::TRACKING;
        return m.encode();
    }
    if (name == "HEAD_DISABLE") {
        m.headMode = HeadMode::DISABLED;
        return m.encode();
    }
    if (name == "HEAD_GAZE") {
        m.headMode = HeadMode::GAZE;
        return m.encode();
    }
    if (name == "HEAD_FORWARD") {
        m.headMode = HeadMode::FORWARD;
        return m.encode();
    }
    if (name == "BASE_UNLOCK") {
        m.baseLocked = false;
        return m.encode();
    }
    if (name == "BASE_LOCK") {
        m.baseLocked = true;
        return m.encode();
    }
    if (name == "BODY_FREE") {
        m.bodyMode = BodyConstraintMode::FREE;
        return m.encode();
    }
    if (name == "BODY_RELATIVE") {
        m.bodyMode = BodyConstraintMode::RELATIVE;
        return m.encode();
    }
    if (name == "BODY_TRACKING") {
        m.bodyMode = BodyConstraintMode::TRACKING;
        return m.encode();
    }
    if (name == "BODY_LOCK") {
        m.bodyMode = BodyConstraintMode::LOCKED;
        return m.encode();
    }
    if (name == "BODY_CUSTOM_LOCK") {
        m.bodyMode = BodyConstraintMode::CUSTOM_LOCKED;
        return m.encode();
    }
    if (name == "BODY_UNLOCK") {
        if (m.bodyMode == BodyConstraintMode::LOCKED || m.bodyMode == BodyConstraintMode::HEAD_COUPLED ||
            m.bodyMode == BodyConstraintMode::CUSTOM_LOCKED) {
            m.bodyMode = BodyConstraintMode::RELATIVE;
        }
        return m.encode();
    }
    if (name == "ARMS_COUPLED") {
        m.bimanualMode = BimanualMode::COUPLED;
        return m.encode();
    }
    if (name == "ARMS_INDEPENDENT") {
        m.bimanualMode = BimanualMode::INDEPENDENT;
        return m.encode();
    }
    if (name == "LEFT_ARM_ENABLE") {
        m.leftArmActive = true;
        return m.encode();
    }
    if (name == "LEFT_ARM_DISABLE") {
        m.leftArmActive = false;
        return m.encode();
    }
    if (name == "RIGHT_ARM_ENABLE") {
        m.rightArmActive = true;
        return m.encode();
    }
    if (name == "RIGHT_ARM_DISABLE") {
        m.rightArmActive = false;
        return m.encode();
    }

    throw std::runtime_error("[applyPartialHumanoidModeString] Unknown partial keyword: " + name);
}

size_t resolveHumanoidModeCommand(size_t currentMode, const std::string& normalizedName) {
    const std::string& s = normalizedName;
    if (s.empty()) {
        return currentMode;
    }

    if (startsWithIgnoreCase(s, "PRESET:")) {
        std::string rest = normalizeHumanoidModeCommandName(s.substr(7));
        if (rest.empty()) {
            throw std::runtime_error("[resolveHumanoidModeCommand] PRESET: requires a mode name");
        }
        auto preset = HumanoidMode::decode(stringToMode(rest));
        preset.headMode = HumanoidMode::decode(currentMode).headMode;
        return preset.encode();
    }

    return applyPartialHumanoidModeString(currentMode, s);
}

std::ostream& operator<<(std::ostream& os, const HumanoidMode& mode) {
    os << "HumanoidMode {\n"
       << "  bodyMode: " << bodyModeToString(mode.bodyMode) << "\n"
       << "  headMode: " << headModeToString(mode.headMode) << "\n"
       << "  leftArmActive: " << (mode.leftArmActive ? "true" : "false") << "\n"
       << "  rightArmActive: " << (mode.rightArmActive ? "true" : "false") << "\n"
       << "  bimanualMode: " << bimanualModeToString(mode.bimanualMode) << "\n"
       << "  baseLocked: " << (mode.baseLocked ? "true" : "false") << "\n"
       << "}";
    return os;
}

//=============================================================================
// Mode Schedule Loading
//=============================================================================

ModeSchedule loadHumanoidModeSchedule(const std::string& filename,
                                       const std::string& topicName,
                                       bool verbose) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    std::vector<scalar_t> eventTimes;
    loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes, verbose);

    const auto modeSequencePath = topicName + ".modeSequence";
    boost::property_tree::ptree modeSequenceTree;
    try {
        modeSequenceTree = pt.get_child(modeSequencePath);
    } catch (const boost::property_tree::ptree_bad_path&) {
        // Default to HM_DUAL_IDLE (fixed base + torso lock + dual independent)
        if (verbose) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("loadHumanoidModeSchedule"), "No mode schedule found, using default HM_DUAL_IDLE");
        }
        return ModeSchedule({}, {Modes::FIXED_DUAL_INDEPENDENT_LOCKED});
    }

    if (modeSequenceTree.empty()) {
        throw std::invalid_argument("[loadHumanoidModeSchedule] " + modeSequencePath + " must not be empty");
    }
    if (eventTimes.size() + 1 != modeSequenceTree.size()) {
        throw std::invalid_argument("[loadHumanoidModeSchedule] " + modeSequencePath + " must contain " +
                                    std::to_string(eventTimes.size() + 1) + " entries, got " +
                                    std::to_string(modeSequenceTree.size()));
    }
    for (size_t i = 1; i < eventTimes.size(); ++i) {
        if (!(eventTimes[i - 1] < eventTimes[i])) {
            throw std::invalid_argument("[loadHumanoidModeSchedule] " + topicName +
                                        ".eventTimes must be strictly increasing");
        }
    }

    std::vector<size_t> modeSequence;
    modeSequence.reserve(modeSequenceTree.size());
    size_t index = 0;
    for (const auto& entry : modeSequenceTree) {
        const auto expectedKey = "[" + std::to_string(index) + "]";
        if (entry.first != expectedKey || !entry.second.data().empty()) {
            throw std::invalid_argument("[loadHumanoidModeSchedule] " + modeSequencePath + "[" +
                                        std::to_string(index) + "] must be a structured entry with bodyMode and headMode");
        }

        std::string bodyMode;
        std::string headMode;
        try {
            bodyMode = entry.second.get<std::string>("bodyMode");
            headMode = entry.second.get<std::string>("headMode");
        } catch (const boost::property_tree::ptree_bad_path&) {
            throw std::invalid_argument("[loadHumanoidModeSchedule] " + modeSequencePath + "[" +
                                        std::to_string(index) + "] requires bodyMode and headMode");
        }

        size_t mode = stringToMode(bodyMode);
        modeSequence.push_back(applyPartialHumanoidModeString(mode, headMode));
        ++index;
    }
    
    if (verbose) {
        std::ostringstream report;
        report << "Loaded mode schedule:\n  Event times: ";
        for (auto t : eventTimes) report << t << " ";
        report << "\n  Mode sequence: ";
        for (const auto mode : modeSequence) report << modeToString(mode) << " ";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("loadHumanoidModeSchedule"), report.str());
    }
    
    return ModeSchedule(eventTimes, modeSequence);
}

} // namespace ocs2::whole_body_mpc
