#include "fa_teleop/xr_state_json.hpp"

#include <cstdlib>
#include <sstream>

#include <nlohmann/json.hpp>

namespace fa_teleop
{
    namespace
    {
        using nlohmann::json;

        template <typename T>
        T valueOr(const json& obj, const char* key, T fallback)
        {
            const auto it = obj.find(key);
            if (it == obj.end() || it->is_null())
            {
                return fallback;
            }
            try
            {
                return it->get<T>();
            }
            catch (const json::exception&)
            {
                return fallback;
            }
        }

        bool parseController(const json& obj, ControllerSample& out)
        {
            if (!obj.is_object() || !obj.contains("pose") || !obj["pose"].is_string())
            {
                return false;
            }
            const auto pose = parsePoseString(obj["pose"].get<std::string>());
            if (!pose)
            {
                return false;
            }
            out.pose = *pose;
            out.trigger = valueOr<double>(obj, "trigger", 0.0);
            out.grip = valueOr<double>(obj, "grip", 0.0);
            out.axis_x = valueOr<double>(obj, "axisX", 0.0);
            out.axis_y = valueOr<double>(obj, "axisY", 0.0);
            out.axis_click = valueOr<bool>(obj, "axisClick", false);
            out.primary_button = valueOr<bool>(obj, "primaryButton", false);
            out.secondary_button = valueOr<bool>(obj, "secondaryButton", false);
            out.menu_button = valueOr<bool>(obj, "menuButton", false);
            return true;
        }
    } // namespace

    std::optional<Pose7> parsePoseString(const std::string& text)
    {
        Pose7 pose{};
        std::stringstream ss(text);
        std::string token;
        size_t count = 0;
        while (std::getline(ss, token, ','))
        {
            if (count >= pose.size())
            {
                return std::nullopt;
            }
            char* end = nullptr;
            const double v = std::strtod(token.c_str(), &end);
            if (end == token.c_str())
            {
                return std::nullopt;
            }
            pose[count++] = v;
        }
        if (count != pose.size())
        {
            return std::nullopt;
        }
        return pose;
    }

    std::optional<XrFrame> parseDeviceStateJson(const std::string& state_json, std::string* error)
    {
        try
        {
            const json outer = json::parse(state_json);
            if (!outer.contains("value") || !outer["value"].is_string())
            {
                if (error) { *error = "missing string field 'value'"; }
                return std::nullopt;
            }
            const json value = json::parse(outer["value"].get<std::string>());

            XrFrame frame;
            frame.timestamp_ns = valueOr<int64_t>(value, "timeStampNs", 0);
            if (value.contains("Head") && value["Head"].is_object() && value["Head"].contains("pose") &&
                value["Head"]["pose"].is_string())
            {
                if (const auto head = parsePoseString(value["Head"]["pose"].get<std::string>()))
                {
                    frame.head = *head;
                    frame.has_head = true;
                }
            }
            if (value.contains("Controller") && value["Controller"].is_object())
            {
                const json& controller = value["Controller"];
                if (controller.contains("left"))
                {
                    frame.has_left = parseController(controller["left"], frame.left);
                }
                if (controller.contains("right"))
                {
                    frame.has_right = parseController(controller["right"], frame.right);
                }
            }
            return frame;
        }
        catch (const nlohmann::json::exception& e)
        {
            if (error) { *error = e.what(); }
            return std::nullopt;
        }
    }
} // namespace fa_teleop
