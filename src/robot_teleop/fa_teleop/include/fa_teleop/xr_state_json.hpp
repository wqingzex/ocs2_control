#pragma once

#include <optional>
#include <string>

#include "fa_teleop/xr_types.hpp"

namespace fa_teleop
{
    /// Parse PXREADevStateJson::stateJson ({"value": "<json>"} with Head / Controller / timeStampNs).
    /// Missing or malformed devices leave has_* = false; std::nullopt (and *error) if unparseable.
    std::optional<XrFrame> parseDeviceStateJson(const std::string& state_json, std::string* error = nullptr);

    /// Parse "x,y,z,qx,qy,qz,qw"; std::nullopt unless exactly 7 numbers.
    std::optional<Pose7> parsePoseString(const std::string& text);
} // namespace fa_teleop
