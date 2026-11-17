#pragma once

#include <string>
#include <utility>

namespace xr_teleop
{
    /// Keep one headset per bridge session; never mix frames from different devices.
    /// Caller holds the frame mutex when used by XrBridgeNode.
    class XrDeviceSelector
    {
    public:
        explicit XrDeviceSelector(std::string expected = {}) : expected_(std::move(expected)) {}

        bool accept(const std::string& id)
        {
            if (id.empty() || (!expected_.empty() && id != expected_)) { return false; }
            if (selected_.empty()) { selected_ = id; }
            return selected_ == id;
        }

    private:
        std::string expected_;
        std::string selected_;
    };
} // namespace xr_teleop
