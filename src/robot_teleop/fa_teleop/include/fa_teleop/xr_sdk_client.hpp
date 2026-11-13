#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "fa_teleop/xr_types.hpp"

namespace fa_teleop
{
    /// Wrapper over the PC Service SDK (PXREARobotSDK, process-wide singleton: one client per process).
    /// Callbacks run on the SDK thread; keep them short and never call start()/stop() from them.
    class XrSdkClient
    {
    public:
        enum class Event
        {
            kServerConnected,
            kServerDisconnected,
            kDeviceFound,
            kDeviceMissing,
            kDeviceConnected,
        };

        using FrameCallback = std::function<void(const XrFrame&)>;
        using EventCallback = std::function<void(Event, const std::string& detail)>;
        using ErrorCallback = std::function<void(const std::string&)>;

        XrSdkClient() = default;
        ~XrSdkClient();
        XrSdkClient(const XrSdkClient&) = delete;
        XrSdkClient& operator=(const XrSdkClient&) = delete;

        /// Connect to the PC Service. Returns false (and sets *error) if PXREAInit fails.
        bool start(FrameCallback on_frame, EventCallback on_event, ErrorCallback on_parse_error,
                   std::string* error);
        void stop();
        bool running() const { return running_; }

    private:
        static void dispatch(void* context, int type, int status, void* user_data);

        FrameCallback on_frame_;
        EventCallback on_event_;
        ErrorCallback on_parse_error_;
        std::atomic<bool> running_{false};
    };
} // namespace fa_teleop
