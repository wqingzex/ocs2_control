#include "xr_teleop/xr_sdk_client.hpp"

#include <cstdint>
#include <algorithm>
#include <iterator>

#include <PXREARobotSDK.h>

#include "xr_teleop/xr_state_json.hpp"

namespace xr_teleop
{
    namespace
    {
        /// PXREARobotSDK keeps global state: only one client per process.
        std::atomic<bool> g_sdk_in_use{false};

        constexpr unsigned kCallbackMask = PXREAServerConnect | PXREAServerDisconnect | PXREADeviceFind |
                                           PXREADeviceMissing | PXREADeviceConnect | PXREADeviceStateJson;
    } // namespace

    XrSdkClient::~XrSdkClient() { stop(); }

    bool XrSdkClient::start(FrameCallback on_frame, EventCallback on_event, ErrorCallback on_parse_error,
                            std::string* error)
    {
        if (running_)
        {
            return true;
        }
        bool expected = false;
        if (!g_sdk_in_use.compare_exchange_strong(expected, true))
        {
            if (error) { *error = "another XrSdkClient is already running in this process"; }
            return false;
        }
        on_frame_ = std::move(on_frame);
        on_event_ = std::move(on_event);
        on_parse_error_ = std::move(on_parse_error);

        // Captureless lambda -> C function pointer; defined inside a member so it may call dispatch().
        const auto callback = [](void* context, PXREAClientCallbackType type, int status, void* user_data)
        {
            XrSdkClient::dispatch(context, static_cast<int>(type), status, user_data);
        };
        running_ = true;
        const int rc = PXREAInit(this, callback, kCallbackMask);
        if (rc != 0)
        {
            running_ = false;
            g_sdk_in_use = false;
            if (error) { *error = "PXREAInit failed with code " + std::to_string(rc); }
            return false;
        }
        return true;
    }

    void XrSdkClient::stop()
    {
        if (!running_)
        {
            return;
        }
        PXREADeinit();
        running_ = false;
        g_sdk_in_use = false;
    }

    void XrSdkClient::dispatch(void* context, int type, int /*status*/, void* user_data)
    {
        auto* self = static_cast<XrSdkClient*>(context);
        if (self == nullptr || !self->running_)
        {
            return;
        }
        const auto detail = [user_data]() -> std::string
        {
            return user_data ? std::string(static_cast<const char*>(user_data)) : std::string();
        };
        switch (type)
        {
        case PXREAServerConnect:
            if (self->on_event_) { self->on_event_(Event::kServerConnected, {}); }
            break;
        case PXREAServerDisconnect:
            if (self->on_event_) { self->on_event_(Event::kServerDisconnected, {}); }
            break;
        case PXREADeviceFind:
            if (self->on_event_) { self->on_event_(Event::kDeviceFound, detail()); }
            break;
        case PXREADeviceMissing:
            if (self->on_event_) { self->on_event_(Event::kDeviceMissing, detail()); }
            break;
        case PXREADeviceConnect:
            if (self->on_event_) { self->on_event_(Event::kDeviceConnected, detail()); }
            break;
        case PXREADeviceStateJson:
        {
            if (user_data == nullptr) { break; }
            const auto& state = *static_cast<const PXREADevStateJson*>(user_data);
            std::string error;
            if (auto frame = parseDeviceStateJson(state.stateJson, &error))
            {
                frame->device_id.assign(std::begin(state.devID),
                                        std::find(std::begin(state.devID), std::end(state.devID), '\0'));
                if (self->on_frame_) { self->on_frame_(*frame); }
            }
            else if (self->on_parse_error_)
            {
                self->on_parse_error_(error);
            }
            break;
        }
        default:
            break;
        }
    }
} // namespace xr_teleop
