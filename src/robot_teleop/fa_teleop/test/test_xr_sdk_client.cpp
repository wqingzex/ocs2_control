#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <optional>
#include <gtest/gtest.h>
#include <PXREARobotSDK.h>
#include "fa_teleop/xr_sdk_client.hpp"

namespace
{
    pfPXREAClientCallback callback = nullptr;
    void* callback_context = nullptr;
}

// Exercise the real callback adapter without starting a service or accessing a headset.
extern "C" int PXREAInit(void* context, pfPXREAClientCallback handler, unsigned)
{
    callback_context = context;
    callback = handler;
    return 0;
}
extern "C" int PXREADeinit()
{
    callback = nullptr;
    callback_context = nullptr;
    return 0;
}

TEST(XrSdkClient, carriesDeviceIdentityWithTrackingFrame)
{
    fa_teleop::XrSdkClient client;
    std::optional<fa_teleop::XrFrame> received;
    ASSERT_TRUE(client.start([&](const auto& frame) { received = frame; }, {}, {}, nullptr));
    PXREADevStateJson state{};
    std::strcpy(state.devID, "PICO-001");
    std::strcpy(state.stateJson, R"({"value":"{\"Head\":{\"pose\":\"0,1,0,0,0,0,1\"},\"timeStampNs\":1}"})");
    callback(callback_context, PXREADeviceStateJson, 0, &state);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->device_id, "PICO-001");
    EXPECT_TRUE(received->has_head);
}

TEST(XrSdkClient, readsDeviceIdWithinTheFixedSizeSdkBuffer)
{
    fa_teleop::XrSdkClient client;
    std::optional<fa_teleop::XrFrame> received;
    ASSERT_TRUE(client.start([&](const auto& frame) { received = frame; }, {}, {}, nullptr));
    PXREADevStateJson state{};
    std::fill(std::begin(state.devID), std::end(state.devID), 'A');
    std::strcpy(state.stateJson, R"({"value":"{\"timeStampNs\":1}"})");
    callback(callback_context, PXREADeviceStateJson, 0, &state);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->device_id, std::string(sizeof(state.devID), 'A'));
}
