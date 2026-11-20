#include "marvin_hardware/comm/sdk_channels.h"

#include "MarvinSDK.h"

namespace marvin_hardware
{
namespace
{
    bool sendLeft(uint8_t* data, long size, long channel) { return OnSetChDataA(data, size, channel); }
    long receiveLeft(unsigned char* data, long* channel) { return OnGetChDataA(data, channel); }
    bool sendRight(uint8_t* data, long size, long channel) { return OnSetChDataB(data, size, channel); }
    long receiveRight(unsigned char* data, long* channel) { return OnGetChDataB(data, channel); }
}  // namespace

ArmChannels SdkChannels::left() { return {sendLeft, receiveLeft}; }
ArmChannels SdkChannels::right() { return {sendRight, receiveRight}; }

}  // namespace marvin_hardware
