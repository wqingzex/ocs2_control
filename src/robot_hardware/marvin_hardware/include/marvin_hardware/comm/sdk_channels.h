#pragma once

#include <cstdint>

namespace marvin_hardware
{
    /** Channel ids of OnSetChData / OnGetChData: 1 = CANFD, 2 = COM1, 3 = COM2. OnGetChData: set *channel to the
     *  channel before the call; the SDK returns the data of that channel. */
    constexpr long CAN_CHANNEL = 1;
    constexpr long COM1_CHANNEL = 2;
    constexpr long COM2_CHANNEL = 3;

    using ChannelSendFunc = bool (*)(uint8_t* data, long size, long channel);
    using ChannelReceiveFunc = long (*)(unsigned char* data, long* channel);

    /** The channel functions of one arm (the SDK has a pair per arm: A = left, B = right). */
    struct ArmChannels
    {
        ChannelSendFunc send;
        ChannelReceiveFunc receive;
    };

    /**
     * Thin wrappers around the Marvin SDK OnSetChDataA/B and OnGetChDataA/B. No locks: the SDK has one send / receive
     * buffer per arm, and OnGetChData is not re-entrant, so only one thread per arm may call send / receive.
     */
    class SdkChannels
    {
    public:
        static ArmChannels left();
        static ArmChannels right();
    };
}  // namespace marvin_hardware
