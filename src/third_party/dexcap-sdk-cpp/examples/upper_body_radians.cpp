#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include "cpp/DexCap.hpp"

namespace
{

volatile std::sig_atomic_t keepRunning = 1;

void StopOnSignal(int)
{
    keepRunning = 0;
}

double ToRadians(uint16_t rawAngle)
{
    // The SDK reports joint angles in hundredths of a degree.
    constexpr double scale = std::numbers::pi_v<double> / 18000.0;
    return static_cast<double>(rawAngle) * scale;
}

template <std::size_t N>
void PrintJointGroup(const char *groupName,
    const char *jointPrefix,
    const std::array<uint16_t, N> &rawAngles)
{
    std::cout << ' ' << groupName << "=[";
    for (std::size_t i = 0; i < rawAngles.size(); ++i)
    {
        if (i != 0)
            std::cout << ", ";

        std::cout << jointPrefix << (i + 1) << '=' << ToRadians(rawAngles[i]);
    }
    std::cout << ']';
}

} // namespace

int main(int argc, char **argv)
{
    const std::string serialPort = argc > 1 ? argv[1] : "/dev/ttyACM0";

    std::signal(SIGINT, StopOnSignal);
    std::signal(SIGTERM, StopOnSignal);

    DexRobot::DexCapSuit suit(ProductVersion::V4);

    std::cout << "Connecting upper-body device on " << serialPort << "..." << std::endl;
    const DexRobot::ExoApparatus device =
        suit.ConnectDevice(serialPort, DexRobot::AdapterType::WIREDUSB);

    if (device != UpBody)
    {
        if (device != UnDefn)
            suit.DisconnectDevice(device);

        std::cerr << "The device on " << serialPort
                  << " is not a DexCap upper-body device. SDK message: "
                  << suit.getErrorMessage() << std::endl;
        return 1;
    }

    // Start only the waist/dual-arm device. No glove is connected or started.
    if (!suit.Start(UpBody))
    {
        std::cerr << "Failed to start upper-body sampling. SDK message: "
                  << suit.getErrorMessage(UpBody) << std::endl;
        suit.Close(UpBody);
        return 1;
    }

    std::cout << "Sampling started. Angles are in radians; press Ctrl-C to stop."
              << std::endl;
    std::cout << std::fixed << std::setprecision(6);

    uint64_t lastTimestamp = 0;
    while (keepRunning != 0)
    {
        // The SDK updates its internal object asynchronously, so copy it immediately.
        const SkeletonJointAngles state = suit.GetBodyJointState();
        if (state.timestamp == 0 || state.timestamp == lastTimestamp)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        lastTimestamp = state.timestamp;

        // Reorder the SDK layout (left, right, waist) to match the V4 URDF order
        // (waist, left, right).
        const std::array<uint16_t, 5> waist = {
            state.Back1, state.Back2, state.Back3, state.Back4, state.Back5};
        const std::array<uint16_t, 9> leftArm = {
            state.LArm1, state.LArm2, state.LArm3, state.LArm4, state.LArm5,
            state.LArm6, state.LArm7, state.LArm8, state.LArm9};
        const std::array<uint16_t, 9> rightArm = {
            state.RArm1, state.RArm2, state.RArm3, state.RArm4, state.RArm5,
            state.RArm6, state.RArm7, state.RArm8, state.RArm9};

        std::cout << "timestamp=" << state.timestamp;
        PrintJointGroup("waist", "waist_joint_", waist);
        PrintJointGroup("left_arm", "left_arm_joint_", leftArm);
        PrintJointGroup("right_arm", "right_arm_joint_", rightArm);
        std::cout << '\n' << std::flush;
    }

    const DEX_RETURN closeResult = suit.Close(UpBody);
    if (closeResult != DEX_SUCCESS)
    {
        std::cerr << "Upper-body device closed with SDK return code "
                  << static_cast<int>(closeResult) << std::endl;
        return 1;
    }

    return 0;
}
