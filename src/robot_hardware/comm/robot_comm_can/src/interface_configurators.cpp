#include "robot_comm_can/interface_configurators.h"

#include <chrono>
#include <cstdlib>
#include <sys/wait.h>
#include <thread>

namespace robot_comm
{
    std::string SudoIpConfigurator::buildCommand(const CanConfig& config)
    {
        // For the Juxie defaults this is, character for character, the command the driver has always run and
        // the one the sudoers rule from scripts/setup.sh allows:
        //   sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000
        //   restart-ms 100 fd on
        std::string command = "sudo ip link set " + config.interface + " down && " + "sudo ip link set " +
            config.interface + " up type can bitrate " + std::to_string(config.bitrate);
        if (config.fd())
        {
            command += " dbitrate " + std::to_string(config.data_bitrate);
        }
        if (config.restart_ms > 0)
        {
            command += " restart-ms " + std::to_string(config.restart_ms);
        }
        if (config.fd())
        {
            command += " fd on";
        }
        return command;
    }

    bool SudoIpConfigurator::apply(const CanConfig& config)
    {
        if (!isValidInterfaceName(config.interface))
        {
            error_ = "invalid CAN interface name '" + config.interface + "'";
            return false;
        }
        const int status = std::system(buildCommand(config).c_str());
        if (status == -1 || WEXITSTATUS(status) != 0)
        {
            error_ = "failed to configure " + config.interface + " (is the sudoers rule from scripts/setup.sh host in place?)";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // let the kernel finish the device setup
        return true;
    }

    std::unique_ptr<CanInterfaceConfigurator> makeConfigurator(const CanConfig& config)
    {
        if (config.manage_interface)
        {
            return std::make_unique<SudoIpConfigurator>();
        }
        return std::make_unique<NoopConfigurator>();
    }
} // namespace robot_comm
