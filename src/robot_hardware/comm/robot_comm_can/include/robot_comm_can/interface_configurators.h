#pragma once

#include <memory>
#include <string>

#include "robot_comm/can.h"

namespace robot_comm
{
    /**
     * Brings the interface up with `sudo ip link set ...`, exactly like the Juxie driver always did. The command
     * has to match, character for character, the sudoers rule `scripts/setup.sh host` writes, so bitrates other
     * than the Juxie defaults need their own sudoers rule.
     *
     * apply() returns after the interface is configured and a 100 ms pause for the kernel to finish.
     */
    class SudoIpConfigurator final : public CanInterfaceConfigurator
    {
    public:
        /// The shell command for `config` (pure, for tests and logging).
        static std::string buildCommand(const CanConfig& config);

        bool apply(const CanConfig& config) override;
        std::string lastError() const override { return error_; }

    private:
        std::string error_;
    };

    /// Does nothing: the interface is already configured (systemd-networkd, a test bench).
    class NoopConfigurator final : public CanInterfaceConfigurator
    {
    public:
        bool apply(const CanConfig&) override { return true; }
        std::string lastError() const override { return {}; }
    };

    /// SudoIpConfigurator when `config.manage_interface`, otherwise NoopConfigurator.
    std::unique_ptr<CanInterfaceConfigurator> makeConfigurator(const CanConfig& config);
} // namespace robot_comm
