#pragma once

#include <cstdint>

namespace ocs2::mpc_core
{
    /// How long the realtime loop may keep using the last MPC policy after the last successful solve (CTL-016).
    /// Past it the policy is not extrapolated any further and the last command is held. A placeholder to tune on the
    /// robot: it must stay well above the MPC period plus an occasional overrun.
    inline constexpr std::int64_t kPolicyStaleTimeoutNs = 300'000'000;

    /// True when the last successful solve is older than the timeout. Times are steady-clock nanoseconds.
    constexpr bool policyIsStale(std::int64_t now_ns, std::int64_t last_solve_ok_ns,
                                 std::int64_t timeout_ns = kPolicyStaleTimeoutNs)
    {
        return now_ns - last_solve_ok_ns > timeout_ns;
    }
}  // namespace ocs2::mpc_core
