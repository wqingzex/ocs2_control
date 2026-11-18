// The realtime loop stops extrapolating the last MPC policy once no solve has succeeded for the timeout (CTL-016).
#include <gtest/gtest.h>

#include <mpc_core/mpc/PolicyAge.h>

using ocs2::mpc_core::kPolicyStaleTimeoutNs;
using ocs2::mpc_core::policyIsStale;

TEST(PolicyAge, a_recent_solve_is_not_stale)
{
    EXPECT_FALSE(policyIsStale(1'000'000'000, 1'000'000'000));
    EXPECT_FALSE(policyIsStale(1'000'000'000 + kPolicyStaleTimeoutNs, 1'000'000'000));  // exactly at the limit
}

TEST(PolicyAge, past_the_timeout_it_is_stale)
{
    EXPECT_TRUE(policyIsStale(1'000'000'000 + kPolicyStaleTimeoutNs + 1, 1'000'000'000));
}

TEST(PolicyAge, a_new_solve_makes_it_fresh_again)
{
    const std::int64_t last_ok = 5'000'000'000;
    const std::int64_t now = last_ok + 10 * kPolicyStaleTimeoutNs;
    EXPECT_TRUE(policyIsStale(now, last_ok));
    EXPECT_FALSE(policyIsStale(now, now - 1'000'000));
}

TEST(PolicyAge, the_timeout_can_be_given)
{
    EXPECT_TRUE(policyIsStale(200, 0, 100));
    EXPECT_FALSE(policyIsStale(200, 0, 500));
}
