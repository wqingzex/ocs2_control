#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_log/health.hpp"

using robot_log::CycleStats;
using robot_log::RateStats;

namespace
{
    constexpr std::uint64_t kMs = 1'000'000;
    constexpr std::uint64_t kUs = 1'000;
}

// --- CycleStats ---------------------------------------------------------------------------------------------
TEST(CycleStatsTest, AveragesAndMaximaOfAWindow)
{
    CycleStats stats;
    stats.record(2 * kMs, 100 * kUs, 3 * kMs);
    stats.record(2 * kMs, 300 * kUs, 3 * kMs);
    stats.record(4 * kMs, 200 * kUs, 5 * kMs);

    const auto s = stats.take();
    EXPECT_EQ(s.cycles, 3u);
    EXPECT_EQ(s.late, 0u);
    EXPECT_NEAR(s.period_avg_ms, 8.0 / 3.0, 1e-9);
    EXPECT_NEAR(s.period_max_ms, 4.0, 1e-9);
    EXPECT_NEAR(s.duration_avg_us, 200.0, 1e-9);
    EXPECT_NEAR(s.duration_max_us, 300.0, 1e-9);
}

TEST(CycleStatsTest, ALatePeriodIsCounted)
{
    CycleStats stats;
    stats.record(2 * kMs, 100 * kUs, 3 * kMs);
    stats.record(7 * kMs, 100 * kUs, 3 * kMs);   // late
    stats.record(3 * kMs, 100 * kUs, 3 * kMs);   // exactly at the limit: not late
    const auto s = stats.take();
    EXPECT_EQ(s.late, 1u);
    EXPECT_NEAR(s.period_max_ms, 7.0, 1e-9);
}

TEST(CycleStatsTest, TakeStartsANewWindow)
{
    CycleStats stats;
    stats.record(9 * kMs, 900 * kUs, 3 * kMs);
    stats.take();
    const auto empty = stats.take();
    EXPECT_EQ(empty.cycles, 0u);
    EXPECT_EQ(empty.late, 0u);
    EXPECT_EQ(empty.period_max_ms, 0.0);   // the maximum of the last window does not leak into this one
    EXPECT_EQ(empty.period_avg_ms, 0.0);   // and no division by zero
    stats.record(1 * kMs, 10 * kUs, 3 * kMs);
    EXPECT_NEAR(stats.take().period_max_ms, 1.0, 1e-9);
}

TEST(CycleStatsTest, OneWriterAndOneReaderLoseNothing)
{
    CycleStats stats;
    constexpr std::uint64_t kCycles = 400000;
    std::atomic<bool> done{false};
    std::uint64_t seen = 0, late = 0;

    std::thread reader([&]() {
        while (!done.load())
        {
            const auto s = stats.take();
            seen += s.cycles;
            late += s.late;
        }
        const auto rest = stats.take();
        seen += rest.cycles;
        late += rest.late;
    });
    for (std::uint64_t i = 0; i < kCycles; ++i)
    {
        stats.record(2 * kMs, 50 * kUs, i % 1000 == 0 ? 1 * kMs : 3 * kMs);   // 1 in 1000 is late
    }
    done = true;
    reader.join();

    EXPECT_EQ(seen, kCycles);
    EXPECT_EQ(late, kCycles / 1000);
}

TEST(CycleStatsTest, RecordingIsCheapEnoughForARealTimeLoop)
{
    CycleStats stats;
    constexpr int kCalls = 2'000'000;
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < kCalls; ++i)
    {
        stats.record(2 * kMs, 100 * kUs, 3 * kMs);
    }
    const double per_call_ns =
        std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - begin).count() / kCalls;
    std::printf("CycleStats::record: %.1f ns per call\n", per_call_ns);
    EXPECT_LT(per_call_ns, 1000.0);   // a cycle of 500 Hz has 2 000 000 ns; this is far below 1 microsecond
}

TEST(CycleTimerTest, TheFirstCallHasNothingToMeasureFromAndLaterOnesAreRecorded)
{
    CycleStats stats;
    std::uint64_t last_begin = 0;
    {
        robot_log::CycleTimer timer(stats, last_begin, 50 * kMs);
    }
    EXPECT_EQ(stats.take().cycles, 0u);
    EXPECT_NE(last_begin, 0u);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    {
        robot_log::CycleTimer timer(stats, last_begin, 50 * kMs);
    }
    const auto s = stats.take();
    EXPECT_EQ(s.cycles, 1u);
    EXPECT_GE(s.period_max_ms, 5.0);
    EXPECT_EQ(s.late, 0u);
}

TEST(CycleTimerTest, EveryWayOutOfTheFunctionIsRecordedAndALatePeriodIsCounted)
{
    CycleStats stats;
    std::uint64_t last_begin = 0;
    const auto call = [&](bool early) {
        robot_log::CycleTimer timer(stats, last_begin, 3 * kMs);
        if (early)
        {
            return 1;   // an early return still records
        }
        return 2;
    };
    call(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(8));   // longer than the 3 ms limit: late
    call(false);
    call(true);
    const auto s = stats.take();
    EXPECT_EQ(s.cycles, 2u);
    EXPECT_EQ(s.late, 1u);
}

TEST(CycleTimerTest, ZeroLimitSwitchesRecordingOff)
{
    CycleStats stats;
    std::uint64_t last_begin = 0;
    for (int i = 0; i < 3; ++i)
    {
        robot_log::CycleTimer timer(stats, last_begin, 0);
    }
    EXPECT_EQ(stats.take().cycles, 0u);
}

// --- RateStats ----------------------------------------------------------------------------------------------
TEST(RateStatsTest, CountRateAndLargestGap)
{
    RateStats stats;
    stats.recordAt(1'000 * kMs);
    stats.recordAt(1'010 * kMs);
    stats.recordAt(1'020 * kMs);
    stats.recordAt(1'095 * kMs);   // a 75 ms hole
    stats.recordAt(1'100 * kMs);

    const auto s = stats.take(2.0);
    EXPECT_EQ(s.count, 5u);
    EXPECT_NEAR(s.rate_hz, 2.5, 1e-9);
    EXPECT_NEAR(s.gap_max_ms, 75.0, 1e-9);
}

TEST(RateStatsTest, TheGapAcrossTwoWindowsIsStillSeen)
{
    RateStats stats;
    stats.recordAt(1'000 * kMs);
    stats.take(1.0);
    stats.recordAt(1'300 * kMs);   // the first item of the next window arrives 300 ms after the last of the previous
    EXPECT_NEAR(stats.take(1.0).gap_max_ms, 300.0, 1e-9);
}

TEST(RateStatsTest, NothingReceivedIsZero)
{
    RateStats stats;
    const auto s = stats.take(10.0);
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.rate_hz, 0.0);
    EXPECT_EQ(s.gap_max_ms, 0.0);
}

// --- the lines ----------------------------------------------------------------------------------------------
TEST(HealthLine, CycleLineHasTheAgreedKeys)
{
    robot_log::CycleSnapshot s;
    s.cycles = 5000;
    s.late = 2;
    s.period_avg_ms = 2.0;
    s.period_max_ms = 3.5;
    s.duration_avg_us = 120;
    s.duration_max_us = 310;
    EXPECT_EQ(robot_log::formatCycleLine("arm_controller", 10.0, s),
              "[HEALTH] arm_controller window=10.0s cycles=5000 period_avg_ms=2.00 period_max_ms=3.50 late=2 "
              "duration_avg_us=120 duration_max_us=310");
}

TEST(HealthLine, RateLineHasTheAgreedKeysAndOptionalExtras)
{
    robot_log::RateSnapshot s;
    s.count = 2000;
    s.rate_hz = 200.0;
    s.gap_max_ms = 9.1;
    EXPECT_EQ(robot_log::formatRateLine("xr_bridge", 10.0, s),
              "[HEALTH] xr_bridge window=10.0s count=2000 rate_hz=200.0 gap_max_ms=9.1");
    EXPECT_EQ(robot_log::formatRateLine("xr_bridge", 10.0, s, "connected=1 parse_fail=0"),
              "[HEALTH] xr_bridge window=10.0s count=2000 rate_hz=200.0 gap_max_ms=9.1 connected=1 parse_fail=0");
}

TEST(HealthWindow, TenSecondsInNormalAndOneInDebug)
{
    unsetenv(robot_log::kLogModeEnvironment);
    EXPECT_EQ(robot_log::healthWindowSeconds(), 10.0);
    setenv(robot_log::kLogModeEnvironment, "debug", 1);
    EXPECT_EQ(robot_log::healthWindowSeconds(), 1.0);
    unsetenv(robot_log::kLogModeEnvironment);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
