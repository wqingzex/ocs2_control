#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "robot_log/flight.hpp"

using robot_log::FlightRecord;
using robot_log::FlightRecorder;
using robot_log::readFlightFile;

namespace
{
    std::string makeTempDir()
    {
        char pattern[] = "/tmp/flight_test_XXXXXX";
        return ::mkdtemp(pattern);
    }

    void removeAll(const std::string& dir)
    {
        const std::string command = "rm -rf '" + dir + "'";
        EXPECT_EQ(std::system(command.c_str()), 0);
    }
}

class Flight : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dir = makeTempDir();
        setenv(robot_log::kFlightDirEnvironment, dir.c_str(), 1);
    }

    void TearDown() override
    {
        unsetenv(robot_log::kFlightDirEnvironment);
        removeAll(dir);
    }

    std::string dir;
};

TEST_F(Flight, WhatIsRecordedIsReadBackInOrderWithTypesAndValues)
{
    auto recorder = FlightRecorder::create("module_a", 64);
    ASSERT_TRUE(recorder);
    recorder->record(7, {1.0, 2.0, 3.0}, 11);
    recorder->record(8, {-4.5}, 22);
    recorder->record(9, {}, 33);

    std::vector<FlightRecord> records;
    std::string module;
    ASSERT_TRUE(readFlightFile(recorder->path(), records, &module));
    EXPECT_EQ(module, "module_a");
    ASSERT_EQ(records.size(), 3u);
    EXPECT_EQ(records[0].type, 7);
    EXPECT_EQ(records[0].aux, 11u);
    EXPECT_EQ(records[0].values, (std::vector<double>{1.0, 2.0, 3.0}));
    EXPECT_EQ(records[1].values, (std::vector<double>{-4.5}));
    EXPECT_TRUE(records[2].values.empty());
    EXPECT_LE(records[0].steady_ns, records[1].steady_ns);
    EXPECT_LE(records[1].steady_ns, records[2].steady_ns);
    EXPECT_GT(records[0].wall_ns, 1'600'000'000ull * 1'000'000'000ull);   // a real wall clock time (after 2020)
}

TEST_F(Flight, MoreThanTwentyEightValuesAreCutNotOverflowed)
{
    auto recorder = FlightRecorder::create("wide", 8);
    ASSERT_TRUE(recorder);
    std::vector<double> many(40, 1.5);
    recorder->record(1, many.data(), many.size());
    std::vector<FlightRecord> records;
    ASSERT_TRUE(readFlightFile(recorder->path(), records));
    ASSERT_EQ(records.size(), 1u);
    EXPECT_EQ(records[0].values.size(), robot_log::kFlightMaxValues);
}

TEST_F(Flight, WhenTheRingWrapsOnlyTheNewestRecordsRemain)
{
    auto recorder = FlightRecorder::create("wrap", 64);
    ASSERT_TRUE(recorder);
    for (int i = 0; i < 200; ++i)
    {
        recorder->record(1, {static_cast<double>(i)});
    }
    std::vector<FlightRecord> records;
    ASSERT_TRUE(readFlightFile(recorder->path(), records));
    ASSERT_EQ(records.size(), 64u);
    for (std::size_t k = 0; k < records.size(); ++k)
    {
        EXPECT_EQ(records[k].values[0], static_cast<double>(136 + k));   // 200 - 64 = 136: oldest first
    }
}

TEST_F(Flight, SkipOldestDropsTheSlotsAWriterIsAboutToReuse)
{
    auto recorder = FlightRecorder::create("skip", 64);
    ASSERT_TRUE(recorder);
    for (int i = 0; i < 200; ++i)
    {
        recorder->record(1, {static_cast<double>(i)});
    }
    std::vector<FlightRecord> records;
    ASSERT_TRUE(readFlightFile(recorder->path(), records, nullptr, 10));
    ASSERT_EQ(records.size(), 54u);
    EXPECT_EQ(records.front().values[0], 146.0);
    EXPECT_EQ(records.back().values[0], 199.0);

    // A ring that has not wrapped has nothing to skip.
    auto fresh = FlightRecorder::create("fresh", 64);
    fresh->record(1, {1.0});
    ASSERT_TRUE(readFlightFile(fresh->path(), records, nullptr, 10));
    EXPECT_EQ(records.size(), 1u);
}

TEST_F(Flight, TheRingOutlivesAProcessThatIsKilled)
{
    const std::string module = "crasher";
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0)
    {
        auto recorder = FlightRecorder::create(module, 4096);
        if (!recorder)
        {
            _exit(2);
        }
        for (int i = 0; i < 1000; ++i)
        {
            recorder->record(5, {static_cast<double>(i), 2.0 * i});
        }
        ::raise(SIGKILL);   // the process dies without running any cleanup
        _exit(3);
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFSIGNALED(status));
    ASSERT_EQ(WTERMSIG(status), SIGKILL);

    std::vector<FlightRecord> records;
    ASSERT_TRUE(readFlightFile(FlightRecorder::directory() + "/" + module + ".ring", records));
    ASSERT_EQ(records.size(), 1000u);
    EXPECT_EQ(records.front().values[0], 0.0);
    EXPECT_EQ(records.back().values[0], 999.0);
    EXPECT_EQ(records.back().values[1], 1998.0);
}

TEST_F(Flight, ARestartKeepsTheOldRingAsPrev)
{
    {
        auto first = FlightRecorder::create("again", 32);
        first->record(1, {111.0});
    }
    auto second = FlightRecorder::create("again", 32);
    ASSERT_TRUE(second);
    second->record(1, {222.0});

    std::vector<FlightRecord> now, before;
    ASSERT_TRUE(readFlightFile(second->path(), now));
    ASSERT_TRUE(readFlightFile(second->path() + ".prev", before));
    ASSERT_EQ(now.size(), 1u);
    ASSERT_EQ(before.size(), 1u);
    EXPECT_EQ(now[0].values[0], 222.0);
    EXPECT_EQ(before[0].values[0], 111.0);
}

TEST_F(Flight, SeveralWritersAndAReaderNeverProduceATornRecord)
{
    auto recorder = FlightRecorder::create("busy", 1024);
    ASSERT_TRUE(recorder);
    constexpr int kWriters = 4;
    constexpr int kPerWriter = 50000;
    std::atomic<bool> done{false};
    std::atomic<int> torn{0};
    std::atomic<std::uint64_t> checked{0};

    // Every record carries (writer, counter, writer * 1e6 + counter): a torn one would not add up.
    std::thread reader([&]() {
        std::vector<FlightRecord> records;
        while (!done.load())
        {
            if (!readFlightFile(recorder->path(), records, nullptr, 64))
            {
                continue;
            }
            for (const auto& r : records)
            {
                ++checked;
                if (r.values.size() != 3 || r.values[2] != r.values[0] * 1.0e6 + r.values[1])
                {
                    ++torn;
                }
            }
        }
    });
    std::vector<std::thread> writers;
    for (int w = 0; w < kWriters; ++w)
    {
        writers.emplace_back([&, w]() {
            for (int i = 0; i < kPerWriter; ++i)
            {
                recorder->record(3, {static_cast<double>(w), static_cast<double>(i), w * 1.0e6 + i});
            }
        });
    }
    for (auto& t : writers) t.join();
    done = true;
    reader.join();

    EXPECT_EQ(recorder->written(), static_cast<std::uint64_t>(kWriters) * kPerWriter);
    EXPECT_EQ(torn.load(), 0);
    EXPECT_GT(checked.load(), 0u);

    // After everything stopped, the ring holds exactly its capacity of intact records.
    std::vector<FlightRecord> records;
    ASSERT_TRUE(readFlightFile(recorder->path(), records));
    EXPECT_EQ(records.size(), 1024u);
    std::set<std::uint64_t> indexes;
    for (const auto& r : records)
    {
        EXPECT_EQ(r.values[2], r.values[0] * 1.0e6 + r.values[1]);
        indexes.insert(r.index);
    }
    EXPECT_EQ(indexes.size(), records.size());
}

TEST_F(Flight, AnImpossibleDirectoryGivesNoRingInsteadOfAnError)
{
    setenv(robot_log::kFlightDirEnvironment, "/proc/no/such/place", 1);
    EXPECT_FALSE(FlightRecorder::create("x"));
}

TEST_F(Flight, ANameThatCouldEscapeTheDirectoryIsRefused)
{
    EXPECT_FALSE(FlightRecorder::create("../evil"));
    EXPECT_FALSE(FlightRecorder::create("a/b"));
    EXPECT_FALSE(FlightRecorder::create(""));
    EXPECT_FALSE(FlightRecorder::create(std::string(64, 'a')));
    EXPECT_TRUE(FlightRecorder::create("arm_controller", 8));
    EXPECT_TRUE(FlightRecorder::create("xr-bridge.1", 8));
}

TEST_F(Flight, ANonRingFileIsRejected)
{
    const std::string path = dir + "/junk.ring";
    FILE* handle = std::fopen(path.c_str(), "wb");
    ASSERT_NE(handle, nullptr);
    const std::string garbage(4096, 'x');
    std::fwrite(garbage.data(), 1, garbage.size(), handle);
    std::fclose(handle);
    std::vector<FlightRecord> records;
    EXPECT_FALSE(readFlightFile(path, records));
    EXPECT_FALSE(readFlightFile(dir + "/missing.ring", records));
}

TEST_F(Flight, RecordingIsCheapEnoughForARealTimeLoop)
{
    auto recorder = FlightRecorder::create("cost", 16384);
    ASSERT_TRUE(recorder);
    constexpr int kCalls = 1'000'000;
    const double values[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < kCalls; ++i)
    {
        recorder->record(4, values, 20);
    }
    const double per_call_ns =
        std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - begin).count() / kCalls;
    std::printf("FlightRecorder::record with 20 values: %.1f ns per call\n", per_call_ns);
    EXPECT_LT(per_call_ns, 2000.0);   // a 500 Hz cycle has 2 000 000 ns
}
