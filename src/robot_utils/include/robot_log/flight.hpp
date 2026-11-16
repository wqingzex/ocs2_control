#pragma once

// The flight recorder: what a module received and sent in the last minute or so, kept in a file in shared memory.
//
// Each module owns one ring, /dev/shm/robot_flight/<module>.ring (ROBOT_FLIGHT_DIR changes the directory). Writing a
// record is a few stores into that memory: no lock, no allocation, no system call except the two clock reads, so it can be
// done from a real-time loop. The ring is a file in tmpfs, so it outlives the process: if the module crashes, the file is
// still there and a watcher can copy it afterwards. (It does not survive a reboot or a power cut.) When a module starts
// again, the old file is kept as <module>.ring.prev.
//
// File layout, all little endian:
//   header, 128 bytes: magic "RFLT0001" | u32 version(1) | u32 record_size(256) | u32 capacity | u32 header_size(128)
//                      | char module[32] | u64 start_wall_ns | u64 start_steady_ns | u32 pid | u32 0
//                      | atomic u64 write_index (offset 80: records written so far) | zero padding up to 128
//   capacity slots of record_size bytes: u64 seq | u64 steady_ns | u64 wall_ns | u16 type | u16 count | u32 aux | f64 v[28]
//   Record number i lives in slot i % capacity. A slot is valid when seq == i + 1: the writer clears seq, fills the slot
//   and sets seq last, so a slot that is being written, or has been overwritten, does not match.
//
// The decoder is robot_log_tools/flight.py; the record types are in robot_log/flight_types.hpp and flight_types.yaml.

#include <atomic>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace robot_log
{
    inline constexpr const char* kFlightDirEnvironment = "ROBOT_FLIGHT_DIR";
    inline constexpr const char* kFlightDefaultDir = "/dev/shm/robot_flight";
    inline constexpr std::uint32_t kFlightRecordSize = 256;
    inline constexpr std::uint32_t kFlightHeaderSize = 128;
    inline constexpr std::uint32_t kFlightMaxValues = 28;
    inline constexpr std::uint32_t kFlightDefaultCapacity = 16384;   // 4 MB: about 80 s at 200 records per second

    struct FlightHeader
    {
        char magic[8];
        std::uint32_t version;
        std::uint32_t record_size;
        std::uint32_t capacity;
        std::uint32_t header_size;
        char module[32];
        std::uint64_t start_wall_ns;
        std::uint64_t start_steady_ns;
        std::uint32_t pid;
        std::uint32_t reserved;
        std::atomic<std::uint64_t> write_index;
        char padding[kFlightHeaderSize - 88];
    };
    static_assert(sizeof(FlightHeader) == kFlightHeaderSize, "the header is 128 bytes");
    static_assert(offsetof(FlightHeader, write_index) == 80, "write_index is at offset 80");

    struct FlightSlot
    {
        std::atomic<std::uint64_t> seq;
        std::uint64_t steady_ns;
        std::uint64_t wall_ns;
        std::uint16_t type;
        std::uint16_t count;
        std::uint32_t aux;
        double v[kFlightMaxValues];
    };
    static_assert(sizeof(FlightSlot) == kFlightRecordSize, "a record is 256 bytes");
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "the ring needs lock-free 64 bit atomics");

    /// One record read back from a ring.
    struct FlightRecord
    {
        std::uint64_t index;
        std::uint64_t steady_ns;
        std::uint64_t wall_ns;
        std::uint16_t type;
        std::uint32_t aux;
        std::vector<double> values;
    };

    class FlightRecorder
    {
    public:
        /// Creates (and maps) the ring of a module. The previous file of the same module becomes <module>.ring.prev.
        /// Returns nullptr when the ring cannot be made (no shared memory, no permission): the module then runs without it.
        static std::shared_ptr<FlightRecorder> create(const std::string& module,
                                                      std::uint32_t capacity = kFlightDefaultCapacity)
        {
            if (module.empty() || module.size() >= sizeof(FlightHeader::module) || capacity < 2)
            {
                return nullptr;
            }
            for (char c : module)
            {
                if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.'))
                {
                    return nullptr;   // the name becomes part of a file name
                }
            }
            const std::string dir = directory();
            ::mkdir(dir.c_str(), 0755);
            const std::string path = dir + "/" + module + ".ring";
            ::rename(path.c_str(), (path + ".prev").c_str());   // keep what the last run left; fails harmlessly when none

            const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
            if (fd < 0)
            {
                return nullptr;
            }
            const std::size_t bytes = kFlightHeaderSize + static_cast<std::size_t>(capacity) * kFlightRecordSize;
            if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0)
            {
                ::close(fd);
                return nullptr;
            }
            void* memory = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            ::close(fd);
            if (memory == MAP_FAILED)
            {
                return nullptr;
            }
            auto recorder = std::shared_ptr<FlightRecorder>(new FlightRecorder());
            recorder->memory_ = memory;
            recorder->bytes_ = bytes;
            recorder->capacity_ = capacity;
            recorder->path_ = path;
            recorder->header_ = static_cast<FlightHeader*>(memory);
            recorder->slots_ = reinterpret_cast<FlightSlot*>(static_cast<char*>(memory) + kFlightHeaderSize);

            FlightHeader* header = recorder->header_;
            std::memset(static_cast<void*>(header), 0, kFlightHeaderSize);   // the file is new and zero filled: for clarity
            std::memcpy(header->magic, "RFLT0001", 8);
            header->version = 1;
            header->record_size = kFlightRecordSize;
            header->capacity = capacity;
            header->header_size = kFlightHeaderSize;
            std::memcpy(header->module, module.c_str(), module.size());
            header->start_wall_ns = wallNowNs();
            header->start_steady_ns = steadyNowNs();
            header->pid = static_cast<std::uint32_t>(::getpid());
            header->write_index.store(0, std::memory_order_release);
            return recorder;
        }

        ~FlightRecorder()
        {
            if (memory_ != nullptr)
            {
                ::munmap(memory_, bytes_);
            }
        }

        FlightRecorder(const FlightRecorder&) = delete;
        FlightRecorder& operator=(const FlightRecorder&) = delete;

        /// Adds one record. Lock free, safe from several threads and from a real-time loop. More than 28 values are cut.
        void record(std::uint16_t type, const double* values, std::size_t count, std::uint32_t aux = 0) noexcept
        {
            const std::uint64_t index = header_->write_index.fetch_add(1, std::memory_order_relaxed);
            FlightSlot& slot = slots_[index % capacity_];
            slot.seq.store(0, std::memory_order_release);   // not valid while it is being written
            const std::size_t n = count < kFlightMaxValues ? count : kFlightMaxValues;
            slot.steady_ns = steadyNowNs();
            slot.wall_ns = wallNowNs();
            slot.type = type;
            slot.count = static_cast<std::uint16_t>(n);
            slot.aux = aux;
            for (std::size_t i = 0; i < n; ++i)
            {
                slot.v[i] = values[i];
            }
            slot.seq.store(index + 1, std::memory_order_release);   // valid
        }

        void record(std::uint16_t type, std::initializer_list<double> values, std::uint32_t aux = 0) noexcept
        {
            record(type, values.begin(), values.size(), aux);
        }

        const std::string& path() const { return path_; }
        std::uint32_t capacity() const { return capacity_; }
        std::uint64_t written() const { return header_->write_index.load(std::memory_order_acquire); }

        /// The directory of the rings: ROBOT_FLIGHT_DIR, else /dev/shm/robot_flight.
        static std::string directory()
        {
            const char* text = std::getenv(kFlightDirEnvironment);
            return (text != nullptr && *text != '\0') ? std::string(text) : std::string(kFlightDefaultDir);
        }

        static std::uint64_t steadyNowNs()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch()).count());
        }

        static std::uint64_t wallNowNs()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::system_clock::now().time_since_epoch()).count());
        }

    private:
        FlightRecorder() = default;

        void* memory_{nullptr};
        std::size_t bytes_{0};
        std::uint32_t capacity_{0};
        std::string path_;
        FlightHeader* header_{nullptr};
        FlightSlot* slots_{nullptr};
    };

    /// Reads a ring file back (the C++ side of what flight.py does): the valid records, oldest first. `skip_oldest` drops
    /// that many of the oldest records when the ring has wrapped: those are the slots a writer is about to reuse, and a
    /// copy taken while it runs can mix old and new bytes there. Returns false when the file is not a ring.
    inline bool readFlightFile(const std::string& path, std::vector<FlightRecord>& out, std::string* module = nullptr,
                               std::uint64_t skip_oldest = 0)
    {
        out.clear();
        const int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0)
        {
            return false;
        }
        struct stat info;
        if (::fstat(fd, &info) != 0 || static_cast<std::size_t>(info.st_size) < kFlightHeaderSize)
        {
            ::close(fd);
            return false;
        }
        void* memory = ::mmap(nullptr, static_cast<std::size_t>(info.st_size), PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (memory == MAP_FAILED)
        {
            return false;
        }
        const auto* header = static_cast<const FlightHeader*>(memory);
        const bool ok = std::memcmp(header->magic, "RFLT0001", 8) == 0 && header->version == 1 &&
                        header->record_size == kFlightRecordSize && header->header_size == kFlightHeaderSize &&
                        header->capacity >= 2 &&
                        static_cast<std::size_t>(info.st_size) >=
                            kFlightHeaderSize + static_cast<std::size_t>(header->capacity) * kFlightRecordSize;
        if (ok)
        {
            if (module != nullptr)
            {
                *module = std::string(header->module, strnlen(header->module, sizeof(header->module)));
            }
            const auto* slots = reinterpret_cast<const FlightSlot*>(static_cast<const char*>(memory) + kFlightHeaderSize);
            const std::uint64_t written = header->write_index.load(std::memory_order_acquire);
            const std::uint64_t capacity = header->capacity;
            std::uint64_t first = written > capacity ? written - capacity : 0;
            if (written > capacity)
            {
                first += skip_oldest;
            }
            for (std::uint64_t i = first; i < written; ++i)
            {
                const FlightSlot& slot = slots[i % capacity];
                const std::uint64_t before = slot.seq.load(std::memory_order_acquire);
                if (before != i + 1)
                {
                    continue;
                }
                FlightRecord record;
                record.index = i;
                record.steady_ns = slot.steady_ns;
                record.wall_ns = slot.wall_ns;
                record.type = slot.type;
                record.aux = slot.aux;
                const std::size_t n = slot.count < kFlightMaxValues ? slot.count : kFlightMaxValues;
                record.values.assign(slot.v, slot.v + n);
                if (slot.seq.load(std::memory_order_acquire) != before)
                {
                    continue;   // overwritten while we read it
                }
                out.push_back(std::move(record));
            }
        }
        ::munmap(memory, static_cast<std::size_t>(info.st_size));
        return ok;
    }
} // namespace robot_log
