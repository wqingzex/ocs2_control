#pragma once

// The blackbox: what a module received and sent in the last minute or so, kept in a file in shared memory.
//
// Each module owns one ring, /dev/shm/robot_blackbox/<module>.ring (ROBOT_BLACKBOX_DIR changes the directory). Writing a
// record is a few stores into that memory: no lock, no allocation, no system call except the two clock reads, so it can be
// done from a real-time loop. The ring is a file in tmpfs, so it outlives the process: if the module crashes, the file is
// still there and a watcher can copy it afterwards. (It does not survive a reboot or a power cut.) When a module starts
// again, the old file is kept as <module>.ring.prev.
//
// File layout, all little endian:
//   header, 128 bytes: magic "RBBX0001" | u32 version(1) | u32 record_size(256) | u32 capacity | u32 header_size(128)
//                      | char module[32] | u64 start_wall_ns | u64 start_steady_ns | u32 pid | u32 0
//                      | atomic u64 write_index (offset 80: records written so far) | zero padding up to 128
//   capacity slots of record_size bytes: u64 seq | u64 steady_ns | u64 wall_ns | u16 type | u16 count | u32 aux | f64 v[28]
//   Record number i lives in slot i % capacity. A slot is valid when seq == i + 1: the writer clears seq, fills the slot
//   and sets seq last, so a slot that is being written, or has been overwritten, does not match.
//
// The decoder is robot_log_tools/blackbox.py; the record types are in robot_log/blackbox_types.hpp and blackbox_types.yaml.

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
    inline constexpr const char* kBlackboxDirEnvironment = "ROBOT_BLACKBOX_DIR";
    inline constexpr const char* kBlackboxDefaultDir = "/dev/shm/robot_blackbox";
    inline constexpr std::uint32_t kBlackboxRecordSize = 256;
    inline constexpr std::uint32_t kBlackboxHeaderSize = 128;
    inline constexpr std::uint32_t kBlackboxMaxValues = 28;
    inline constexpr std::uint32_t kBlackboxDefaultCapacity = 16384;   // 4 MB: about 80 s at 200 records per second

    struct BlackboxHeader
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
        char padding[kBlackboxHeaderSize - 88];
    };
    static_assert(sizeof(BlackboxHeader) == kBlackboxHeaderSize, "the header is 128 bytes");
    static_assert(offsetof(BlackboxHeader, write_index) == 80, "write_index is at offset 80");

    struct BlackboxSlot
    {
        std::atomic<std::uint64_t> seq;
        std::uint64_t steady_ns;
        std::uint64_t wall_ns;
        std::uint16_t type;
        std::uint16_t count;
        std::uint32_t aux;
        double v[kBlackboxMaxValues];
    };
    static_assert(sizeof(BlackboxSlot) == kBlackboxRecordSize, "a record is 256 bytes");
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "the ring needs lock-free 64 bit atomics");

    /// One record read back from a ring.
    struct BlackboxRecord
    {
        std::uint64_t index;
        std::uint64_t steady_ns;
        std::uint64_t wall_ns;
        std::uint16_t type;
        std::uint32_t aux;
        std::vector<double> values;
    };

    class BlackboxRecorder
    {
    public:
        /// Creates (and maps) the ring of a module. The previous file of the same module becomes <module>.ring.prev.
        /// Returns nullptr when the ring cannot be made (no shared memory, no permission): the module then runs without it.
        static std::shared_ptr<BlackboxRecorder> create(const std::string& module,
                                                      std::uint32_t capacity = kBlackboxDefaultCapacity)
        {
            if (module.empty() || module.size() >= sizeof(BlackboxHeader::module) || capacity < 2)
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
            const std::size_t bytes = kBlackboxHeaderSize + static_cast<std::size_t>(capacity) * kBlackboxRecordSize;
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
            auto recorder = std::shared_ptr<BlackboxRecorder>(new BlackboxRecorder());
            recorder->memory_ = memory;
            recorder->bytes_ = bytes;
            recorder->capacity_ = capacity;
            recorder->path_ = path;
            recorder->header_ = static_cast<BlackboxHeader*>(memory);
            recorder->slots_ = reinterpret_cast<BlackboxSlot*>(static_cast<char*>(memory) + kBlackboxHeaderSize);

            BlackboxHeader* header = recorder->header_;
            std::memset(static_cast<void*>(header), 0, kBlackboxHeaderSize);   // the file is new and zero filled: for clarity
            std::memcpy(header->magic, "RBBX0001", 8);
            header->version = 1;
            header->record_size = kBlackboxRecordSize;
            header->capacity = capacity;
            header->header_size = kBlackboxHeaderSize;
            std::memcpy(header->module, module.c_str(), module.size());
            header->start_wall_ns = wallNowNs();
            header->start_steady_ns = steadyNowNs();
            header->pid = static_cast<std::uint32_t>(::getpid());
            header->write_index.store(0, std::memory_order_release);
            return recorder;
        }

        ~BlackboxRecorder()
        {
            if (memory_ != nullptr)
            {
                ::munmap(memory_, bytes_);
            }
        }

        BlackboxRecorder(const BlackboxRecorder&) = delete;
        BlackboxRecorder& operator=(const BlackboxRecorder&) = delete;

        /// Adds one record. Lock free, safe from several threads and from a real-time loop. More than 28 values are cut.
        void record(std::uint16_t type, const double* values, std::size_t count, std::uint32_t aux = 0) noexcept
        {
            const std::uint64_t index = header_->write_index.fetch_add(1, std::memory_order_relaxed);
            BlackboxSlot& slot = slots_[index % capacity_];
            slot.seq.store(0, std::memory_order_release);   // not valid while it is being written
            const std::size_t n = count < kBlackboxMaxValues ? count : kBlackboxMaxValues;
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

        /// The directory of the rings: ROBOT_BLACKBOX_DIR, else /dev/shm/robot_blackbox.
        static std::string directory()
        {
            const char* text = std::getenv(kBlackboxDirEnvironment);
            return (text != nullptr && *text != '\0') ? std::string(text) : std::string(kBlackboxDefaultDir);
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
        BlackboxRecorder() = default;

        void* memory_{nullptr};
        std::size_t bytes_{0};
        std::uint32_t capacity_{0};
        std::string path_;
        BlackboxHeader* header_{nullptr};
        BlackboxSlot* slots_{nullptr};
    };

    /// Reads a ring file back (the C++ side of what blackbox.py does): the valid records, oldest first. `skip_oldest` drops
    /// that many of the oldest records when the ring has wrapped: those are the slots a writer is about to reuse, and a
    /// copy taken while it runs can mix old and new bytes there. Returns false when the file is not a ring.
    inline bool readBlackboxFile(const std::string& path, std::vector<BlackboxRecord>& out, std::string* module = nullptr,
                               std::uint64_t skip_oldest = 0)
    {
        out.clear();
        const int fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0)
        {
            return false;
        }
        struct stat info;
        if (::fstat(fd, &info) != 0 || static_cast<std::size_t>(info.st_size) < kBlackboxHeaderSize)
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
        const auto* header = static_cast<const BlackboxHeader*>(memory);
        const bool ok = std::memcmp(header->magic, "RBBX0001", 8) == 0 && header->version == 1 &&
                        header->record_size == kBlackboxRecordSize && header->header_size == kBlackboxHeaderSize &&
                        header->capacity >= 2 &&
                        static_cast<std::size_t>(info.st_size) >=
                            kBlackboxHeaderSize + static_cast<std::size_t>(header->capacity) * kBlackboxRecordSize;
        if (ok)
        {
            if (module != nullptr)
            {
                *module = std::string(header->module, strnlen(header->module, sizeof(header->module)));
            }
            const auto* slots = reinterpret_cast<const BlackboxSlot*>(static_cast<const char*>(memory) + kBlackboxHeaderSize);
            const std::uint64_t written = header->write_index.load(std::memory_order_acquire);
            const std::uint64_t capacity = header->capacity;
            std::uint64_t first = written > capacity ? written - capacity : 0;
            if (written > capacity)
            {
                first += skip_oldest;
            }
            for (std::uint64_t i = first; i < written; ++i)
            {
                const BlackboxSlot& slot = slots[i % capacity];
                const std::uint64_t before = slot.seq.load(std::memory_order_acquire);
                if (before != i + 1)
                {
                    continue;
                }
                BlackboxRecord record;
                record.index = i;
                record.steady_ns = slot.steady_ns;
                record.wall_ns = slot.wall_ns;
                record.type = slot.type;
                record.aux = slot.aux;
                const std::size_t n = slot.count < kBlackboxMaxValues ? slot.count : kBlackboxMaxValues;
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
