/**
 * Measurements of the arm's flange RS485 ports (COM1 / COM2) through the Marvin SDK.
 * No ROS, no marvin_hardware, no motion: it only sends the Modbus frames you give it on the 485 ports and, in
 * `load`, an *empty* control frame (OnClearSet + OnSetSend, no target state, no joint command) at a fixed rate.
 *
 * What each mode answers (background: docs/待办.md, 末端扁平化 stage 0):
 *   latency   round trip of one request at a time on one port, and how many replies are lost
 *   load      the same while empty control frames go out at 500 Hz from the same thread (like write())
 *   dual      COM1 and COM2 requests in flight together: do both replies arrive, how far apart
 *   threads   whether the SDK survives concurrent calls (1 writer + 1 reader, 2 writers + 1 reader, ...)
 *   --wrap    bare OnSetChData vs OnClearSet / OnSetChData / OnSetSend (does it have to be sandwiched?)
 *
 * The default frame is a read-only Modbus "read input registers" (slave 0x27, FC 04). It goes to whatever device is on
 * the port: check the device and slave id first, and keep the arm still. Nothing is sent before you confirm.
 *
 * Build: colcon build --packages-select marvin_hardware
 * Run:   ./install/marvin_hardware/lib/marvin_hardware/arm_rs485_bench --help
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "MarvinSDK.h"

namespace
{
    using Clock = std::chrono::steady_clock;
    using Bytes = std::vector<uint8_t>;

    constexpr long kChannelCom1 = 2;  // SDK: 1 = CANFD, 2 = COM1 (port1), 3 = COM2 (port2)
    constexpr long kChannelCom2 = 3;
    constexpr size_t kSdkBufferSize = 256;

    std::atomic<bool> g_stop{false};
    void onSignal(int) { g_stop.store(true); }

    // ---- options ----------------------------------------------------------------------------------------------------

    enum class Wrap { None, Sandwich };

    struct Options
    {
        std::string mode;
        std::string ip;
        bool right = false;          // SDK arm B
        int port = 1;                // latency / load / threads 1w1r: 1 = COM1, 2 = COM2
        Bytes frame1 = {0x27, 0x04, 0x00, 0x00, 0x00, 0x07, 0xB6, 0xCE};
        Bytes frame2;                // COM2: no default, the device there is up to you
        Wrap wrap = Wrap::None;
        int count = 500;
        int gap_ms = 2;
        int timeout_ms = 50;
        int duration_s = 10;
        int load_hz = 500;
        int every = 1;               // load: a request every N control cycles
        std::string threads = "1w1r";
        bool allow_unsafe = false;
        bool validate = true;
        bool yes = false;
        std::string csv;             // path prefix
    };

    void usage()
    {
        std::puts(
            "arm_rs485_bench <latency|load|dual|threads> --ip A.B.C.D [options]\n"
            "  --arm left|right        left = SDK arm A (default), right = arm B\n"
            "  --port 1|2              latency / load / threads 1w1r: COM1 (default) or COM2\n"
            "  --frame1 \"27 04 ...\"    request on COM1 (default: Modbus slave 0x27, FC 04, read 7 registers)\n"
            "  --frame2 \"...\"          request on COM2 (required by dual / threads 2w*, and by --port 2)\n"
            "  --wrap none|sandwich    none: bare OnSetChData; sandwich: OnClearSet + OnSetChData + OnSetSend\n"
            "  --count N               latency / dual: requests (default 500)\n"
            "  --gap-ms N              pause between requests (default 2)\n"
            "  --timeout-ms N          give up on a reply after N ms (default 50)\n"
            "  --duration S            load / threads: seconds (default 10)\n"
            "  --load-hz N             load: control cycle rate (default 500)\n"
            "  --every N               load: one request every N cycles (default 1)\n"
            "  --threads 1w1r|2w1r|2w2r  one writer + one reader (default) | one writer per port + one reader |\n"
            "                          one writer per port + two readers (OnGetChData is documented not re-entrant;\n"
            "                          needs --allow-unsafe, may crash this tool)\n"
            "  --no-validate           do not check slave id / function code of replies (non-Modbus frames)\n"
            "  --csv PREFIX            write every round trip (us, -1 = lost) to PREFIX_<mode>_<port>.csv\n"
            "  --yes                   do not ask for confirmation");
    }

    bool parseHex(const std::string& text, Bytes& out)
    {
        out.clear();
        std::string digits;
        for (char c : text)
        {
            if (std::isxdigit(static_cast<unsigned char>(c))) digits.push_back(c);
            else if (c != ' ' && c != ',' && c != ':' && c != '-') return false;
        }
        if (digits.empty() || digits.size() % 2 != 0 || digits.size() / 2 > kSdkBufferSize) return false;
        for (size_t i = 0; i < digits.size(); i += 2)
            out.push_back(static_cast<uint8_t>(std::stoi(digits.substr(i, 2), nullptr, 16)));
        return true;
    }

    bool parseOptions(int argc, char** argv, Options& o)
    {
        if (argc < 2) return false;
        o.mode = argv[1];
        for (int i = 2; i < argc; ++i)
        {
            const std::string key = argv[i];
            if (key == "--allow-unsafe") { o.allow_unsafe = true; continue; }
            if (key == "--no-validate") { o.validate = false; continue; }
            if (key == "--yes") { o.yes = true; continue; }
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "missing value for %s\n", key.c_str());
                return false;
            }
            const std::string v = argv[++i];
            if (key == "--ip") o.ip = v;
            else if (key == "--arm") o.right = v == "right";
            else if (key == "--port") o.port = std::atoi(v.c_str());
            else if (key == "--frame1") { if (!parseHex(v, o.frame1)) return false; }
            else if (key == "--frame2") { if (!parseHex(v, o.frame2)) return false; }
            else if (key == "--wrap") o.wrap = v == "sandwich" ? Wrap::Sandwich : Wrap::None;
            else if (key == "--count") o.count = std::atoi(v.c_str());
            else if (key == "--gap-ms") o.gap_ms = std::atoi(v.c_str());
            else if (key == "--timeout-ms") o.timeout_ms = std::atoi(v.c_str());
            else if (key == "--duration") o.duration_s = std::atoi(v.c_str());
            else if (key == "--load-hz") o.load_hz = std::atoi(v.c_str());
            else if (key == "--every") o.every = std::max(1, std::atoi(v.c_str()));
            else if (key == "--threads") o.threads = v;
            else if (key == "--csv") o.csv = v;
            else
            {
                std::fprintf(stderr, "unknown option %s\n", key.c_str());
                return false;
            }
        }
        const bool known_mode = o.mode == "latency" || o.mode == "load" || o.mode == "dual" || o.mode == "threads";
        const bool known_threads = o.threads == "1w1r" || o.threads == "2w1r" || o.threads == "2w2r";
        const bool single_port = o.mode == "latency" || o.mode == "load" || (o.mode == "threads" && o.threads == "1w1r");
        const bool needs_frame2 = o.mode == "dual" || (o.mode == "threads" && o.threads != "1w1r") ||
                                  (single_port && o.port == 2);
        if (!known_mode || !known_threads || o.ip.empty() || (o.port != 1 && o.port != 2)) return false;
        if (needs_frame2 && o.frame2.empty()) return false;
        if (o.mode == "threads" && o.threads == "2w2r" && !o.allow_unsafe)
        {
            std::fprintf(stderr, "2w2r calls OnGetChData from two threads: add --allow-unsafe if you accept a crash\n");
            return false;
        }
        return o.count > 0 && o.gap_ms >= 0 && o.timeout_ms > 0 && o.duration_s > 0 && o.load_hz > 0;
    }

    // ---- SDK access (arm A = left, B = right) -----------------------------------------------------------------------

    struct ArmApi
    {
        bool (*clear_ch)();
        bool (*set_ch)(unsigned char*, long, long);
        long (*get_ch)(unsigned char*, long*);
    };

    ArmApi apiFor(bool right)
    {
        return right ? ArmApi{OnClearChDataB, OnSetChDataB, OnGetChDataB}
                     : ArmApi{OnClearChDataA, OnSetChDataA, OnGetChDataA};
    }

    bool connect(const std::string& ip)
    {
        unsigned int p[4];
        if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &p[0], &p[1], &p[2], &p[3]) != 4) return false;
        if (p[0] > 255 || p[1] > 255 || p[2] > 255 || p[3] > 255) return false;
        if (!OnLinkTo(static_cast<unsigned char>(p[0]), static_cast<unsigned char>(p[1]),
                      static_cast<unsigned char>(p[2]), static_cast<unsigned char>(p[3])))
            return false;
        usleep(100000);
        OnClearSet();  // an empty control frame: no servo error clearing, nothing that moves the arm
        OnSetSend();
        usleep(100000);
        return true;
    }

    bool sendFrame(const ArmApi& api, const Bytes& frame, long channel, Wrap wrap)
    {
        unsigned char buffer[kSdkBufferSize] = {0};
        std::memcpy(buffer, frame.data(), frame.size());
        if (wrap == Wrap::Sandwich) OnClearSet();
        const bool ok = api.set_ch(buffer, static_cast<long>(frame.size()), channel);
        if (wrap == Wrap::Sandwich) OnSetSend();
        return ok;
    }

    /** A Modbus reply: same slave id and function code (or the exception form, FC | 0x80). */
    bool replyMatches(const Bytes& request, const unsigned char* reply, long size)
    {
        return size >= 2 && request.size() >= 2 && reply[0] == request[0] &&
               (reply[1] == request[1] || reply[1] == (request[1] | 0x80));
    }

    double microsSince(Clock::time_point t0)
    {
        return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    }

    // ---- statistics ---------------------------------------------------------------------------------------------------

    struct Stats
    {
        std::vector<double> samples_us;  // one per request, -1 = no reply in time
        uint64_t sent = 0, ok = 0, lost = 0, rejected = 0, bad_reply = 0;
    };

    /** `sorted` ascending; p in [0, 1]. */
    double percentile(const std::vector<double>& sorted, double p)
    {
        if (sorted.empty()) return 0.0;
        const auto rank = static_cast<size_t>(std::ceil(p * static_cast<double>(sorted.size())));
        return sorted[std::min(sorted.size() - 1, rank == 0 ? 0 : rank - 1)];
    }

    void printStats(const std::string& title, const Stats& s)
    {
        std::vector<double> good;
        for (double x : s.samples_us)
            if (x >= 0) good.push_back(x);
        std::sort(good.begin(), good.end());
        const double loss = s.sent ? 100.0 * static_cast<double>(s.lost) / static_cast<double>(s.sent) : 0.0;
        std::printf("\n=== %s ===\n", title.c_str());
        std::printf("sent %llu  replied %llu  lost %llu (%.2f%%)  refused by SDK %llu  reply not matching request %llu\n",
                    (unsigned long long)s.sent, (unsigned long long)s.ok, (unsigned long long)s.lost, loss,
                    (unsigned long long)s.rejected, (unsigned long long)s.bad_reply);
        if (!good.empty())
            std::printf("round trip us: min %.0f  p50 %.0f  p95 %.0f  p99 %.0f  max %.0f\n", good.front(),
                        percentile(good, 0.50), percentile(good, 0.95), percentile(good, 0.99), good.back());
    }

    void writeCsv(const Options& o, const std::string& port, const Stats& s)
    {
        if (o.csv.empty()) return;
        const std::string path = o.csv + "_" + o.mode + "_" + port + ".csv";
        std::ofstream out(path);
        if (!out)
        {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return;
        }
        out << "index,rtt_us\n";
        for (size_t i = 0; i < s.samples_us.size(); ++i) out << i << ',' << s.samples_us[i] << '\n';
        std::printf("wrote %s\n", path.c_str());
    }

    const Bytes& frameOf(const Options& o, int port) { return port == 1 ? o.frame1 : o.frame2; }
    long channelOf(int port) { return port == 1 ? kChannelCom1 : kChannelCom2; }
    std::string portName(int port) { return "com" + std::to_string(port); }

    // ---- modes --------------------------------------------------------------------------------------------------------

    /** One request at a time on one port. */
    int runLatency(const ArmApi& api, const Options& o)
    {
        Stats s;
        const Bytes& frame = frameOf(o, o.port);
        const long channel = channelOf(o.port);
        api.clear_ch();
        for (int i = 0; i < o.count && !g_stop.load(); ++i)
        {
            const auto t0 = Clock::now();
            if (!sendFrame(api, frame, channel, o.wrap))
            {
                ++s.rejected;
                s.samples_us.push_back(-1);
                std::this_thread::sleep_for(std::chrono::milliseconds(o.gap_ms));
                continue;
            }
            ++s.sent;
            double rtt = -1;
            unsigned char buffer[kSdkBufferSize];
            while (microsSince(t0) < o.timeout_ms * 1000.0)
            {
                long ch = 0;
                const long n = api.get_ch(buffer, &ch);
                if (n <= 0) continue;
                rtt = microsSince(t0);
                ++s.ok;
                if (o.validate && !replyMatches(frame, buffer, n)) ++s.bad_reply;
                break;
            }
            if (rtt < 0)
            {
                ++s.lost;
                api.clear_ch();
            }
            s.samples_us.push_back(rtt);
            std::this_thread::sleep_for(std::chrono::milliseconds(o.gap_ms));
        }
        printStats("latency, COM" + std::to_string(o.port) + ", wrap " + (o.wrap == Wrap::Sandwich ? "sandwich" : "none"), s);
        writeCsv(o, portName(o.port), s);
        return 0;
    }

    /** Empty control frames at load_hz from this thread, with a request every `every` cycles like write() does. */
    int runLoad(const ArmApi& api, const Options& o)
    {
        Stats s;
        std::vector<double> cycle_us;
        const Bytes& frame = frameOf(o, o.port);
        const long channel = channelOf(o.port);
        const auto period = std::chrono::nanoseconds(1'000'000'000LL / o.load_hz);
        const auto end = Clock::now() + std::chrono::seconds(o.duration_s);
        api.clear_ch();

        bool in_flight = false;
        Clock::time_point sent_at;
        Clock::time_point last_start = Clock::now();
        uint64_t cycle = 0, control_failed = 0;
        unsigned char buffer[kSdkBufferSize];
        while (Clock::now() < end && !g_stop.load())
        {
            const auto cycle_start = Clock::now();
            if (cycle > 0) cycle_us.push_back(std::chrono::duration<double, std::micro>(cycle_start - last_start).count());
            last_start = cycle_start;

            OnClearSet();
            if (!OnSetSend()) ++control_failed;
            if (in_flight && microsSince(sent_at) > o.timeout_ms * 1000.0)
            {
                ++s.lost;
                s.samples_us.push_back(-1);
                in_flight = false;
                api.clear_ch();
            }
            if (!in_flight && cycle % static_cast<uint64_t>(o.every) == 0)
            {
                sent_at = Clock::now();
                if (sendFrame(api, frame, channel, Wrap::None))
                {
                    in_flight = true;
                    ++s.sent;
                }
                else
                {
                    ++s.rejected;
                }
            }
            while (Clock::now() < cycle_start + period)  // poll until the next cycle: microsecond resolution
            {
                if (!in_flight) continue;
                long ch = 0;
                const long n = api.get_ch(buffer, &ch);
                if (n <= 0) continue;
                s.samples_us.push_back(microsSince(sent_at));
                ++s.ok;
                if (o.validate && !replyMatches(frame, buffer, n)) ++s.bad_reply;
                in_flight = false;
            }
            ++cycle;
        }
        printStats("load " + std::to_string(o.load_hz) + " Hz, COM" + std::to_string(o.port), s);
        std::sort(cycle_us.begin(), cycle_us.end());
        std::printf("control cycles %llu (OnSetSend failed %llu); cycle period us: p50 %.0f  p99 %.0f  max %.0f (want %lld)\n",
                    (unsigned long long)cycle, (unsigned long long)control_failed, percentile(cycle_us, 0.5),
                    percentile(cycle_us, 0.99), cycle_us.empty() ? 0.0 : cycle_us.back(),
                    (long long)(period.count() / 1000));
        writeCsv(o, portName(o.port), s);
        return 0;
    }

    /** COM1 and COM2 requests back to back, both in flight; replies told apart by the channel the SDK reports. */
    int runDual(const ArmApi& api, const Options& o)
    {
        Stats port[2];
        std::vector<double> gap_us;
        uint64_t both = 0, unexpected_channel = 0, extra_replies = 0;
        unsigned char buffer[kSdkBufferSize];
        api.clear_ch();
        for (int i = 0; i < o.count && !g_stop.load(); ++i)
        {
            const auto t0 = Clock::now();
            bool sent[2];
            for (int p = 0; p < 2; ++p)
            {
                sent[p] = sendFrame(api, frameOf(o, p + 1), channelOf(p + 1), o.wrap);
                if (sent[p]) ++port[p].sent;
                else ++port[p].rejected;
            }
            double rtt[2] = {-1, -1};
            auto done = [&](int p) { return !sent[p] || rtt[p] >= 0; };
            while (microsSince(t0) < o.timeout_ms * 1000.0 && !(done(0) && done(1)))
            {
                long ch = 0;
                const long n = api.get_ch(buffer, &ch);
                if (n <= 0) continue;
                const int p = ch == kChannelCom1 ? 0 : ch == kChannelCom2 ? 1 : -1;
                if (p < 0)
                {
                    ++unexpected_channel;
                    continue;
                }
                if (rtt[p] >= 0)
                {
                    ++extra_replies;
                    continue;
                }
                rtt[p] = microsSince(t0);
                ++port[p].ok;
                if (o.validate && !replyMatches(frameOf(o, p + 1), buffer, n)) ++port[p].bad_reply;
            }
            for (int p = 0; p < 2; ++p)
            {
                if (!sent[p]) continue;
                if (rtt[p] < 0) ++port[p].lost;
                port[p].samples_us.push_back(rtt[p]);
            }
            if (rtt[0] >= 0 && rtt[1] >= 0)
            {
                ++both;
                gap_us.push_back(std::abs(rtt[0] - rtt[1]));
            }
            else
            {
                api.clear_ch();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(o.gap_ms));
        }
        printStats("dual, COM1 (both in flight)", port[0]);
        printStats("dual, COM2 (both in flight)", port[1]);
        std::sort(gap_us.begin(), gap_us.end());
        std::printf("\nrounds with both replies %llu; reply on an unexpected channel %llu; second reply of a port %llu\n",
                    (unsigned long long)both, (unsigned long long)unexpected_channel, (unsigned long long)extra_replies);
        if (!gap_us.empty())
            std::printf("time between the two replies us: p50 %.0f  p95 %.0f  max %.0f\n", percentile(gap_us, 0.5),
                        percentile(gap_us, 0.95), gap_us.back());
        writeCsv(o, portName(1), port[0]);
        writeCsv(o, portName(2), port[1]);
        return 0;
    }

    struct ThreadCounters
    {
        std::atomic<uint64_t> sent[2] = {0, 0}, refused[2] = {0, 0}, received[2] = {0, 0}, mismatched[2] = {0, 0};
        std::atomic<uint64_t> unexpected_channel{0}, bad_size{0};
    };

    void writerLoop(const ArmApi& api, const Options& o, int port, ThreadCounters& c)
    {
        while (!g_stop.load())
        {
            if (sendFrame(api, frameOf(o, port), channelOf(port), Wrap::None)) ++c.sent[port - 1];
            else ++c.refused[port - 1];
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, o.gap_ms)));
        }
    }

    void readerLoop(const ArmApi& api, const Options& o, ThreadCounters& c)
    {
        unsigned char buffer[kSdkBufferSize];
        while (!g_stop.load())
        {
            long ch = 0;
            const long n = api.get_ch(buffer, &ch);
            if (n <= 0) continue;
            if (n > static_cast<long>(kSdkBufferSize))
            {
                ++c.bad_size;
                continue;
            }
            const int p = ch == kChannelCom1 ? 0 : ch == kChannelCom2 ? 1 : -1;
            if (p < 0)
            {
                ++c.unexpected_channel;
                continue;
            }
            ++c.received[p];
            if (o.validate && !replyMatches(frameOf(o, p + 1), buffer, n)) ++c.mismatched[p];
        }
    }

    /** Concurrent SDK use. A crash or a hang here is itself the result: note which variant it was. */
    int runThreads(const ArmApi& api, const Options& o)
    {
        ThreadCounters c;
        std::vector<std::thread> workers;
        api.clear_ch();
        workers.emplace_back(readerLoop, std::cref(api), std::cref(o), std::ref(c));
        if (o.threads == "2w2r") workers.emplace_back(readerLoop, std::cref(api), std::cref(o), std::ref(c));
        if (o.threads == "1w1r")
        {
            workers.emplace_back(writerLoop, std::cref(api), std::cref(o), o.port, std::ref(c));
        }
        else
        {
            workers.emplace_back(writerLoop, std::cref(api), std::cref(o), 1, std::ref(c));
            workers.emplace_back(writerLoop, std::cref(api), std::cref(o), 2, std::ref(c));
        }
        const auto end = Clock::now() + std::chrono::seconds(o.duration_s);
        while (Clock::now() < end && !g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::this_thread::sleep_for(std::chrono::seconds(1));  // let late replies arrive
        g_stop.store(true);
        for (auto& w : workers) w.join();

        std::printf("\n=== threads %s, %d s ===\n", o.threads.c_str(), o.duration_s);
        for (int p = 0; p < 2; ++p)
        {
            if (c.sent[p] == 0 && c.refused[p] == 0) continue;
            const double missing =
                c.sent[p] ? 100.0 * (1.0 - static_cast<double>(c.received[p]) / static_cast<double>(c.sent[p])) : 0.0;
            std::printf("COM%d: sent %llu  refused %llu  received %llu  (missing %.2f%%)  not matching request %llu\n", p + 1,
                        (unsigned long long)c.sent[p], (unsigned long long)c.refused[p],
                        (unsigned long long)c.received[p], missing, (unsigned long long)c.mismatched[p]);
        }
        std::printf("reply on an unexpected channel %llu; reply longer than the buffer %llu\n",
                    (unsigned long long)c.unexpected_channel, (unsigned long long)c.bad_size);
        return 0;
    }

    bool confirmed(const Options& o)
    {
        std::printf("\nmode %s on arm %s (%s), COM%d%s\n", o.mode.c_str(), o.right ? "B/right" : "A/left", o.ip.c_str(),
                    o.port, o.mode == "dual" ? " + COM2" : "");
        std::puts("This sends the frame(s) to the device on the port(s) and, in load, empty control frames; nothing that");
        std::puts("moves the arm. The arm must be still, in a safe state, with the emergency stop in reach.");
        if (o.yes) return true;
        std::printf("Type YES to continue: ");
        std::string answer;
        std::getline(std::cin, answer);
        return answer == "YES";
    }
}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
    {
        usage();
        return 2;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    if (!confirmed(options))
    {
        std::puts("not confirmed, nothing sent");
        return 1;
    }
    if (!connect(options.ip))
    {
        std::fprintf(stderr, "cannot connect to %s\n", options.ip.c_str());
        return 1;
    }
    const ArmApi api = apiFor(options.right);
    int rc = 0;
    if (options.mode == "latency") rc = runLatency(api, options);
    else if (options.mode == "load") rc = runLoad(api, options);
    else if (options.mode == "dual") rc = runDual(api, options);
    else rc = runThreads(api, options);
    OnRelease();
    return rc;
}
