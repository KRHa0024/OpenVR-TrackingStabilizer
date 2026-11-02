#pragma once
// Recent log lines kept in memory for the dashboard overlay (pipe command `log <seq>`).
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>

namespace ots {

class LogRing {
public:
    // kind: "driver" (another driver's IVRDriverLog line, e.g. lighthouse), "ots" (ours), "filter".
    void Add(const char* kind, std::string_view text);

    // {"ok":true,"next":<seq to ask for next>,"entries":[{"seq":..,"ms":<unix ms>,"kind":"..","text":".."}]}
    std::string JsonSince(uint64_t seq, size_t maxEntries);

private:
    struct Entry {
        uint64_t seq;
        int64_t unixMs;
        const char* kind;
        std::string text;
    };
    static constexpr size_t kCapacity = 2000;

    std::mutex mutex_;
    std::deque<Entry> entries_;
    uint64_t nextSeq_ = 1;
};

LogRing& GlobalLog();

} // namespace ots
