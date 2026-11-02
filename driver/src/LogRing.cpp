#include "LogRing.h"

#include "JsonUtil.h"

#include <chrono>
#include <sstream>

namespace ots {

LogRing& GlobalLog() {
    static LogRing ring;
    return ring;
}

void LogRing::Add(const char* kind, std::string_view text) {
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.remove_suffix(1);
    const int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::lock_guard lock(mutex_);
    entries_.push_back({nextSeq_++, ms, kind, std::string(text.substr(0, 1000))});
    if (entries_.size() > kCapacity) entries_.pop_front();
}

std::string LogRing::JsonSince(uint64_t seq, size_t maxEntries) {
    std::ostringstream o;
    std::lock_guard lock(mutex_);
    if (seq >= nextSeq_) seq = 0; // client remembers a seq from before SteamVR restarted
    // First entry newer than seq.
    size_t i = 0;
    if (!entries_.empty() && seq >= entries_.front().seq) i = size_t(seq - entries_.front().seq + 1);
    uint64_t next = seq;
    o << "{\"ok\":true,\"entries\":[";
    bool first = true;
    for (size_t n = 0; i < entries_.size() && n < maxEntries; ++i, ++n) {
        const Entry& e = entries_[i];
        o << (first ? "" : ",") << "{\"seq\":" << e.seq << ",\"ms\":" << e.unixMs << ",\"kind\":\"" << e.kind
          << "\",\"text\":" << JsonEscape(e.text) << "}";
        first = false;
        next = e.seq;
    }
    if (first && !entries_.empty() && seq < entries_.front().seq) next = entries_.front().seq - 1;
    o << "],\"next\":" << next << "}";
    return o.str();
}

} // namespace ots
