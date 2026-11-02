#pragma once
// Minimal JSON string escaping for hand-built responses; no parser needed for Phase 0.
#include <cstdio>
#include <string>
#include <string_view>

namespace ots {

inline std::string JsonEscape(std::string_view s) {
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (unsigned char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out.push_back(char(c));
            }
        }
    }
    out.push_back('"');
    return out;
}

} // namespace ots
