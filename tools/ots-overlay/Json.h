#pragma once
// Minimal JSON reader for the driver's pipe responses (objects, arrays, strings, numbers, bools).
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ots {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    static Json Parse(std::string_view text) {
        size_t i = 0;
        Json j = ParseValue(text, i);
        return j;
    }

    Type type() const { return type_; }
    bool IsNull() const { return type_ == Type::Null; }
    bool Bool(bool fallback = false) const { return type_ == Type::Bool ? b_ : fallback; }
    double Num(double fallback = 0) const { return type_ == Type::Number ? n_ : fallback; }
    const std::string& Str() const { static const std::string empty; return type_ == Type::String ? s_ : empty; }
    const std::vector<Json>& Arr() const { static const std::vector<Json> empty; return type_ == Type::Array ? a_ : empty; }
    const Json& operator[](const char* key) const {
        static const Json null;
        if (type_ != Type::Object) return null;
        auto it = o_.find(key);
        return it == o_.end() ? null : it->second;
    }

private:
    Type type_ = Type::Null;
    bool b_ = false;
    double n_ = 0;
    std::string s_;
    std::vector<Json> a_;
    std::map<std::string, Json> o_;

    static void Ws(std::string_view t, size_t& i) {
        while (i < t.size() && (t[i] == ' ' || t[i] == '\n' || t[i] == '\r' || t[i] == '\t')) ++i;
    }

    static std::string ParseString(std::string_view t, size_t& i) {
        std::string out;
        ++i; // opening quote
        while (i < t.size() && t[i] != '"') {
            char c = t[i++];
            if (c == '\\' && i < t.size()) {
                char e = t[i++];
                switch (e) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'u': {
                    if (i + 4 > t.size()) return out;
                    unsigned cp = unsigned(std::strtoul(std::string(t.substr(i, 4)).c_str(), nullptr, 16));
                    i += 4;
                    if (cp < 0x80) out += char(cp);
                    else if (cp < 0x800) { out += char(0xC0 | (cp >> 6)); out += char(0x80 | (cp & 0x3F)); }
                    else { out += char(0xE0 | (cp >> 12)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
                    break;
                }
                default: out += e;
                }
            } else {
                out += c;
            }
        }
        ++i; // closing quote
        return out;
    }

    static Json ParseValue(std::string_view t, size_t& i) {
        Json j;
        Ws(t, i);
        if (i >= t.size()) return j;
        const char c = t[i];
        if (c == '{') {
            j.type_ = Type::Object;
            ++i;
            for (;;) {
                Ws(t, i);
                if (i >= t.size() || t[i] == '}') { ++i; break; }
                if (t[i] != '"') { ++i; continue; }
                std::string key = ParseString(t, i);
                Ws(t, i);
                if (i < t.size() && t[i] == ':') ++i;
                j.o_[key] = ParseValue(t, i);
                Ws(t, i);
                if (i < t.size() && t[i] == ',') ++i;
            }
        } else if (c == '[') {
            j.type_ = Type::Array;
            ++i;
            for (;;) {
                Ws(t, i);
                if (i >= t.size() || t[i] == ']') { ++i; break; }
                j.a_.push_back(ParseValue(t, i));
                Ws(t, i);
                if (i < t.size() && t[i] == ',') ++i;
            }
        } else if (c == '"') {
            j.type_ = Type::String;
            j.s_ = ParseString(t, i);
        } else if (t.substr(i, 4) == "true") {
            j.type_ = Type::Bool, j.b_ = true, i += 4;
        } else if (t.substr(i, 5) == "false") {
            j.type_ = Type::Bool, j.b_ = false, i += 5;
        } else if (t.substr(i, 4) == "null") {
            i += 4;
        } else {
            char* end = nullptr;
            const std::string rest(t.substr(i, 40));
            j.n_ = std::strtod(rest.c_str(), &end);
            j.type_ = Type::Number;
            i += size_t(end - rest.c_str());
            if (end == rest.c_str()) ++i; // skip garbage
        }
        return j;
    }
};

} // namespace ots
