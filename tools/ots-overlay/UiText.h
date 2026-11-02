#pragma once
// UI strings in English and Japanese. Static text in app.rml is written as {%key%}; text built at run
// time uses Text() / Fmt().
#include <string>

namespace ots {

// Returns the string for `key` in `lang` ("en" / "ja"); the key itself if it is unknown.
const std::string& Text(const std::string& lang, const std::string& key);

// Text() with "{0}", "{1}" ... replaced by the arguments.
std::string Fmt(const std::string& lang, const std::string& key, const std::string& a0 = {}, const std::string& a1 = {});

// Replaces every {%key%} in `rml` with its XML-escaped translation.
std::string TranslateRml(const std::string& rml, const std::string& lang);

} // namespace ots
