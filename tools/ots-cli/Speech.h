#pragma once
// Japanese voice prompts through SAPI 5 (Microsoft Haruka Desktop on Japanese Windows).
// Falls back to silence when no Japanese voice is installed; beeps still carry the timing.
#include <string>

struct ISpVoice;

class Speaker {
public:
    ~Speaker();
    bool Init();               // picks a Japanese (LANGID 0x411) voice if available
    bool Available() const { return voice_ != nullptr; }
    void Say(const std::wstring& text);  // returns immediately
    void SayAndWait(const std::wstring& text);
    void Wait();

private:
    ISpVoice* voice_ = nullptr;
    bool comInitialized_ = false;
};
