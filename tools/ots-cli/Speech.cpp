#include "Speech.h"

#include <windows.h>
#include <sapi.h>

Speaker::~Speaker() {
    if (voice_) {
        voice_->WaitUntilDone(INFINITE);
        voice_->Release();
    }
    if (comInitialized_) CoUninitialize();
}

bool Speaker::Init() {
    comInitialized_ = SUCCEEDED(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED));
    if (FAILED(CoCreateInstance(CLSID_SpVoice, nullptr, CLSCTX_ALL, IID_ISpVoice, reinterpret_cast<void**>(&voice_)))) {
        voice_ = nullptr;
        return false;
    }

    ISpObjectTokenCategory* category = nullptr;
    IEnumSpObjectTokens* tokens = nullptr;
    ISpObjectToken* token = nullptr;
    bool japanese = false;
    if (SUCCEEDED(CoCreateInstance(CLSID_SpObjectTokenCategory, nullptr, CLSCTX_ALL, IID_ISpObjectTokenCategory,
                                   reinterpret_cast<void**>(&category))) &&
        SUCCEEDED(category->SetId(SPCAT_VOICES, FALSE)) &&
        SUCCEEDED(category->EnumTokens(L"Language=411", nullptr, &tokens)) &&
        tokens->Next(1, &token, nullptr) == S_OK) {
        japanese = SUCCEEDED(voice_->SetVoice(token));
    }
    if (token) token->Release();
    if (tokens) tokens->Release();
    if (category) category->Release();

    if (!japanese) {
        voice_->Release();
        voice_ = nullptr;
        return false;
    }
    voice_->SetRate(1);
    return true;
}

void Speaker::Say(const std::wstring& text) {
    if (voice_ && !text.empty()) voice_->Speak(text.c_str(), SPF_ASYNC | SPF_PURGEBEFORESPEAK, nullptr);
}

void Speaker::SayAndWait(const std::wstring& text) {
    Say(text);
    Wait();
}

void Speaker::Wait() {
    if (voice_) voice_->WaitUntilDone(INFINITE);
}
