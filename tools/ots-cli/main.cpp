// ots-cli: talks to the driver over the control pipe and runs scripted experiments.
//
//   ots-cli ping | status | watch
//   ots-cli start [label] | stop | mark <label...>
//   ots-cli filter [on|off] | devices | config | set <key> <value>
//   ots-cli protocol static  [--duration 60] [--record]
//   ots-cli protocol occlude [--target right_hand] [--holds 1,3,5,10] [--settle 3] [--repeat 3] [--rest 5] [--record]
//   ots-cli protocol timed   --label NAME [--detail TEXT | --details A,B,C [--switch 10]] [--duration 5] [--settle 2] [--repeat 3] [--rest 5] [--record]
//   ots-cli protocol e4 | e5 [--only stomp,sit] [--repeat 3] [--no-voice] [--record]   voice-guided presets
//   ots-cli voice-check | voice-test
#include "PipeClient.h"
#include "Speech.h"

#include <windows.h>

#include <chrono>
#include <cstdio>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

bool Run(const std::string& line, bool print = true) {
    const auto r = ots::PipeRequest(line);
    if (!r) {
        std::fprintf(stderr, "driver not reachable (is SteamVR running with the trackingstabilizer driver?)\n");
        return false;
    }
    if (print) std::printf("%s\n", r->c_str());
    return r->find("\"ok\":true") != std::string::npos;
}

void SleepSeconds(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

// Audible cues so the experiment can be run from inside the headset.
void CuePrepare() { for (int i = 0; i < 3; ++i) { Beep(660, 120); SleepSeconds(0.88); } }
void CueGo() { Beep(880, 400); }
void CueTick() { Beep(990, 60); SleepSeconds(0.08); Beep(990, 60); }
void CueRelease() { Beep(1320, 250); }
void CueCondition(int n) { for (int i = 0; i < n; ++i) { Beep(440, 300); SleepSeconds(0.25); } }
void CueDone() { Beep(523, 150); Beep(659, 150); Beep(784, 300); }

struct Args {
    std::vector<std::string> positional;
    std::map<std::string, std::string> options; // --key value, or --flag -> "1"

    std::string Get(const std::string& key, const std::string& fallback) const {
        auto it = options.find(key);
        return it == options.end() ? fallback : it->second;
    }
    double GetDouble(const std::string& key, double fallback) const { return std::stod(Get(key, std::to_string(fallback))); }
    int GetInt(const std::string& key, int fallback) const { return std::stoi(Get(key, std::to_string(fallback))); }
    bool Has(const std::string& key) const { return options.count(key) != 0; }
};

Args Parse(int argc, char** argv, int first) {
    Args a;
    for (int i = first; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--", 0) == 0) {
            const std::string key = s.substr(2);
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) a.options[key] = argv[++i];
            else a.options[key] = "1";
        } else {
            a.positional.push_back(s);
        }
    }
    return a;
}

std::vector<double> ParseList(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) if (!item.empty()) out.push_back(std::stod(item));
    return out;
}

struct TrialTiming {
    double hold = 5;   // guaranteed-applied interval (<name>_start .. <name>_uncover)
    double settle = 3; // time to apply / remove the condition by hand without rushing
    double rest = 5;
    int repeat = 3;
};

// Spoken right after the matching beep (long beep / double tick / high beep).
struct Prompts {
    std::wstring act, keep, release;
};

Speaker g_speaker;
bool g_voice = false;

void Say(const std::wstring& text) {
    if (g_voice) g_speaker.Say(text);
}
void SayAndWait(const std::wstring& text) {
    if (g_voice) g_speaker.SayAndWait(text);
}

// One trial:
//   3 short beeps            prepare
//   long beep  -> _cover     apply the condition (cover / hide); `settle` seconds to do it calmly
//   double tick -> _start    condition fully applied, hold still for `hold` seconds
//   high beep  -> _uncover   remove the condition; `settle` seconds to do it calmly
//              -> _end       back to normal, then `rest`
bool RunTrials(const std::string& name, const std::string& detail, const TrialTiming& tm, const Prompts* voice = nullptr) {
    for (int rep = 1; rep <= tm.repeat; ++rep) {
        std::ostringstream tag;
        tag << " " << detail << " hold=" << tm.hold << " rep=" << rep;
        auto mark = [&](const char* phase) { return Run("mark " + name + "_" + phase + tag.str(), false); };

        std::printf("[%s] %s hold=%.1fs trial %d/%d: 準備...\n", name.c_str(), detail.c_str(), tm.hold, rep, tm.repeat);
        if (voice) SayAndWait(std::to_wstring(rep) + L"回目");
        CuePrepare();
        if (!mark("cover")) return false;
        CueGo();
        if (voice) Say(voice->act);
        std::printf("  -> 始める（%.1f 秒以内に、ゆっくりで大丈夫）\n", tm.settle);
        SleepSeconds(tm.settle);
        if (!mark("start")) return false;
        CueTick();
        if (voice) Say(voice->keep);
        std::printf("  -> キープ（%.1f 秒）\n", tm.hold);
        SleepSeconds(tm.hold);
        if (!mark("uncover")) return false;
        CueRelease();
        if (voice) Say(voice->release);
        std::printf("  -> 戻す（%.1f 秒以内に）\n", tm.settle);
        SleepSeconds(tm.settle);
        if (!mark("end")) return false;
        std::printf("  -> 休憩 %.1f 秒\n", tm.rest);
        SleepSeconds(tm.rest);
    }
    return true;
}

TrialTiming ReadTiming(const Args& a, double defaultHold, double defaultSettle) {
    TrialTiming tm;
    tm.hold = a.GetDouble("duration", defaultHold);
    tm.settle = a.GetDouble("settle", defaultSettle);
    tm.rest = a.GetDouble("rest", 5);
    tm.repeat = a.GetInt("repeat", 3);
    return tm;
}

// ---- Voice-guided presets (docs/experiment-guide.md) ----

struct PresetCondition {
    std::string detail;
    std::wstring intro; // spoken in full before the condition's first trial
    Prompts prompts;
    double settle = 0;  // 0 = preset default
};

struct Preset {
    std::string label;
    std::wstring opening;
    TrialTiming timing;
    std::vector<PresetCondition> conditions;
};

std::map<std::string, Preset> BuildPresets() {
    std::map<std::string, Preset> p;

    Preset e5;
    e5.label = "impact";
    e5.opening = L"衝撃と速い動きのテストを始めます。コントローラーを両手に持ち、足のトラッカーを付けて、"
                 L"周りに物がないことを確認してください。それぞれ3回ずつ行います。";
    e5.timing = {3, 1.5, 5, 3};
    e5.conditions = {
        {"action=clap",
         L"条件1、クラップ。ピピッ、の合図から3秒間、両手のコントローラーのリング同士を、軽く2、3回ぶつけてください。"
         L"ピン、の合図で止めます。",
         {L"構えて", L"ぶつけて", L"止めて"}},
        {"action=swing",
         L"条件2、スイング。ピピッ、の合図から3秒間、ボールを投げるように、腕を素早く2、3回振ってください。"
         L"ピン、の合図で止めます。",
         {L"構えて", L"振って", L"止めて"}},
        {"action=stomp",
         L"条件3、足踏み。ピピッ、の合図から3秒間、その場で強めに3、4回、足踏みしてください。ピン、の合図で止めます。",
         {L"構えて", L"踏んで", L"止めて"}},
    };
    p["e5"] = e5;

    Preset e4;
    e4.label = "body";
    e4.opening = L"体の陰になる姿勢のテストを始めます。ヘッドセット、コントローラー、トラッカーを全部付けて、"
                 L"プレイエリアの中央に立ってください。それぞれの姿勢を3回ずつ行います。";
    e4.timing = {3, 4, 5, 3};
    e4.conditions = {
        {"pose=bend",
         L"条件1、前屈。ピー、の合図で深く前屈して、ピピッ、の合図から3秒間、そのまま止まってください。"
         L"ピン、の合図で体を起こします。",
         {L"前屈して", L"そのまま", L"起きて"}},
        {"pose=crouch",
         L"条件2、しゃがむ。ピー、の合図でしゃがんで、ピピッ、の合図から3秒間、そのまま止まってください。"
         L"ピン、の合図で立ち上がります。",
         {L"しゃがんで", L"そのまま", L"立って"}},
        {"pose=sit",
         L"条件3、あぐら。ピー、の合図で床にあぐらで座って、ピピッ、の合図から3秒間、そのまま止まってください。"
         L"ピン、の合図で立ち上がります。",
         {L"座って", L"そのまま", L"立って"}, 5},
        {"pose=lie",
         L"条件4、横向きに寝る。ピー、の合図で床に横向きに寝て、ピピッ、の合図から3秒間、そのまま止まってください。"
         L"ピン、の合図で起き上がって立ちます。",
         {L"寝て", L"そのまま", L"起きて"}, 7},
    };
    p["e4"] = e4;
    return p;
}

bool RunPreset(const Preset& preset, const Args& a) {
    // --only stomp,sit : run just the conditions whose detail contains one of these words.
    std::vector<std::string> only;
    std::stringstream ss(a.Get("only", ""));
    for (std::string item; std::getline(ss, item, ',');)
        if (!item.empty()) only.push_back(item);
    auto selected = [&](const PresetCondition& c) {
        if (only.empty()) return true;
        for (const auto& w : only)
            if (c.detail.find(w) != std::string::npos) return true;
        return false;
    };

    SayAndWait(preset.opening);
    SleepSeconds(a.GetDouble("switch", 3));
    for (size_t i = 0; i < preset.conditions.size(); ++i) {
        const PresetCondition& c = preset.conditions[i];
        if (!selected(c)) continue;
        TrialTiming tm = preset.timing;
        if (c.settle > 0) tm.settle = c.settle;
        tm.repeat = a.GetInt("repeat", tm.repeat);
        std::printf("\n=== 条件 %zu/%zu: %s（低いビープ %zu 回） ===\n", i + 1, preset.conditions.size(), c.detail.c_str(), i + 1);
        CueCondition(int(i + 1));
        SayAndWait(c.intro);
        SleepSeconds(a.GetDouble("switch", 3));
        if (!RunTrials(preset.label, c.detail, tm, &c.prompts)) return false;
    }
    SayAndWait(L"テストは終了です。お疲れさまでした。");
    return true;
}

int Protocol(const Args& a) {
    if (a.positional.empty()) {
        std::fprintf(stderr, "protocol name required: static | occlude | timed\n");
        return 2;
    }
    const std::string name = a.positional[0];
    const bool record = a.Has("record");
    const auto presets = BuildPresets();
    const auto preset = presets.find(name);
    const std::string recordLabel = preset != presets.end() ? preset->second.label
                                    : name == "timed"       ? a.Get("label", name)
                                                            : name;
    if (preset != presets.end() && !a.Has("no-voice")) {
        g_voice = g_speaker.Init();
        if (!g_voice) std::printf("日本語の音声が見つからないため、ビープのみで進めます\n");
    }
    if (record && !Run("start " + recordLabel)) return 1;

    bool ok = true;
    if (preset != presets.end()) {
        ok = RunPreset(preset->second, a);
    } else if (name == "static") {
        const double duration = a.GetDouble("duration", 60);
        std::printf("[static] 全デバイスを置いたまま動かさないでください（%.0f 秒）\n", duration);
        CuePrepare();
        ok = Run("mark static_start duration=" + std::to_string(duration), false);
        CueGo();
        if (ok) SleepSeconds(duration);
        ok = ok && Run("mark static_end", false);
        CueRelease();
    } else if (name == "occlude") {
        const std::string target = a.Get("target", "device");
        // 1s is below and 3/5/10s are above the ~2s point where lighthouse resets tracking.
        const auto holds = ParseList(a.Get("holds", "1,3,5,10"));
        TrialTiming tm = ReadTiming(a, 0, 3);
        for (double hold : holds) {
            tm.hold = hold;
            ok = RunTrials("occlude", "target=" + target, tm);
            if (!ok) break;
        }
    } else if (name == "timed") {
        const std::string label = a.Get("label", "");
        if (label.empty()) {
            std::fprintf(stderr, "timed protocol needs --label\n");
            ok = false;
        } else {
            // --details a,b,c runs the trials once per condition; condition N is announced with N low beeps.
            std::vector<std::string> details;
            std::stringstream ss(a.Get("details", a.Get("detail", "-")));
            for (std::string item; std::getline(ss, item, ',');)
                if (!item.empty()) details.push_back(item);
            const double switchTime = a.GetDouble("switch", 10);
            for (size_t i = 0; i < details.size() && ok; ++i) {
                if (details.size() > 1) {
                    std::printf("\n=== 条件 %zu/%zu: %s（低いビープ %zu 回）。%.0f 秒後に開始 ===\n",
                                i + 1, details.size(), details[i].c_str(), i + 1, switchTime);
                    CueCondition(int(i + 1));
                    SleepSeconds(switchTime);
                }
                ok = RunTrials(label, details[i], ReadTiming(a, 5, 2));
            }
        }
    } else {
        std::fprintf(stderr, "unknown protocol: %s\n", name.c_str());
        ok = false;
    }

    CueDone();
    if (record) Run("stop");
    return ok ? 0 : 1;
}

void Usage() {
    std::printf(
        "usage:\n"
        "  ots-cli ping | status | watch\n"
        "  ots-cli start [label] | stop | mark <label...>\n"
        "  ots-cli filter [on|off] | devices | config | set <key> <value>   (Phase 1 filter, not persisted)\n"
        "  ots-cli log [seq]                                                  (recent driver log lines)\n"
        "  ots-cli protocol static  [--duration 60] [--record]\n"
        "  ots-cli protocol occlude [--target right_hand] [--holds 1,3,5,10] [--settle 3] [--repeat 3] [--rest 5] [--record]\n"
        "  ots-cli protocol timed   --label NAME [--detail TEXT | --details A,B,C [--switch 10]] [--duration 5] [--settle 2] [--repeat 3] [--rest 5] [--record]\n"
        "\n"
        "  ots-cli protocol e4 | e5 [--only stomp,sit] [--repeat 3] [--switch 3] [--no-voice] [--record]   (voice-guided presets)\n"
        "  ots-cli voice-check | voice-test\n"
        "\n"
        "--details runs the trials once per condition; condition N is announced with N low beeps, then --switch s to get ready.\n"
        "trial: 3 short beeps (prepare) -> long beep: apply within --settle s -> double tick: hold still\n"
        "       -> high beep: remove within --settle s -> rest\n");
}

} // namespace

int main(int argc, char** argv) {
    SetConsoleOutputCP(CP_UTF8);
    if (argc < 2) {
        Usage();
        return 2;
    }
    const std::string cmd = argv[1];

    if (cmd == "ping" || cmd == "status" || cmd == "stop" || cmd == "devices" || cmd == "config") return Run(cmd) ? 0 : 1;
    if (cmd == "filter" || cmd == "set" || cmd == "log") {
        std::string line = cmd;
        for (int i = 2; i < argc; ++i) line += " " + std::string(argv[i]);
        return Run(line) ? 0 : 1;
    }
    if (cmd == "start") return Run(argc > 2 ? "start " + std::string(argv[2]) : "start") ? 0 : 1;
    if (cmd == "mark") {
        std::string label;
        for (int i = 2; i < argc; ++i) label += (i > 2 ? " " : "") + std::string(argv[i]);
        return Run("mark " + label) ? 0 : 1;
    }
    if (cmd == "watch") {
        for (;;) {
            if (!Run("status")) return 1;
            SleepSeconds(1);
        }
    }
    if (cmd == "protocol") return Protocol(Parse(argc, argv, 2));
    if (cmd == "voice-check" || cmd == "voice-test") {
        Speaker speaker;
        const bool ok = speaker.Init();
        std::printf("japanese voice: %s\n", ok ? "available" : "not found");
        if (ok && cmd == "voice-test") speaker.SayAndWait(L"音声案内のテストです。聞こえていれば準備完了です。");
        return ok ? 0 : 1;
    }

    Usage();
    return 2;
}
