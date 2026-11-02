#include "DriverClient.h"

#include "Json.h"
#include "PipeClient.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sstream>

namespace ots {
namespace {

int64_t SteadyMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::vector<std::string> SplitCsv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    for (std::string item; std::getline(ss, item, ',');)
        if (!item.empty()) out.push_back(item);
    return out;
}

} // namespace

void AppState::AddUiLog(const std::string& text) {
    // caller must NOT hold m
    std::lock_guard lock(m);
    LogEntry e;
    e.kind = "ui";
    e.text = text;
    e.unixMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    log.push_back(std::move(e));
}

void DriverClient::Start() {
    stop_ = false;
    thread_ = std::thread(&DriverClient::Loop, this);
}

void DriverClient::Stop() {
    stop_ = true;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void DriverClient::Send(const std::string& command) {
    {
        std::lock_guard lock(queueMutex_);
        if (command.rfind("set ", 0) == 0) {
            const std::string key = command.substr(0, command.find(' ', 4) + 1); // "set <key> "
            for (auto it = queue_.begin(); it != queue_.end();)
                it = it->rfind(key, 0) == 0 ? queue_.erase(it) : it + 1;
        }
        queue_.push_back(command);
    }
    cv_.notify_all();
}

void DriverClient::Loop() {
    auto lastPoll = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    while (!stop_) {
        const bool active = active_, fast = modelFast_;
        const auto period = std::chrono::milliseconds(active ? 100 : 250);
        std::deque<std::string> pending;
        {
            std::unique_lock lock(queueMutex_);
            const auto wait = fast ? std::chrono::milliseconds(33)
                                   : std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::max(std::chrono::steady_clock::duration::zero(), lastPoll + period - std::chrono::steady_clock::now()));
            cv_.wait_for(lock, wait, [&] { return stop_ || !queue_.empty(); });
            pending.swap(queue_);
        }
        for (const auto& c : pending) {
            const auto reply = PipeRequest(c, 500);
            const Json j = reply ? Json::Parse(*reply) : Json();
            if (!reply) state_.AddUiLog(c + " -> driver not reachable");
            else if (!j["ok"].Bool()) state_.AddUiLog(c + " -> error: " + j["error"].Str());
        }
        if (stop_) break;
        // Hidden UI: a lighter poll every 250 ms keeps the log and device list current.
        if (std::chrono::steady_clock::now() - lastPoll >= period) {
            lastPoll = std::chrono::steady_clock::now();
            Poll(active);
        }
        if (active || fast) PollModel();
    }
}

void DriverClient::PollModel() {
    {
        std::lock_guard lock(state_.m);
        if (!state_.driverConnected) return;
    }
    const auto reply = PipeRequest("model", 100);
    const Json j = reply ? Json::Parse(*reply) : Json();
    std::vector<ModelPose> models;
    if (j["ok"].Bool()) {
        auto vec = [](const Json& a, Vec3* v) {
            const auto& e = a.Arr();
            if (e.size() < 3) return false;
            *v = {float(e[0].Num() / 1000), float(e[1].Num() / 1000), float(e[2].Num() / 1000)};
            return true;
        };
        for (const Json& d : j["devices"].Arr()) {
            ModelPose m;
            m.serial = d["serial"].Str();
            m.hand = d["kind"].Str() == "hand";
            m.estimating = d["estimating"].Bool();
            if (!vec(d["neck"], &m.neck) || !vec(d["end"], &m.end)) continue;
            m.haveShoulder = vec(d["shoulder"], &m.shoulder);
            m.haveElbow = vec(d["elbow"], &m.elbow);
            models.push_back(std::move(m));
        }
    }
    std::lock_guard lock(state_.m);
    state_.models = std::move(models);
}

void DriverClient::Poll(bool withTrace) {
    const auto status = PipeRequest("status", 200);
    if (!status) {
        std::lock_guard lock(state_.m);
        state_.driverConnected = false;
        return;
    }
    const auto devices = PipeRequest("devices", 200);
    const auto config = PipeRequest("config", 200);
    uint64_t since;
    {
        std::lock_guard lock(state_.m);
        since = state_.logNext;
    }
    const auto log = PipeRequest("log " + std::to_string(since), 200);
    std::optional<std::string> trace;
    if (withTrace) {
        char cmd[64];
        std::snprintf(cmd, sizeof(cmd), "trace %.3f", traceSince_);
        trace = PipeRequest(cmd, 300);
    }

    const Json js = Json::Parse(*status);
    const Json jd = devices ? Json::Parse(*devices) : Json();
    const Json jc = config ? Json::Parse(*config) : Json();
    const Json jl = log ? Json::Parse(*log) : Json();
    const Json jt = trace ? Json::Parse(*trace) : Json();

    std::lock_guard lock(state_.m);
    state_.driverConnected = true;
    state_.driverStatusRaw = *status;
    state_.recording = js["recording"].Bool();
    state_.recordingPath = js["path"].Str();

    if (jd["ok"].Bool()) {
        state_.filterDevices.clear();
        for (const Json& d : jd["devices"].Arr()) {
            FilterDeviceInfo f;
            f.serial = d["serial"].Str();
            f.bodyReady = d["bodyReady"].Bool();
            f.handReady = d["handReady"].Bool();
            f.handActive = d["handActive"].Bool();
            f.handSide = int(d["handSide"].Num());
            f.lighthouse = d["lighthouse"].Bool(true);
            f.bodyActive = d["bodyActive"].Bool();
            f.torsoLengthCm = float(d["torsoLengthCm"].Num());
            f.imuOpen = d["imuOpen"].Bool();
            f.imuAligned = d["imuAligned"].Bool();
            f.imuActive = d["imuActive"].Bool();
            f.imuError = int(d["imuError"].Num(-1));
            f.imuRate = float(d["imuRate"].Num());
            f.mode = d["mode"].Str();
            f.state = d["state"].Str();
            f.stateSeconds = d["stateSeconds"].Num();
            f.holds = int(d["holds"].Num());
            f.emitted = int64_t(d["emitted"].Num());
            f.stepsLimited = int64_t(d["stepsLimited"].Num());
            state_.filterDevices[uint32_t(d["index"].Num())] = f;
        }
    }
    if (jc["ok"].Bool()) {
        const Json& c = jc["config"];
        FilterConfig& f = state_.filter;
        f.valid = true;
        f.enabled = c["filterEnabled"].Bool();
        f.controllers = c["filterControllers"].Bool(true);
        f.trackers = c["filterTrackers"].Bool(true);
        f.hmd = c["filterHmd"].Bool();
        f.imuEnabled = c["imuEnabled"].Bool();
        f.bodyEnabled = c["bodyEnabled"].Bool();
        f.handEnabled = c["handEnabled"].Bool();
        f.handShoulderWidthCm = float(c["handShoulderWidthCm"].Num(36));
        f.handArmReachCm = float(c["handArmReachCm"].Num(70));
        f.handMaxPredictionMs = float(c["handMaxPredictionMs"].Num(3000));
        f.bodyWaistSerial = c["bodyWaistSerial"].Str();
        f.bodyHmdSerial = c["bodyHmdSerial"].Str();
        f.bodyHmdAuto = c["bodyHmdAuto"].Str();
        f.smoothing = float(c["smoothing"].Num(0));
        f.probationMs = float(c["probationMs"].Num(400));
        f.shortProbationMs = float(c["shortProbationMs"].Num(100));
        f.blendMs = float(c["blendMs"].Num(200));
        f.stepLimitCm = float(c["stepLimitCm"].Num(4.5));
        f.stepBlendMs = float(c["stepBlendMs"].Num(150));
        f.includeSerials = SplitCsv(c["filterIncludeSerials"].Str());
        f.excludeSerials = SplitCsv(c["filterExcludeSerials"].Str());
    }
    if (jl["ok"].Bool()) {
        const uint64_t next = uint64_t(jl["next"].Num());
        if (next < state_.logNext) state_.log.clear(); // driver restarted
        for (const Json& e : jl["entries"].Arr()) {
            LogEntry le;
            le.seq = uint64_t(e["seq"].Num());
            le.unixMs = int64_t(e["ms"].Num());
            le.kind = e["kind"].Str();
            le.text = e["text"].Str();
            state_.log.push_back(std::move(le));
        }
        state_.logNext = next;
        if (state_.log.size() > 3000) state_.log.erase(state_.log.begin(), state_.log.begin() + (state_.log.size() - 3000));
    }
    if (withTrace) {
        state_.traceSupported = jt["ok"].Bool();
        if (state_.traceSupported) {
            const double now = jt["now"].Num();
            if (now < state_.traceNow) state_.trace.clear(); // driver restarted
            state_.traceNow = now;
            state_.traceNowLocalMs = SteadyMs();
            for (const Json& d : jt["devices"].Arr()) {
                auto& samples = state_.trace[d["serial"].Str()];
                for (const Json& s : d["s"].Arr()) {
                    const auto& a = s.Arr();
                    if (a.size() < 9) continue;
                    TraceSample t;
                    t.t = a[0].Num();
                    t.raw = {float(a[1].Num() / 1000), float(a[2].Num() / 1000), float(a[3].Num() / 1000)};
                    t.out = {float(a[4].Num() / 1000), float(a[5].Num() / 1000), float(a[6].Num() / 1000)};
                    t.rawValid = a[7].Num() != 0;
                    t.state = a[8].Str().empty() ? 'T' : a[8].Str()[0];
                    if (!samples.empty() && t.t <= samples.back().t) continue;
                    samples.push_back(t);
                    traceSince_ = std::max(traceSince_, t.t);
                }
            }
            for (auto& [serial, samples] : state_.trace) {
                size_t drop = 0;
                while (drop < samples.size() && samples[drop].t < now - 25) ++drop;
                samples.erase(samples.begin(), samples.begin() + drop);
            }
        }
    }
}

} // namespace ots
