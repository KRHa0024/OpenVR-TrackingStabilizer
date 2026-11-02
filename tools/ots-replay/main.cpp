// ots-replay: run recorded poses through ots::filter::TrackingFilter offline.
//
//   ots-replay <in.otsrec> [-o out.otsrec] [--hmd] [--emit-hz 250] [--set key=value ...]
//     [--body-waist N --body-hmd N --set bodyEnabled=1] [--mask start:end ...]
//
// Writes a copy of the session where every pose of a filtered device is replaced by the filter's
// output, poses synthesized by the emitter are added (hookSlot 100), and state transitions are
// added as FilterEvent records. All other records are copied unchanged, so the Phase 0 analysis
// scripts work on the output as they do on raw recordings.
#include "RecordFormat.h"
#include "TrackingFilter.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rec = ots::rec;
using ots::filter::Config;
using ots::filter::TrackingFilter;

namespace {

struct Record {
    rec::RecordHeader h;
    std::vector<uint8_t> payload;
};

class Reader {
public:
    explicit Reader(const char* path) : f_(std::fopen(path, "rb")) {}
    ~Reader() { if (f_) std::fclose(f_); }
    bool Open(rec::FileHeader* header) {
        if (!f_ || std::fread(header, sizeof(*header), 1, f_) != 1) return false;
        if (std::memcmp(header->magic, rec::kMagic, sizeof(rec::kMagic)) != 0 || header->version != rec::kVersion) return false;
        std::fseek(f_, long(header->headerSize), SEEK_SET);
        return true;
    }
    bool Next(Record* r) {
        if (std::fread(&r->h, sizeof(r->h), 1, f_) != 1) return false;
        r->payload.resize(r->h.payloadSize);
        return r->h.payloadSize == 0 || std::fread(r->payload.data(), r->h.payloadSize, 1, f_) == 1;
    }
    void Rewind(const rec::FileHeader& header) { std::fseek(f_, long(header.headerSize), SEEK_SET); }

private:
    FILE* f_;
};

void Write(FILE* out, rec::RecordType type, int64_t qpc, const void* a, uint32_t aSize, const void* b = nullptr, uint32_t bSize = 0) {
    rec::RecordHeader h{};
    h.type = uint16_t(type);
    h.payloadSize = aSize + bSize;
    h.qpc = qpc;
    std::fwrite(&h, sizeof(h), 1, out);
    if (aSize) std::fwrite(a, aSize, 1, out);
    if (bSize) std::fwrite(b, bSize, 1, out);
}

void WritePose(FILE* out, int64_t qpc, uint32_t device, uint32_t slot, const vr::DriverPose_t& pose) {
    rec::PosePayloadHeader ph{device, uint32_t(sizeof(pose)), slot};
    Write(out, rec::RecordType::Pose, qpc, &ph, sizeof(ph), &pose, sizeof(pose));
}

// Minimal extraction of an integer field from the DeviceInfo JSON written by the driver.
bool JsonInt(const std::string& json, const char* key, int* value) {
    const std::string k = std::string("\"") + key + "\":";
    const size_t at = json.find(k);
    if (at == std::string::npos) return false;
    *value = std::atoi(json.c_str() + at + k.size());
    return true;
}

bool SetConfig(Config& c, const std::string& kv) {
    const size_t eq = kv.find('=');
    if (eq == std::string::npos) return false;
    const std::string k = kv.substr(0, eq);
    char* end = nullptr;
    const char* value = kv.c_str()+eq+1;
    const double v = std::strtod(value,&end);
    if (end == value || *end != '\0' || !std::isfinite(v) || v < 0) return false;
    std::map<std::string, double*> d = {
        {"suspectWindow", &c.suspectWindow}, {"shortInterval", &c.shortInterval}, {"suspectMax", &c.suspectMax},
        {"streamTimeout", &c.streamTimeout}, {"probation", &c.probation}, {"shortProbation", &c.shortProbation}, {"blend", &c.blend},
        {"stepLimit", &c.stepLimit}, {"stepBlend", &c.stepBlend}, {"stepMaxDt", &c.stepMaxDt}, {"maxHold", &c.maxHold},
        {"smoothing", &c.smoothing}}; // 0..1
    if (auto it = d.find(k); it != d.end()) { *it->second = v; return true; }
    if (k == "suspectEnabled") { c.suspectEnabled = v != 0; return true; }
    if (k == "holdReportsValid") { c.holdReportsValid = v != 0; return true; }
    if (k == "bodyEnabled") { c.body.enabled = v != 0; return true; }
    if (k == "handEnabled") { c.hand.enabled = v != 0; return true; }
    if (k == "handShoulderWidth") { c.hand.shoulderWidth = v; return v >= .2 && v <= .6; }
    if (k == "handArmReach") { c.hand.armReach = v; return v >= .3 && v <= 1.2; }
    if (k == "handMaxPrediction") { c.hand.maxPrediction = v; return v >= .1 && v <= 3; }
    if (k == "handArmFollow") { c.hand.armFollow = v; return v >= 0 && v <= 1; }
    if (k == "handForearm") { c.hand.forearm = v; return v >= .1 && v <= .5; }
    if (k == "handFollowBody") { c.hand.followBody = v != 0; return true; }
    if (k == "handFollowDeadzone") { c.hand.followDeadzone = v; return v >= 0 && v <= .5; }
    if (k == "bodyNeckDown") { c.body.neckDown = v; return std::isfinite(v) && v >= 0 && v <= .30; }
    if (k == "bodyNeckBack") { c.body.neckBack = v; return std::isfinite(v) && v >= 0 && v <= .30; }
    if (k == "suspectArmCount") { c.suspectArmCount = int(v); return true; }
    return false;
}

std::string DefaultOutput(const std::string& in) {
    const size_t dot = in.rfind(".otsrec");
    return (dot == std::string::npos ? in : in.substr(0, dot)) + ".filtered.otsrec";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: ots-replay <in.otsrec> [-o out.otsrec] [--hmd] [--emit-hz 250] [--set key=value ...]\n"
                             "  [--body-waist N --body-hmd N --set bodyEnabled=1] [--mask start:end ...]\n"
                             "  [--reference-hmd N --set handEnabled=1] [--mask-device N ... --mask start:end ...]\n");
        return 2;
    }
    std::string inPath = argv[1], outPath;
    bool includeHmd = false;
    double emitHz = 250;
    Config cfg;
    int waistIndex = -1, headIndex = -1;
    std::vector<std::pair<double,double>> masks;
    std::vector<int> maskDevices;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-o" && i + 1 < argc) outPath = argv[++i];
        else if (a == "--hmd") includeHmd = true;
        else if (a == "--emit-hz" && i + 1 < argc) emitHz = std::atof(argv[++i]);
        else if (a == "--body-waist" && i+1 < argc) waistIndex = std::atoi(argv[++i]);
        else if ((a == "--body-hmd" || a == "--reference-hmd") && i+1 < argc) headIndex = std::atoi(argv[++i]);
        else if (a == "--mask-device" && i+1 < argc) maskDevices.push_back(std::atoi(argv[++i]));
        else if (a == "--mask" && i+1 < argc) {
            double begin = 0, end = 0; char tail = 0;
            if (std::sscanf(argv[++i],"%lf:%lf%c",&begin,&end,&tail) != 2 || !std::isfinite(begin) || !std::isfinite(end) || begin < 0 || end <= begin) {
                std::fprintf(stderr,"mask must be start:end in seconds\n"); return 2;
            }
            masks.emplace_back(begin,end);
        }
        else if (a == "--set" && i + 1 < argc) {
            if (!SetConfig(cfg, argv[++i])) { std::fprintf(stderr, "unknown setting: %s\n", argv[i]); return 2; }
        } else { std::fprintf(stderr, "unknown argument: %s\n", a.c_str()); return 2; }
    }
    if (outPath.empty()) outPath = DefaultOutput(inPath);
    if (!std::isfinite(emitHz) || emitHz < 10 || emitHz > 1000) { std::fprintf(stderr,"emit Hz must be 10..1000\n"); return 2; }
    std::error_code pathError;
    if (std::filesystem::equivalent(inPath,outPath,pathError) ||
        std::filesystem::absolute(inPath).lexically_normal() == std::filesystem::absolute(outPath).lexically_normal()) {
        std::fprintf(stderr,"output must differ from input\n"); return 2;
    }
    if (!masks.empty() && maskDevices.empty() && waistIndex >= 0) maskDevices.push_back(waistIndex);
    if ((cfg.body.enabled && (waistIndex < 0 || waistIndex == headIndex)) ||
        ((cfg.body.enabled || cfg.hand.enabled || !masks.empty()) && headIndex < 0) ||
        (!masks.empty() && maskDevices.empty())) {
        std::fprintf(stderr,"specify an HMD and explicit model/mask targets\n"); return 2;
    }

    Reader reader(inPath.c_str());
    rec::FileHeader header;
    if (!reader.Open(&header)) { std::fprintf(stderr, "cannot read %s\n", inPath.c_str()); return 1; }

    // Pass 1: which device indices are controllers / trackers (HMD optional).
    std::map<uint32_t, int> deviceClass;
    std::map<uint32_t, int> handSides;
    Record r;
    while (reader.Next(&r))
        if (r.h.type == uint16_t(rec::RecordType::DeviceInfo)) {
            const std::string json(r.payload.begin(), r.payload.end());
            int index, cls;
            if (JsonInt(json, "index", &index) && JsonInt(json, "class", &cls)) {
                deviceClass[uint32_t(index)] = cls;
                int role = 0;
                JsonInt(json,"role",&role);
                handSides[uint32_t(index)] = cls == vr::TrackedDeviceClass_Controller ?
                    (role == vr::TrackedControllerRole_LeftHand ? -1 : role == vr::TrackedControllerRole_RightHand ? 1 : 0) : 0;
            }
        }
    auto filtered = [&](uint32_t d) {
        auto it = deviceClass.find(d);
        if (it == deviceClass.end()) return false;
        return it->second == vr::TrackedDeviceClass_Controller || it->second == vr::TrackedDeviceClass_GenericTracker ||
               (includeHmd && it->second == vr::TrackedDeviceClass_HMD);
    };
    if ((cfg.body.enabled && deviceClass[uint32_t(waistIndex)] != vr::TrackedDeviceClass_GenericTracker) ||
        ((cfg.body.enabled || cfg.hand.enabled || !masks.empty()) && deviceClass[uint32_t(headIndex)] != vr::TrackedDeviceClass_HMD)) {
        std::fprintf(stderr,"body roles must map to a GenericTracker and an HMD in DeviceInfo\n"); return 2;
    }
    for (const int d : maskDevices) {
        if (d < 0 || d == headIndex || !filtered(uint32_t(d))) { std::fprintf(stderr,"mask target must be a filtered controller/tracker\n"); return 2; }
    }
    if (cfg.hand.enabled && std::none_of(handSides.begin(),handSides.end(),[](const auto& entry) { return entry.second != 0; })) {
        std::fprintf(stderr,"hand model requires left/right controller roles in DeviceInfo\n"); return 2;
    }

    FILE* out = std::fopen(outPath.c_str(), "wb");
    if (!out) { std::fprintf(stderr, "cannot write %s\n", outPath.c_str()); return 1; }
    rec::FileHeader outHeader = header;
    std::snprintf(outHeader.label, sizeof(outHeader.label), "%s+filter", header.label);
    std::fwrite(&outHeader, sizeof(outHeader), 1, out);

    const double freq = double(header.qpcFrequency);
    auto seconds = [&](int64_t qpc) { return double(qpc - header.qpcStart) / freq; };
    auto qpcOf = [&](double t) { return header.qpcStart + int64_t(t * freq); };

    std::map<uint32_t, std::unique_ptr<TrackingFilter>> filters;
    vr::DriverPose_t head{};
    double headT = -1;
    vr::DriverPose_t rawHands[2]{};
    double rawHandTimes[2] = {-1,-1};
    auto references = [&](uint32_t d, TrackingFilter& f) {
        if (int(d) == waistIndex) f.SetBodyReference(head,headT);
        if (f.GetConfig().hand.enabled) {
            const int other = f.GetConfig().hand.side < 0 ? 1 : 0;
            f.SetHandReference(head,headT,rawHands[other],rawHandTimes[other]);
        }
    };
    std::map<uint32_t,int> bodyStatus;
    std::map<uint32_t,uint64_t> bodySamples;
    std::map<uint32_t,int> handStatus;
    std::map<uint32_t,uint64_t> handSamples;
    long long processNs = 0, processCalls = 0;
    std::map<uint32_t, uint64_t> emittedCount, transitionsCount;
    auto flushTransitions = [&](uint32_t d, TrackingFilter& f) {
        for (const auto& tr : f.TakeTransitions()) {
            char buf[256];
            const int n = std::snprintf(buf, sizeof(buf), "{\"device\":%u,\"from\":\"%s\",\"to\":\"%s\",\"reason\":\"%s\"}", d,
                                        ToString(tr.from), ToString(tr.to), tr.reason);
            Write(out, rec::RecordType::FilterEvent, qpcOf(tr.t), buf, uint32_t(n));
            ++transitionsCount[d];
        }
    };
    auto bodyEvent = [&](uint32_t d, TrackingFilter& f, double t) {
        if (f.HandActive()) ++handSamples[d];
        const int hand = f.HandActive() ? 3 : f.HandReady() ? 2 : 1;
        if (f.GetConfig().hand.enabled && handStatus[d] != hand) {
            handStatus[d] = hand;
            const std::string event = "{\"event\":\"hand_model\",\"device\":" + std::to_string(d) + ",\"status\":\"" +
                (hand == 3 ? "estimating" : hand == 2 ? "hold_or_ready" : "waiting") + "\"}";
            Write(out,rec::RecordType::Note,qpcOf(t),event.data(),uint32_t(event.size()));
        }
        const int status = f.BodyActive() ? 3 : f.BodyReady() ? 2 : 1;
        if (f.BodyActive()) ++bodySamples[d];
        if (!f.GetConfig().body.enabled || bodyStatus[d] == status) return;
        bodyStatus[d] = status;
        const std::string event = "{\"event\":\"body_model\",\"device\":" + std::to_string(d) + ",\"status\":\"" +
            (status == 3 ? "estimating" : status == 2 ? "hold_or_ready" : "learning") + "\",\"torsoLength\":" + std::to_string(f.TorsoLength()) + "}";
        Write(out,rec::RecordType::Note,qpcOf(t),event.data(),uint32_t(event.size()));
    };
    const std::string replayNote = "{\"event\":\"body_replay\",\"bodyEnabled\":" + std::string(cfg.body.enabled ? "true" : "false") +
        ",\"handEnabled\":" + std::string(cfg.hand.enabled ? "true" : "false") +
        ",\"waist\":" + std::to_string(waistIndex) + ",\"hmd\":" + std::to_string(headIndex) + "}";
    Write(out,rec::RecordType::Note,header.qpcStart,replayNote.data(),uint32_t(replayNote.size()));
    for (const int device : maskDevices) for (auto [begin,end] : masks) {
        const std::string maskNote = "{\"event\":\"pseudo_occlusion\",\"device\":" + std::to_string(device) +
            ",\"begin\":" + std::to_string(begin) + ",\"end\":" + std::to_string(end) + "}";
        Write(out,rec::RecordType::Note,header.qpcStart,maskNote.data(),uint32_t(maskNote.size()));
    }

    // Pass 2: replay in file order, ticking the emitter like the driver's thread would.
    reader.Rewind(header);
    const double tick = 1.0 / emitHz;
    double nextTick = 0;
    while (reader.Next(&r)) {
        const double t = seconds(r.h.qpc);
        for (; nextTick <= t; nextTick += tick)
            for (auto& [d, f] : filters) {
                references(d,*f);
                vr::DriverPose_t e;
                if (f->Emit(nextTick, &e)) {
                    WritePose(out, qpcOf(nextTick), d, rec::kEmittedHookSlot, e);
                    ++emittedCount[d];
                    bodyEvent(d,*f,nextTick);
                }
                flushTransitions(d, *f);
            }

        // A live Phase 1 recording already holds the driver's filter output and transitions:
        // drop them so the output only contains this replay's.
        if (r.h.type == uint16_t(rec::RecordType::FilteredPose) || r.h.type == uint16_t(rec::RecordType::FilterEvent)) continue;
        if (r.h.type == uint16_t(rec::RecordType::Note)) {
            const std::string note(r.payload.begin(),r.payload.end());
            if (note.find("\"event\":\"body_model\"") != std::string::npos || note.find("\"event\":\"hand_model\"") != std::string::npos) continue;
        }

        if (r.h.type == uint16_t(rec::RecordType::Pose) && r.payload.size() >= sizeof(rec::PosePayloadHeader)) {
            rec::PosePayloadHeader ph;
            std::memcpy(&ph, r.payload.data(), sizeof(ph));
            if (ph.poseStructSize >= sizeof(vr::DriverPose_t) &&
                r.payload.size() >= sizeof(ph) + sizeof(vr::DriverPose_t)) {
                vr::DriverPose_t in;
                std::memcpy(&in, r.payload.data() + sizeof(ph), sizeof(in));
                if (int(ph.deviceIndex) == headIndex) { head = in; headT = t; }
                const bool masked = std::find(maskDevices.begin(),maskDevices.end(),int(ph.deviceIndex)) != maskDevices.end() &&
                    std::any_of(masks.begin(),masks.end(),[&](const auto& m) { return t >= m.first && t < m.second; });
                if (masked) in.result = vr::TrackingResult_Calibrating_OutOfRange;
                const int side = handSides[ph.deviceIndex];
                if (side != 0) { const int hand = side < 0 ? 0 : 1; rawHands[hand] = in; rawHandTimes[hand] = t; }
                if (!filtered(ph.deviceIndex)) {
                    Write(out,rec::RecordType(r.h.type),r.h.qpc,r.payload.data(),uint32_t(r.payload.size()));
                    continue;
                }
                auto& f = filters[ph.deviceIndex];
                if (!f) {
                    auto c = cfg;
                    c.body.enabled = cfg.body.enabled && int(ph.deviceIndex) == waistIndex;
                    c.hand.enabled = cfg.hand.enabled && side != 0;
                    c.hand.side = side;
                    f = std::make_unique<TrackingFilter>(c);
                }
                references(ph.deviceIndex,*f);
                const auto c0 = std::chrono::steady_clock::now();
                const vr::DriverPose_t o = f->Process(in, t);
                processNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - c0).count();
                ++processCalls;
                WritePose(out, r.h.qpc, ph.deviceIndex, ph.hookSlot, o);
                flushTransitions(ph.deviceIndex, *f);
                bodyEvent(ph.deviceIndex,*f,t);
                continue;
            }
        }
        Write(out, rec::RecordType(r.h.type), r.h.qpc, r.payload.data(), uint32_t(r.payload.size()));
    }
    std::fclose(out);

    std::printf("%s -> %s\n", inPath.c_str(), outPath.c_str());
    std::printf("  Process(): %lld calls, average %.0f ns\n", processCalls, processCalls ? double(processNs) / processCalls : 0.0);
    for (auto& [d, f] : filters)
        std::printf("  device %2u: transitions %llu, emitted %llu, steps limited %llu, final %s, body samples %llu, hand samples %llu\n", d,
                    (unsigned long long)transitionsCount[d], (unsigned long long)emittedCount[d],
                    (unsigned long long)f->StepsLimited(), ToString(f->GetState()),(unsigned long long)bodySamples[d],(unsigned long long)handSamples[d]);
    return 0;
}
