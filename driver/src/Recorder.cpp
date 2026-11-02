#include "Recorder.h"

#include <windows.h>

#include <chrono>
#include <cstring>
#include <ctime>

namespace ots {

int64_t QpcNow() {
    LARGE_INTEGER v;
    QueryPerformanceCounter(&v);
    return v.QuadPart;
}

static int64_t QpcFrequency() {
    LARGE_INTEGER v;
    QueryPerformanceFrequency(&v);
    return v.QuadPart;
}

static std::string SanitizeLabel(std::string_view label) {
    std::string out;
    for (char c : label) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
        out.push_back(ok ? c : '_');
        if (out.size() >= 48) break;
    }
    return out;
}

// Paths go to the pipe as JSON, and path::string() throws on characters outside the system code page.
static std::string Utf8(const std::filesystem::path& p) {
    const std::u8string s = p.u8string();
    return std::string(s.begin(), s.end());
}

Recorder::~Recorder() { Stop(); }

bool Recorder::Start(const std::filesystem::path& dir, std::string_view label, std::string* outPathOrError) {
    std::lock_guard life(lifecycleMutex_);
    if (recording_.load()) {
        if (outPathOrError) *outPathOrError = "already recording";
        return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_s(&local, &now);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &local);
    std::string name = stamp;
    const std::string safe = SanitizeLabel(label);
    if (!safe.empty()) name += "_" + safe;
    const std::filesystem::path path = dir / (name + ".otsrec");

    FILE* f = _wfopen(path.c_str(), L"wb");
    if (!f) {
        if (outPathOrError) *outPathOrError = "cannot open " + Utf8(path);
        return false;
    }

    rec::FileHeader header{};
    std::memcpy(header.magic, rec::kMagic, sizeof(header.magic));
    header.version = rec::kVersion;
    header.headerSize = sizeof(header);
    header.qpcFrequency = QpcFrequency();
    header.qpcStart = QpcNow();
    header.unixTimeMsStart = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::strncpy(header.label, std::string(label).c_str(), sizeof(header.label) - 1);
    fwrite(&header, sizeof(header), 1, f);

    {
        std::lock_guard lock(mutex_);
        pending_.clear();
        newDevices_.clear();
        stopRequested_ = false;
        poses_ = logs_ = markers_ = dropped_ = 0;
    }
    seenDevices_.store(0);
    bytesWritten_.store(sizeof(header));
    file_ = f;
    path_ = Utf8(path);
    qpcStart_ = header.qpcStart;

    recording_.store(true, std::memory_order_release);
    writer_ = std::thread(&Recorder::WriterLoop, this);

    if (outPathOrError) *outPathOrError = path_;
    return true;
}

void Recorder::Stop() {
    std::lock_guard life(lifecycleMutex_);
    if (!recording_.exchange(false)) return;
    {
        std::lock_guard lock(mutex_);
        stopRequested_ = true;
    }
    cv_.notify_all();
    if (writer_.joinable()) writer_.join();
    fclose(file_);
    file_ = nullptr;
}

Recorder::Stats Recorder::GetStats() {
    Stats s;
    s.recording = recording_.load();
    {
        std::lock_guard life(lifecycleMutex_);
        s.path = path_;
    }
    {
        std::lock_guard lock(mutex_);
        s.poses = poses_;
        s.logs = logs_;
        s.markers = markers_;
        s.dropped = dropped_;
    }
    s.bytesWritten = bytesWritten_.load();
    if (s.recording) {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        s.seconds = double(QpcNow() - qpcStart_) / double(f.QuadPart);
    }
    return s;
}

void Recorder::Append(rec::RecordType type, int64_t qpc, const void* a, uint32_t aSize, const void* b, uint32_t bSize) {
    rec::RecordHeader h{};
    h.type = static_cast<uint16_t>(type);
    h.payloadSize = aSize + bSize;
    h.qpc = qpc;
    const size_t total = sizeof(h) + aSize + bSize;

    std::lock_guard lock(mutex_);
    if (!recording_.load(std::memory_order_relaxed)) return;
    if (pending_.size() + total > kMaxPendingBytes) {
        ++dropped_;
        return;
    }
    const size_t at = pending_.size();
    pending_.resize(at + total);
    uint8_t* p = pending_.data() + at;
    std::memcpy(p, &h, sizeof(h));
    if (aSize) std::memcpy(p + sizeof(h), a, aSize);
    if (bSize) std::memcpy(p + sizeof(h) + aSize, b, bSize);
    switch (type) {
    case rec::RecordType::Pose: ++poses_; break;
    case rec::RecordType::DriverLog: ++logs_; break;
    case rec::RecordType::Marker: ++markers_; break;
    default: break;
    }
}

void Recorder::PushPose(uint32_t deviceIndex, uint32_t hookSlot, const void* pose, uint32_t poseSize) {
    if (!recording_.load(std::memory_order_relaxed)) return;
    const int64_t qpc = QpcNow();
    rec::PosePayloadHeader ph{deviceIndex, poseSize, hookSlot};
    Append(rec::RecordType::Pose, qpc, &ph, sizeof(ph), pose, poseSize);

    if (deviceIndex < kMaxDevices) {
        const uint64_t bit = 1ull << deviceIndex;
        if (!(seenDevices_.fetch_or(bit, std::memory_order_relaxed) & bit)) {
            {
                std::lock_guard lock(mutex_);
                newDevices_.push_back(deviceIndex);
            }
            cv_.notify_one();
        }
    }
}

void Recorder::PushFilteredPose(uint32_t deviceIndex, uint32_t hookSlot, const void* pose, uint32_t poseSize) {
    if (!recording_.load(std::memory_order_relaxed)) return;
    rec::PosePayloadHeader ph{deviceIndex, poseSize, hookSlot};
    Append(rec::RecordType::FilteredPose, QpcNow(), &ph, sizeof(ph), pose, poseSize);
}

void Recorder::PushLog(uint64_t logObject, std::string_view text) {
    if (!recording_.load(std::memory_order_relaxed)) return;
    rec::LogPayloadHeader lh{logObject};
    Append(rec::RecordType::DriverLog, QpcNow(), &lh, sizeof(lh), text.data(), uint32_t(text.size()));
}

void Recorder::PushText(rec::RecordType type, std::string_view text) {
    if (!recording_.load(std::memory_order_relaxed)) return;
    Append(type, QpcNow(), text.data(), uint32_t(text.size()), nullptr, 0);
}

void Recorder::PushRawPoses(const void* poses, uint32_t count, uint32_t structSize) {
    if (!recording_.load(std::memory_order_relaxed)) return;
    rec::RawPosesPayloadHeader rh{count, structSize};
    Append(rec::RecordType::RawPoses, QpcNow(), &rh, sizeof(rh), poses, count * structSize);
}

void Recorder::PushImu(uint32_t deviceIndex, const void* samples, uint32_t count, uint32_t structSize) {
    if (!recording_.load(std::memory_order_relaxed) || count == 0) return;
    rec::ImuPayloadHeader h{deviceIndex, count, structSize};
    Append(rec::RecordType::ImuSamples, QpcNow(), &h, sizeof(h), samples, count * structSize);
}

void Recorder::Flush(std::vector<uint8_t>& buf) {
    if (buf.empty()) return;
    fwrite(buf.data(), 1, buf.size(), file_);
    bytesWritten_.fetch_add(buf.size());
    buf.clear();
}

void Recorder::WriterLoop() {
    using namespace std::chrono;
    std::vector<uint8_t> local;
    std::vector<uint32_t> devices;
    auto nextTick = steady_clock::now();

    for (;;) {
        bool stopping;
        {
            std::unique_lock lock(mutex_);
            cv_.wait_for(lock, milliseconds(10), [&] { return stopRequested_ || !newDevices_.empty(); });
            stopping = stopRequested_;
            local.swap(pending_);
            devices.swap(newDevices_);
        }
        Flush(local);

        // Callbacks run outside the lock; they push records that land in the next flush.
        // While stopping, recording_ is already false so their pushes are no-ops.
        for (uint32_t d : devices)
            if (onNewDevice) onNewDevice(d);
        devices.clear();
        if (!stopping && steady_clock::now() >= nextTick) {
            nextTick += seconds(1);
            if (onTick) onTick();
        }

        if (stopping) {
            std::lock_guard lock(mutex_);
            local.swap(pending_);
            break;
        }
    }
    Flush(local);
    fflush(file_);
}

} // namespace ots
