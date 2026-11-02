#pragma once
// Session recorder. Producers (hook detours on arbitrary SteamVR threads) only append
// bytes to an in-memory buffer under a short lock; a writer thread flushes to disk.
// Has no OpenVR dependency so it can be exercised by tests/recorder_selftest.
#include "RecordFormat.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ots {

int64_t QpcNow();

class Recorder {
public:
    struct Stats {
        bool recording = false;
        std::string path;
        uint64_t poses = 0;
        uint64_t logs = 0;
        uint64_t markers = 0;
        uint64_t dropped = 0;
        uint64_t bytesWritten = 0;
        double seconds = 0.0;
    };

    // Called on the writer thread (never on a producer thread).
    std::function<void(uint32_t deviceIndex)> onNewDevice;
    std::function<void()> onTick; // ~1 Hz while recording

    ~Recorder();

    // Creates <dir>/<timestamp>_<label>.otsrec. Returns false if already recording or on I/O error.
    bool Start(const std::filesystem::path& dir, std::string_view label, std::string* outPathOrError);
    void Stop();
    bool IsRecording() const { return recording_.load(std::memory_order_acquire); }
    Stats GetStats();

    // Hot path. Cheap no-op when not recording.
    void PushPose(uint32_t deviceIndex, uint32_t hookSlot, const void* pose, uint32_t poseSize);
    void PushFilteredPose(uint32_t deviceIndex, uint32_t hookSlot, const void* pose, uint32_t poseSize);
    void PushLog(uint64_t logObject, std::string_view text);
    void PushText(rec::RecordType type, std::string_view text);
    void PushRawPoses(const void* poses, uint32_t count, uint32_t structSize);
    void PushImu(uint32_t deviceIndex, const void* samples, uint32_t count, uint32_t structSize);

private:
    static constexpr size_t kMaxPendingBytes = 64ull << 20; // drop rather than grow unbounded
    static constexpr uint32_t kMaxDevices = 64;

    void Append(rec::RecordType type, int64_t qpc, const void* a, uint32_t aSize, const void* b, uint32_t bSize);
    void WriterLoop();
    void Flush(std::vector<uint8_t>& buf);

    std::atomic<bool> recording_{false};
    std::atomic<uint64_t> seenDevices_{0}; // bit per device index, reset each session

    std::mutex mutex_; // guards pending_, newDevices_, counters
    std::condition_variable cv_;
    std::vector<uint8_t> pending_;
    std::vector<uint32_t> newDevices_;
    bool stopRequested_ = false;
    uint64_t poses_ = 0, logs_ = 0, markers_ = 0, dropped_ = 0;

    std::mutex lifecycleMutex_; // serializes Start/Stop
    std::thread writer_;
    FILE* file_ = nullptr;
    std::string path_;
    std::atomic<uint64_t> bytesWritten_{0};
    int64_t qpcStart_ = 0;
};

} // namespace ots
