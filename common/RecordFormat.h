#pragma once
// On-disk format of a recording session (*.otsrec). Little-endian, no padding.
// Parsed by tools/analysis/otsrec.py — keep both in sync.
//
//   FileHeader
//   { RecordHeader, payload[payloadSize] } ...
#include <cstdint>

namespace ots::rec {

inline constexpr char kMagic[8] = {'O', 'T', 'S', 'R', 'E', 'C', '0', '1'};
inline constexpr uint32_t kVersion = 1;

#pragma pack(push, 1)
struct FileHeader {
    char magic[8];
    uint32_t version;
    uint32_t headerSize;      // sizeof(FileHeader)
    int64_t qpcFrequency;     // QueryPerformanceFrequency
    int64_t qpcStart;         // QPC at session start
    int64_t unixTimeMsStart;  // wall clock at session start
    char label[64];           // NUL-terminated UTF-8
};

struct RecordHeader {
    uint16_t type;  // RecordType
    uint16_t flags; // reserved
    uint32_t payloadSize;
    int64_t qpc;    // QPC when the record was captured
};

// Payload of RecordType::Pose, followed by poseStructSize bytes of vr::DriverPose_t
// exactly as the driver passed it to TrackedDevicePoseUpdated.
struct PosePayloadHeader {
    uint32_t deviceIndex;
    uint32_t poseStructSize;
    uint32_t hookSlot;        // which IVRServerDriverHost version delivered it
};

// Payload of RecordType::DriverLog, followed by the message bytes (no NUL).
struct LogPayloadHeader {
    uint64_t logObject;       // IVRDriverLog instance pointer; identifies the calling driver
};

// Payload of RecordType::RawPoses, followed by count * structSize bytes of vr::TrackedDevicePose_t.
struct RawPosesPayloadHeader {
    uint32_t count;
    uint32_t structSize;
};
#pragma pack(pop)

enum class RecordType : uint16_t {
    Pose = 1,       // PosePayloadHeader + DriverPose_t
    DriverLog = 2,  // LogPayloadHeader + text
    Marker = 3,     // UTF-8 text from a client ("mark ...")
    DeviceInfo = 4, // UTF-8 JSON object describing one device index
    RawPoses = 5,   // RawPosesPayloadHeader + TrackedDevicePose_t[] (periodic snapshot)
    Note = 6,       // UTF-8 text from the driver itself (start/stop, stats)
    FilterEvent = 7, // UTF-8 JSON: {"device":N,"from":"TRACKING","to":"HOLD","reason":"..."}
    FilteredPose = 8, // PosePayloadHeader + DriverPose_t actually forwarded to vrserver by the filter
                      // (hookSlot kEmittedHookSlot = synthesized by the emitter)
    ImuSamples = 9,   // ImuPayloadHeader + vr::ImuSample_t[] read from lighthouse's IMU IOBuffer
};

#pragma pack(push, 1)
struct ImuPayloadHeader {
    uint32_t deviceIndex;
    uint32_t count;
    uint32_t structSize;
};
#pragma pack(pop)

// PosePayloadHeader::hookSlot value for poses synthesized by the filter's emitter.
inline constexpr uint32_t kEmittedHookSlot = 100;

static_assert(sizeof(FileHeader) == 104);
static_assert(sizeof(RecordHeader) == 16);

} // namespace ots::rec
