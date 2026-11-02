// Writes a synthetic session with known contents; tools/analysis/test_otsrec.py checks the parse.
// Usage: recorder_selftest <output-dir>
#include "Recorder.h"

#include <openvr_driver.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: recorder_selftest <output-dir>\n");
        return 2;
    }

    constexpr int kDevices = 3;
    constexpr int kPosesPerDevice = 2000;

    ots::Recorder recorder;
    recorder.onNewDevice = [&](uint32_t d) {
        recorder.PushText(ots::rec::RecordType::DeviceInfo,
                          "{\"index\":" + std::to_string(d) + ",\"class\":3,\"serial\":\"TEST-" + std::to_string(d) + "\"}");
    };

    std::string path;
    if (!recorder.Start(argv[1], "selftest", &path)) {
        std::fprintf(stderr, "start failed: %s\n", path.c_str());
        return 1;
    }
    recorder.PushText(ots::rec::RecordType::Marker, "begin");

    // One producer thread per device, like lighthouse delivering from several threads.
    std::vector<std::thread> producers;
    for (int d = 0; d < kDevices; ++d) {
        producers.emplace_back([&recorder, d] {
            for (int i = 0; i < kPosesPerDevice; ++i) {
                vr::DriverPose_t pose{};
                pose.qRotation = {1, 0, 0, 0};
                pose.qWorldFromDriverRotation = {1, 0, 0, 0};
                pose.qDriverFromHeadRotation = {1, 0, 0, 0};
                pose.vecPosition[0] = d;
                pose.vecPosition[1] = i * 0.001;
                pose.vecVelocity[2] = -1.5;
                pose.result = (i % 500 < 450) ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
                pose.poseIsValid = true;
                pose.deviceIsConnected = true;
                recorder.PushPose(uint32_t(d), 0, &pose, sizeof(pose));
                if (i % 100 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    recorder.PushLog(0x1234, "LHR-7E570000 C: Resetting tracking: no optical samples for 2002ms");
    for (auto& t : producers) t.join();

    vr::TrackedDevicePose_t raw[4] = {};
    raw[1].bPoseIsValid = true;
    raw[1].mDeviceToAbsoluteTracking.m[0][3] = 1.25f;
    recorder.PushRawPoses(raw, 4, sizeof(vr::TrackedDevicePose_t));

    recorder.PushText(ots::rec::RecordType::Marker, "end");
    const auto stats = recorder.GetStats();
    recorder.Stop();

    // A folder name outside any code page: the path comes back as UTF-8 instead of throwing.
    {
        const std::u8string folder = u8"selftest-\U0001F4C1";
        const std::filesystem::path dir = std::filesystem::path(argv[1]) / folder;
        ots::Recorder other;
        std::string unicodePath;
        const bool started = other.Start(dir, "unicode", &unicodePath);
        other.Stop();
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
        if (!started || unicodePath.find(std::string(folder.begin(), folder.end())) == std::string::npos) {
            std::fprintf(stderr, "unicode folder failed\n");
            return 1;
        }
    }

    for (char& c : path)
        if (c == '\\') c = '/';
    std::printf("{\"path\":\"%s\",\"poses\":%llu,\"dropped\":%llu}\n", path.c_str(),
                (unsigned long long)stats.poses, (unsigned long long)stats.dropped);
    return stats.poses == kDevices * kPosesPerDevice && stats.dropped == 0 ? 0 : 1;
}
