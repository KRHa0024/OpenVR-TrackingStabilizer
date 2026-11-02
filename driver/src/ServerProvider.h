#pragma once
#include "FilterManager.h"
#include "PipeServer.h"
#include "Recorder.h"

#include <openvr_driver.h>

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

namespace ots {

// Registers no devices. Exists to get loaded into vrserver, install hooks and serve the pipe.
class ServerProvider final : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    void RunFrame() override {}
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    std::string HandleCommand(std::string_view line);
    std::string StartRecording(std::string_view label);
    std::string StatusJson();
    void WriteDeviceInfo(uint32_t deviceIndex);
    void WriteRawPoseSnapshot();
    static FilterManager::Settings ReadFilterSettings();

    Recorder recorder_;
    std::unique_ptr<FilterManager> filter_;
    PipeServer pipe_;
    std::filesystem::path recordDir_;
    bool hooksActive_ = false;
};

} // namespace ots
