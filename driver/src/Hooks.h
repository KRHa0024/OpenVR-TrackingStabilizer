#pragma once
// Inline hooks (MinHook) on vrserver's implementation of the driver-facing interfaces.
// Because the hooked functions are vrserver code shared by every driver context, these
// observe calls made by all drivers (lighthouse included), not only ours.
#include <openvr_driver.h>

#include <string>

namespace ots {

class Recorder;
class FilterManager;

struct HookOptions {
    bool poses = true;
    bool driverLog = true;
};

struct HookStatus {
    // One slot per IVRServerDriverHost version we try; empty name = not available.
    struct Slot {
        std::string interfaceName;
        bool installed = false;
        bool sharedWithPrevious = false; // same function as an earlier slot, not hooked twice
    };
    static constexpr int kPoseSlots = 1;
    Slot pose[kPoseSlots];
    bool logInstalled = false;
};

bool InstallHooks(vr::IVRDriverContext* context, Recorder* recorder, const HookOptions& options);
// Routes hooked poses through `filter` (nullptr = pass everything through).
void SetPoseFilter(FilterManager* filter);
bool PoseHookInstalled();
// Calls vrserver's original TrackedDevicePoseUpdated directly (used by the filter's emitter).
void ForwardPose(void* host, uint32_t device, const vr::DriverPose_t* pose);
// Disables hooks and waits (bounded) for in-flight detours to return before the DLL unloads.
void RemoveHooks();
HookStatus GetHookStatus();

} // namespace ots
