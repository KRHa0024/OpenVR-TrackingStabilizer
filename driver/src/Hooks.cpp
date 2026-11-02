#include "Hooks.h"

#include "DriverLog.h"
#include "FilterManager.h"
#include "LogRing.h"
#include "Recorder.h"

#include <MinHook.h>
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

namespace ots {
namespace {

// x64 MSVC: a virtual member call passes `this` in rcx, so a free function taking the
// object pointer first has the same ABI. The pose is taken by pointer (same ABI as the
// reference) so a bogus null can be rejected without undefined behaviour.
using PoseUpdatedFn = void (*)(void* self, uint32_t unWhichDevice, const vr::DriverPose_t* newPose, uint32_t unPoseStructSize);
using LogFn = void (*)(void* self, const char* pchLogMessage);

// vtable indices for IVRServerDriverHost_006 and IVRDriverLog_001 (openvr_driver.h).
constexpr int kVtblTrackedDevicePoseUpdated = 1;
constexpr int kVtblLog = 0;

// Only _006: lighthouse delivers every pose through it (Phase 0, E1-E6). Do NOT add _005 back:
// in vrserver its vtable slot 1 is a different function (BeyondSteamVR calls it with a null
// third argument), and treating it as TrackedDevicePoseUpdated crashed vrserver on
// 2026-10-24 16:58 (docs/phase0.md).
constexpr const char* kPoseInterfaces[HookStatus::kPoseSlots] = {"IVRServerDriverHost_006"};

std::atomic<Recorder*> g_recorder{nullptr};
std::atomic<FilterManager*> g_filter{nullptr};
std::atomic<int> g_inFlight{0};
PoseUpdatedFn g_origPose[HookStatus::kPoseSlots] = {};
void* g_poseTargets[HookStatus::kPoseSlots] = {};
LogFn g_origLog = nullptr;
void* g_logTarget = nullptr;
HookStatus g_status;
bool g_mhInitialized = false;

struct InFlightGuard {
    InFlightGuard() { g_inFlight.fetch_add(1, std::memory_order_acq_rel); }
    ~InFlightGuard() { g_inFlight.fetch_sub(1, std::memory_order_acq_rel); }
};

template <int Slot>
void PoseDetour(void* self, uint32_t unWhichDevice, const vr::DriverPose_t* newPose, uint32_t unPoseStructSize) {
    InFlightGuard guard;
    // Only touch what looks like a real pose; anything else is forwarded untouched.
    if (newPose && unPoseStructSize >= sizeof(vr::DriverPose_t) && unWhichDevice < vr::k_unMaxTrackedDeviceCount) {
        if (Recorder* r = g_recorder.load(std::memory_order_acquire))
            r->PushPose(unWhichDevice, Slot, newPose, sizeof(vr::DriverPose_t));
        if (FilterManager* f = g_filter.load(std::memory_order_acquire); f && unPoseStructSize == sizeof(vr::DriverPose_t)) {
            vr::DriverPose_t out;
            if (f->Process(self, unWhichDevice, *newPose, &out)) {
                g_origPose[Slot](self, unWhichDevice, &out, unPoseStructSize);
                return;
            }
        }
    }
    g_origPose[Slot](self, unWhichDevice, newPose, unPoseStructSize);
}

constexpr PoseUpdatedFn kPoseDetours[HookStatus::kPoseSlots] = {&PoseDetour<0>};

void LogDetour(void* self, const char* pchLogMessage) {
    InFlightGuard guard;
    if (pchLogMessage && !IsInsideOwnLog()) {
        if (Recorder* r = g_recorder.load(std::memory_order_acquire))
            r->PushLog(reinterpret_cast<uint64_t>(self), pchLogMessage);
        GlobalLog().Add("driver", pchLogMessage);
    }
    g_origLog(self, pchLogMessage);
}

void* VtableEntry(void* object, int index) {
    void** vtbl = *reinterpret_cast<void***>(object);
    return vtbl[index];
}

bool HookFunction(void* target, void* detour, void** original, const char* what) {
    MH_STATUS st = MH_CreateHook(target, detour, original);
    if (st != MH_OK) {
        DriverLog("hook %s: MH_CreateHook failed: %s", what, MH_StatusToString(st));
        return false;
    }
    st = MH_EnableHook(target);
    if (st != MH_OK) {
        DriverLog("hook %s: MH_EnableHook failed: %s", what, MH_StatusToString(st));
        MH_RemoveHook(target);
        return false;
    }
    DriverLog("hook %s: installed at %p", what, target);
    return true;
}

} // namespace

bool InstallHooks(vr::IVRDriverContext* context, Recorder* recorder, const HookOptions& options) {
    g_status = HookStatus{};
    MH_STATUS st = MH_Initialize();
    if (st != MH_OK && st != MH_ERROR_ALREADY_INITIALIZED) {
        DriverLog("MH_Initialize failed: %s", MH_StatusToString(st));
        return false;
    }
    g_mhInitialized = true;
    g_recorder.store(recorder, std::memory_order_release);

    bool any = false;
    if (options.poses) {
        for (int slot = 0; slot < HookStatus::kPoseSlots; ++slot) {
            vr::EVRInitError err = vr::VRInitError_None;
            void* host = context->GetGenericInterface(kPoseInterfaces[slot], &err);
            if (!host || err != vr::VRInitError_None) {
                DriverLog("hook: %s not available (error %d)", kPoseInterfaces[slot], int(err));
                continue;
            }
            g_status.pose[slot].interfaceName = kPoseInterfaces[slot];
            void* target = VtableEntry(host, kVtblTrackedDevicePoseUpdated);
            const bool shared = std::find(std::begin(g_poseTargets), std::end(g_poseTargets), target) != std::end(g_poseTargets);
            if (shared) {
                g_status.pose[slot].sharedWithPrevious = true;
                continue;
            }
            if (HookFunction(target, reinterpret_cast<void*>(kPoseDetours[slot]),
                             reinterpret_cast<void**>(&g_origPose[slot]), kPoseInterfaces[slot])) {
                g_poseTargets[slot] = target;
                g_status.pose[slot].installed = true;
                any = true;
            }
        }
    }

    if (options.driverLog) {
        vr::EVRInitError err = vr::VRInitError_None;
        void* log = context->GetGenericInterface(vr::IVRDriverLog_Version, &err);
        if (log && err == vr::VRInitError_None) {
            void* target = VtableEntry(log, kVtblLog);
            if (HookFunction(target, reinterpret_cast<void*>(&LogDetour), reinterpret_cast<void**>(&g_origLog), vr::IVRDriverLog_Version)) {
                g_logTarget = target;
                g_status.logInstalled = true;
                any = true;
            }
        }
    }
    return any;
}

void SetPoseFilter(FilterManager* filter) { g_filter.store(filter, std::memory_order_release); }

bool PoseHookInstalled() { return g_origPose[0] != nullptr; }

void ForwardPose(void* host, uint32_t device, const vr::DriverPose_t* pose) {
    if (PoseUpdatedFn orig = g_origPose[0]) {
        InFlightGuard guard;
        orig(host, device, pose, sizeof(vr::DriverPose_t));
    }
}

void RemoveHooks() {
    if (!g_mhInitialized) return;
    MH_DisableHook(MH_ALL_HOOKS);
    g_recorder.store(nullptr, std::memory_order_release);
    g_filter.store(nullptr, std::memory_order_release);

    // A thread may still be executing a detour body; give it time to leave our code.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (g_inFlight.load(std::memory_order_acquire) > 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    MH_Uninitialize();
    g_mhInitialized = false;
    std::fill(std::begin(g_poseTargets), std::end(g_poseTargets), nullptr);
    std::fill(std::begin(g_origPose), std::end(g_origPose), nullptr);
    g_logTarget = nullptr;
    g_status = HookStatus{};
}

HookStatus GetHookStatus() { return g_status; }

} // namespace ots
