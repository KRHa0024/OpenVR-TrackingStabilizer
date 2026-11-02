#include "VrSource.h"

#include <openvr.h>

#include <cmath>
#include <cstring>
#include <map>

namespace ots {

const char* TrackingResultName(int result) {
    switch (result) {
    case vr::TrackingResult_Uninitialized: return "未初期化";
    case vr::TrackingResult_Calibrating_InProgress: return "調整中";
    case vr::TrackingResult_Calibrating_OutOfRange: return "範囲外";
    case vr::TrackingResult_Running_OK: return "OK";
    case vr::TrackingResult_Running_OutOfRange: return "範囲外(動作中)";
    case vr::TrackingResult_Fallback_RotationOnly: return "回転のみ";
    default: return "?";
    }
}

const char* DeviceClassName(int deviceClass) {
    switch (deviceClass) {
    case vr::TrackedDeviceClass_HMD: return "HMD";
    case vr::TrackedDeviceClass_Controller: return "コントローラー";
    case vr::TrackedDeviceClass_GenericTracker: return "トラッカー";
    case vr::TrackedDeviceClass_TrackingReference: return "ベースステーション";
    default: return "その他";
    }
}

namespace {

std::string StringProp(vr::IVRSystem* sys, uint32_t i, vr::ETrackedDeviceProperty p) {
    char buf[vr::k_unMaxPropertyStringSize];
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    sys->GetStringTrackedDeviceProperty(i, p, buf, sizeof(buf), &err);
    return err == vr::TrackedProp_Success ? std::string(buf) : std::string();
}

} // namespace

void PollVr(AppState& state) {
    vr::IVRSystem* sys = vr::VRSystem();
    if (!sys) return;
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, vr::k_unMaxTrackedDeviceCount);
    // An invalid pose carries a zero matrix; keep showing such a device where it was last seen
    // instead of drawing it at the origin.
    std::map<uint32_t, DeviceInfo> previous;
    {
        std::lock_guard lock(state.m);
        for (const DeviceInfo& d : state.devices) previous[d.index] = d;
    }

    std::vector<DeviceInfo> devices;
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        const vr::ETrackedDeviceClass cls = sys->GetTrackedDeviceClass(i);
        if (cls == vr::TrackedDeviceClass_Invalid) continue;
        DeviceInfo d;
        d.index = i;
        d.deviceClass = cls;
        d.connected = sys->IsTrackedDeviceConnected(i);
        d.serial = StringProp(sys, i, vr::Prop_SerialNumber_String);
        d.model = StringProp(sys, i, vr::Prop_ModelNumber_String);
        vr::ETrackedPropertyError err = vr::TrackedProp_Success;
        d.role = sys->GetInt32TrackedDeviceProperty(i, vr::Prop_ControllerRoleHint_Int32, &err);
        d.hasBattery = sys->GetBoolTrackedDeviceProperty(i, vr::Prop_DeviceProvidesBatteryStatus_Bool, &err) && err == vr::TrackedProp_Success;
        if (d.hasBattery) {
            d.battery = sys->GetFloatTrackedDeviceProperty(i, vr::Prop_DeviceBatteryPercentage_Float, &err);
            d.charging = sys->GetBoolTrackedDeviceProperty(i, vr::Prop_DeviceIsCharging_Bool, &err);
        }
        if (cls == vr::TrackedDeviceClass_TrackingReference) {
            d.modeLabel = StringProp(sys, i, vr::Prop_ModeLabel_String);
            const float l = sys->GetFloatTrackedDeviceProperty(i, vr::Prop_FieldOfViewLeftDegrees_Float, &err);
            if (err == vr::TrackedProp_Success) {
                const float r = sys->GetFloatTrackedDeviceProperty(i, vr::Prop_FieldOfViewRightDegrees_Float, &err);
                const float t = sys->GetFloatTrackedDeviceProperty(i, vr::Prop_FieldOfViewTopDegrees_Float, &err);
                const float b = sys->GetFloatTrackedDeviceProperty(i, vr::Prop_FieldOfViewBottomDegrees_Float, &err);
                if (l + r > 1) d.fovH = l + r;
                if (t + b > 1) d.fovV = t + b;
            }
        }
        const vr::TrackedDevicePose_t& p = poses[i];
        d.poseValid = p.bPoseIsValid;
        d.trackingResult = p.eTrackingResult;
        const auto& m = p.mDeviceToAbsoluteTracking.m;
        d.position = {m[0][3], m[1][3], m[2][3]};
        d.forward = {-m[0][2], -m[1][2], -m[2][2]};
        d.right = {m[0][0], m[1][0], m[2][0]};
        d.up = {m[0][1], m[1][1], m[2][1]};
        if (!p.bPoseIsValid) {
            auto it = previous.find(i);
            if (it != previous.end()) {
                d.position = it->second.position;
                d.forward = it->second.forward, d.right = it->second.right, d.up = it->second.up;
            }
        }
        devices.push_back(std::move(d));
    }

    std::vector<Vec3> area;
    if (vr::IVRChaperone* ch = vr::VRChaperone()) {
        vr::HmdQuad_t rect;
        if (ch->GetPlayAreaRect(&rect))
            for (const auto& c : rect.vCorners) area.push_back({c.v[0], c.v[1], c.v[2]});
    }

    // Maps the driver's world space (the lane graph's trace) into the standing universe (the map).
    const vr::HmdMatrix34_t rawToStanding = sys->GetRawZeroPoseToStandingAbsoluteTrackingPose();

    std::lock_guard lock(state.m);
    state.vrConnected = true;
    state.devices = std::move(devices);
    state.playArea = std::move(area);
    std::memcpy(state.rawToStanding, rawToStanding.m, sizeof(state.rawToStanding));
}

} // namespace ots
