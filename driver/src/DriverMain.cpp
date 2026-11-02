#include "ServerProvider.h"

#include <openvr_driver.h>

#include <cstring>

#define OTS_DLL_EXPORT extern "C" __declspec(dllexport)

static ots::ServerProvider g_serverProvider;

OTS_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (std::strcmp(pInterfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) return &g_serverProvider;
    if (pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}

static_assert(sizeof(vr::DriverPose_t) == 280, "tools/analysis/otsrec.py assumes this layout");
static_assert(sizeof(vr::TrackedDevicePose_t) == 80, "tools/analysis/otsrec.py assumes this layout");
