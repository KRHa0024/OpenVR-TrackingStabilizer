#pragma once

namespace ots {

// SteamVR driver id: settings section is "driver_<kDriverName>", DLL is driver_<kDriverName>.dll.
inline constexpr const char* kDriverName = "trackingstabilizer";
inline constexpr const char* kSettingsSection = "driver_trackingstabilizer";

// Control channel between the driver (server) and tools / future overlay (clients).
inline constexpr const wchar_t* kPipeName = L"\\\\.\\pipe\\OpenVR-TrackingStabilizer";

} // namespace ots
