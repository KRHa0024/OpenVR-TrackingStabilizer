#pragma once

namespace ots {

// printf-style logging to vrserver.txt (prefixed "trackingstabilizer:" by SteamVR).
// Safe to call while the IVRDriverLog hook is installed: our own messages are not recorded.
void DriverLog(const char* fmt, ...);

// True while the current thread is inside DriverLog (used by the log hook to skip our own lines).
bool IsInsideOwnLog();

} // namespace ots
