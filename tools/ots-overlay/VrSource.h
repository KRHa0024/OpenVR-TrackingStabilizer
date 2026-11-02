#pragma once
// Reads devices, poses, battery and the play area from the OpenVR client API.
#include "State.h"

namespace ots {

void PollVr(AppState& state);
const char* TrackingResultName(int result);
const char* DeviceClassName(int deviceClass);

} // namespace ots
