#include "DriverLog.h"

#include "LogRing.h"

#include <openvr_driver.h>

#include <cstdarg>
#include <cstdio>

namespace ots {

static thread_local bool t_insideOwnLog = false;

bool IsInsideOwnLog() { return t_insideOwnLog; }

void DriverLog(const char* fmt, ...) {
    vr::IVRDriverLog* log = vr::VRDriverLog();
    if (!log) return;

    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    GlobalLog().Add("ots", buf);
    t_insideOwnLog = true;
    log->Log(buf);
    t_insideOwnLog = false;
}

} // namespace ots
