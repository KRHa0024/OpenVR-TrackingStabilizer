#pragma once
// Client side of the driver's control pipe: one request line, one JSON response line.
#include "OtsConstants.h"

#include <windows.h>

#include <optional>
#include <string>

namespace ots {

// Returns nullopt when the driver is not reachable (SteamVR not running / driver not loaded).
inline std::optional<std::string> PipeRequest(const std::string& line, DWORD busyWaitMs = 2000) {
    HANDLE pipe = INVALID_HANDLE_VALUE;
    for (int attempt = 0; attempt < 5; ++attempt) {
        pipe = CreateFileW(kPipeName, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (pipe != INVALID_HANDLE_VALUE) break;
        const DWORD err = GetLastError();
        if (err == ERROR_PIPE_BUSY) {
            if (!WaitNamedPipeW(kPipeName, busyWaitMs)) break;
        } else if (err == ERROR_FILE_NOT_FOUND && attempt < 4) {
            Sleep(20); // server between instances, or driver not loaded (then all attempts fail)
        } else {
            break;
        }
    }
    if (pipe == INVALID_HANDLE_VALUE) return std::nullopt;

    const std::string request = line + "\n";
    DWORD written = 0;
    WriteFile(pipe, request.data(), DWORD(request.size()), &written, nullptr);

    std::string response;
    char buf[8192];
    while (response.find('\n') == std::string::npos) {
        DWORD read = 0;
        if (!ReadFile(pipe, buf, sizeof(buf), &read, nullptr) || read == 0) break;
        response.append(buf, read);
    }
    CloseHandle(pipe);
    if (const size_t nl = response.find('\n'); nl != std::string::npos) response.resize(nl);
    return response;
}

} // namespace ots
