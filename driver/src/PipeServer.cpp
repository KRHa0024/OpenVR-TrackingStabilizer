#include "PipeServer.h"

#include "DriverLog.h"

#include <windows.h>

#include <exception>

namespace ots {

PipeServer::~PipeServer() { Stop(); }

bool PipeServer::Start(const wchar_t* pipeName, Handler handler) {
    if (thread_.joinable()) return false;
    name_ = pipeName;
    handler_ = std::move(handler);
    stop_ = false;
    exited_ = false;
    thread_ = std::thread(&PipeServer::Loop, this);
    return true;
}

void PipeServer::Stop() {
    if (!thread_.joinable()) return;
    stop_ = true;
    // Unblock ConnectNamedPipe / ReadFile on the server thread. Retried because the thread may
    // be between iterations (no pipe instance yet) when we first try.
    while (!exited_) {
        CancelSynchronousIo(thread_.native_handle());
        HANDLE poke = CreateFileW(name_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (poke != INVALID_HANDLE_VALUE) CloseHandle(poke);
        Sleep(10);
    }
    thread_.join();
}

void PipeServer::Loop() {
    auto create = [this] {
        return CreateNamedPipeW(name_.c_str(), PIPE_ACCESS_DUPLEX,
                                PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
                                2, 64 * 1024, 64 * 1024, 0, nullptr);
    };
    HANDLE pipe = create();
    while (!stop_) {
        if (pipe == INVALID_HANDLE_VALUE) {
            DriverLog("pipe: CreateNamedPipe failed (%lu)", GetLastError());
            Sleep(1000);
            pipe = create();
            continue;
        }
        const BOOL connected = ConnectNamedPipe(pipe, nullptr) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
        if (stop_) break;
        // Open the next instance before serving, so a client arriving while this one is being
        // served (or right after it leaves) always finds the pipe. Clients are still served one at a time.
        HANDLE next = create();
        if (connected) ServeClient(pipe);
        DisconnectNamedPipe(pipe);
        CloseHandle(pipe);
        pipe = next;
    }
    if (pipe != INVALID_HANDLE_VALUE) {
        DisconnectNamedPipe(pipe);
        CloseHandle(pipe);
    }
    exited_ = true;
}

void PipeServer::ServeClient(void* pipeHandle) {
    HANDLE pipe = static_cast<HANDLE>(pipeHandle);
    std::string buffer;
    char chunk[4096];
    while (!stop_) {
        DWORD read = 0;
        if (!ReadFile(pipe, chunk, sizeof(chunk), &read, nullptr) || read == 0) return;
        buffer.append(chunk, read);

        size_t nl;
        while ((nl = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, nl);
            buffer.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            // An exception leaving this thread would take vrserver down.
            std::string response;
            try {
                response = handler_(line);
            } catch (const std::exception& e) {
                DriverLog("pipe: command failed: %s", e.what());
                response = "{\"ok\":false,\"error\":\"internal error\"}";
            }
            response.push_back('\n');
            DWORD written = 0;
            if (!WriteFile(pipe, response.data(), DWORD(response.size()), &written, nullptr)) return;
        }
        if (buffer.size() > 64 * 1024) return; // garbage client
    }
}

} // namespace ots
