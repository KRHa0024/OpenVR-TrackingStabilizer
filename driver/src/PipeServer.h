#pragma once
// Line-oriented control channel on a named pipe. One client at a time; each request line
// ("status", "start <label>", ...) gets exactly one response line (JSON).
#include <atomic>
#include <functional>
#include <string>
#include <string_view>
#include <thread>

namespace ots {

class PipeServer {
public:
    using Handler = std::function<std::string(std::string_view line)>;

    ~PipeServer();
    bool Start(const wchar_t* pipeName, Handler handler);
    void Stop();

private:
    void Loop();
    void ServeClient(void* pipe);

    std::wstring name_;
    Handler handler_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> exited_{false};
};

} // namespace ots
