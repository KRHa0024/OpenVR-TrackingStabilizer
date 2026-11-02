#pragma once
// Polls the driver's control pipe on a background thread and runs queued commands, so the render
// loop never blocks on the pipe.
#include "State.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace ots {

class DriverClient {
public:
    explicit DriverClient(AppState& state) : state_(state) {}
    ~DriverClient() { Stop(); }

    void Start();
    void Stop();
    // Fire and forget; failures are added to the UI log. A queued `set <key>` replaces an older
    // queued one for the same key, so dragging a slider does not pile up commands.
    void Send(const std::string& command);
    // While the UI is visible the driver is polled every 100 ms and the lane graph's trace is fetched.
    void SetActive(bool active) { active_ = active; }
    // The body model's joints are fetched with every poll while the UI is visible, and ~30 times a second
    // while `fast` (the skeleton is shown in VR).
    void SetModelFast(bool fast) { modelFast_ = fast; }

private:
    void Loop();
    void Poll(bool withTrace);
    void PollModel();

    AppState& state_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> active_{false};
    std::atomic<bool> modelFast_{false};
    std::mutex queueMutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    double traceSince_ = 0; // driver clock of the newest trace sample we have
};

} // namespace ots
