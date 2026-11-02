#pragma once
// A normal desktop window that shows the UI's render target (scaled to the window) and passes mouse
// input back in UI layout units. Closing it only hides it; the SteamVR overlay keeps running.
#include <d3d11.h>
#include <dxgi.h>

#include <functional>

namespace ots {

class DesktopWindow {
public:
    struct Callbacks {
        std::function<void(float x, float y)> mouseMove;   // UI layout units (uiWidth x uiHeight)
        std::function<void(int button, bool down)> mouseButton;
        std::function<void(float linesDown)> mouseWheel;
        std::function<void()> mouseLeave;
        std::function<void()> closed;                      // the user closed the window
        std::function<void()> tick;                        // ~30 Hz while Windows' modal move / size loop runs
        std::function<void()> showRequested;               // a second launch asked for the window
    };

    bool Create(ID3D11Device* device, int uiWidth, int uiHeight, Callbacks callbacks);
    // Asks the running instance's window to show itself; false when there is none.
    static bool RequestShow();
    void Destroy();
    void SetVisible(bool visible);
    bool Visible() const;   // shown and not minimised
    bool Minimized() const; // shown but minimised
    // Handles queued window messages; returns false after WM_QUIT.
    bool PumpMessages();
    void Present(ID3D11Texture2D* uiTexture);           // any size; the back buffer follows it
    // Render scale that would fill the window pixel for pixel (0.5..3; 0 when hidden or minimised).
    float PixelScale() const;

private:
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
    LRESULT Handle(UINT msg, WPARAM wp, LPARAM lp);
    void ToUi(LPARAM lp, float* x, float* y) const;
    SIZE FrameSize() const; // window size minus client size at the window's DPI
    void FitAspect(int* clientW, int* clientH) const; // shrink one side to the UI's aspect ratio

    HWND hwnd_ = nullptr;
    ID3D11Device* device_ = nullptr;
    ID3D11DeviceContext* context_ = nullptr;
    IDXGISwapChain* swapChain_ = nullptr;
    int uiW_ = 0, uiH_ = 0;
    bool visible_ = false;
    bool tracking_ = false; // TrackMouseEvent armed
    Callbacks cb_;
};

} // namespace ots
