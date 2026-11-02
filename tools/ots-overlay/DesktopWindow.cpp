#include "DesktopWindow.h"

#include <windowsx.h>

#include <algorithm>

namespace ots {

namespace {
constexpr wchar_t kClass[] = L"OtsTrackingStabilizerWindow";
constexpr UINT kShowRequest = WM_APP + 1;
}

bool DesktopWindow::RequestShow() {
    HWND hwnd = FindWindowW(kClass, nullptr);
    if (!hwnd) return false;
    DWORD process = 0;
    GetWindowThreadProcessId(hwnd, &process);
    AllowSetForegroundWindow(process); // only this process, just launched by the user, may hand that on
    return PostMessageW(hwnd, kShowRequest, 0, 0) != 0;
}

bool DesktopWindow::Create(ID3D11Device* device, int uiWidth, int uiHeight, Callbacks callbacks) {
    device_ = device;
    device_->GetImmediateContext(&context_);
    uiW_ = uiWidth;
    uiH_ = uiHeight;
    cb_ = std::move(callbacks);

    HINSTANCE inst = GetModuleHandleW(nullptr);
    WNDCLASSEXW wc{sizeof(wc)};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = &DesktopWindow::WndProc;
    wc.hInstance = inst;
    wc.hIcon = LoadIconW(inst, MAKEINTRESOURCEW(1));
    wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
    wc.lpszClassName = kClass;
    RegisterClassExW(&wc);

    // 1 dp per logical pixel (the layout is sized to be comfortable to read at that size), smaller
    // only when that does not fit in 90% of the work area. Smaller would mean smaller text.
    RECT work{};
    SystemParametersInfoW(SPI_GETWORKAREA, 0, &work, 0);
    const int availW = work.right - work.left, availH = work.bottom - work.top;
    const float scale = std::min({GetDpiForSystem() / 96.0f, availW * 0.9f / uiW_, availH * 0.9f / uiH_});
    RECT r{0, 0, LONG(uiW_ * scale), LONG(uiH_ * scale)};
    AdjustWindowRectExForDpi(&r, WS_OVERLAPPEDWINDOW, FALSE, 0, GetDpiForSystem());
    const int w = r.right - r.left, h = r.bottom - r.top;
    hwnd_ = CreateWindowExW(0, kClass, L"OpenVR Tracking Stabilizer", WS_OVERLAPPEDWINDOW, work.left + (availW - w) / 2,
                            work.top + (availH - h) / 2, w, h, nullptr, nullptr, inst, this);
    if (!hwnd_) return false;

    IDXGIDevice* dxgiDevice = nullptr;
    IDXGIAdapter* adapter = nullptr;
    IDXGIFactory* factory = nullptr;
    bool ok = SUCCEEDED(device_->QueryInterface(IID_PPV_ARGS(&dxgiDevice))) && SUCCEEDED(dxgiDevice->GetAdapter(&adapter)) &&
              SUCCEEDED(adapter->GetParent(IID_PPV_ARGS(&factory)));
    if (ok) {
        // The back buffer has the render target's size (Present resizes it); DXGI stretches it to the window.
        DXGI_SWAP_CHAIN_DESC sd{};
        sd.BufferCount = 1;
        sd.BufferDesc.Width = UINT(uiW_);
        sd.BufferDesc.Height = UINT(uiH_);
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.OutputWindow = hwnd_;
        sd.SampleDesc.Count = 1;
        sd.Windowed = TRUE;
        sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
        ok = SUCCEEDED(factory->CreateSwapChain(device_, &sd, &swapChain_));
        if (ok) factory->MakeWindowAssociation(hwnd_, DXGI_MWA_NO_ALT_ENTER);
    }
    if (factory) factory->Release();
    if (adapter) adapter->Release();
    if (dxgiDevice) dxgiDevice->Release();
    return ok;
}

void DesktopWindow::Destroy() {
    if (swapChain_) swapChain_->Release(), swapChain_ = nullptr;
    if (context_) context_->Release(), context_ = nullptr;
    if (hwnd_) DestroyWindow(hwnd_), hwnd_ = nullptr;
}

bool DesktopWindow::Visible() const { return hwnd_ && visible_ && !IsIconic(hwnd_); }
bool DesktopWindow::Minimized() const { return hwnd_ && visible_ && IsIconic(hwnd_); }

void DesktopWindow::SetVisible(bool visible) {
    if (!hwnd_ || visible == visible_) return;
    visible_ = visible;
    ShowWindow(hwnd_, visible ? SW_SHOW : SW_HIDE);
}

bool DesktopWindow::PumpMessages() {
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) return false;
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return true;
}

void DesktopWindow::Present(ID3D11Texture2D* uiTexture) {
    if (!swapChain_ || !visible_ || IsIconic(hwnd_)) return;
    D3D11_TEXTURE2D_DESC td;
    uiTexture->GetDesc(&td);
    DXGI_SWAP_CHAIN_DESC sd;
    swapChain_->GetDesc(&sd);
    if ((sd.BufferDesc.Width != td.Width || sd.BufferDesc.Height != td.Height) &&
        FAILED(swapChain_->ResizeBuffers(1, td.Width, td.Height, DXGI_FORMAT_R8G8B8A8_UNORM, 0)))
        return; // CopyResource needs matching sizes; try again next frame
    ID3D11Texture2D* back = nullptr;
    if (SUCCEEDED(swapChain_->GetBuffer(0, IID_PPV_ARGS(&back)))) {
        context_->CopyResource(back, uiTexture);
        back->Release();
    }
    swapChain_->Present(0, 0);
}

float DesktopWindow::PixelScale() const {
    if (!Visible()) return 0;
    RECT rc;
    GetClientRect(hwnd_, &rc);
    return std::clamp(float(rc.right) / uiW_, 0.5f, 3.0f);
}

SIZE DesktopWindow::FrameSize() const {
    RECT frame{0, 0, 0, 0};
    AdjustWindowRectExForDpi(&frame, WS_OVERLAPPEDWINDOW, FALSE, 0, GetDpiForWindow(hwnd_));
    return {frame.right - frame.left, frame.bottom - frame.top};
}

void DesktopWindow::FitAspect(int* w, int* h) const {
    if (*w * uiH_ > *h * uiW_) *w = *h * uiW_ / uiH_;
    else *h = *w * uiH_ / uiW_;
}

void DesktopWindow::ToUi(LPARAM lp, float* x, float* y) const {
    RECT rc;
    GetClientRect(hwnd_, &rc);
    const float cw = float(std::max<LONG>(1, rc.right)), ch = float(std::max<LONG>(1, rc.bottom));
    *x = GET_X_LPARAM(lp) * uiW_ / cw;
    *y = GET_Y_LPARAM(lp) * uiH_ / ch;
}

LRESULT CALLBACK DesktopWindow::WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) SetWindowLongPtrW(hwnd, GWLP_USERDATA, LONG_PTR(reinterpret_cast<CREATESTRUCTW*>(lp)->lpCreateParams));
    auto* self = reinterpret_cast<DesktopWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    return self && self->hwnd_ == hwnd ? self->Handle(msg, wp, lp) : DefWindowProcW(hwnd, msg, wp, lp);
}

LRESULT DesktopWindow::Handle(UINT msg, WPARAM wp, LPARAM lp) {
    float x, y;
    switch (msg) {
    case WM_MOUSEMOVE:
        if (!tracking_) {
            TRACKMOUSEEVENT t{sizeof(t), TME_LEAVE, hwnd_, 0};
            tracking_ = TrackMouseEvent(&t);
        }
        ToUi(lp, &x, &y);
        cb_.mouseMove(x, y);
        return 0;
    case WM_MOUSELEAVE:
        tracking_ = false;
        cb_.mouseLeave();
        return 0;
    case WM_LBUTTONDOWN:
    case WM_RBUTTONDOWN:
    case WM_MBUTTONDOWN:
        SetCapture(hwnd_);
        ToUi(lp, &x, &y);
        cb_.mouseMove(x, y);
        cb_.mouseButton(msg == WM_LBUTTONDOWN ? 0 : msg == WM_RBUTTONDOWN ? 1 : 2, true);
        return 0;
    case WM_LBUTTONUP:
    case WM_RBUTTONUP:
    case WM_MBUTTONUP:
        ReleaseCapture();
        ToUi(lp, &x, &y);
        cb_.mouseMove(x, y);
        cb_.mouseButton(msg == WM_LBUTTONUP ? 0 : msg == WM_RBUTTONUP ? 1 : 2, false);
        return 0;
    case WM_MOUSEWHEEL:
        cb_.mouseWheel(-float(GET_WHEEL_DELTA_WPARAM(wp)) / WHEEL_DELTA);
        return 0;
    // The texture is stretched to the client area, so the window keeps the UI's aspect ratio: while
    // dragging an edge (the other dimension follows), when maximised, and when snapped.
    case WM_SIZING: {
        RECT* r = reinterpret_cast<RECT*>(lp);
        const SIZE f = FrameSize();
        if (wp == WMSZ_TOP || wp == WMSZ_BOTTOM) { // the height was dragged: width follows
            const int ch = r->bottom - r->top - f.cy;
            r->right = r->left + ch * uiW_ / uiH_ + f.cx;
            return TRUE;
        }
        const int chNew = (r->right - r->left - f.cx) * uiH_ / uiW_;
        if (wp == WMSZ_TOPLEFT || wp == WMSZ_TOPRIGHT) r->top = r->bottom - chNew - f.cy;
        else r->bottom = r->top + chNew + f.cy;
        return TRUE;
    }
    case WM_GETMINMAXINFO: {
        auto* mm = reinterpret_cast<MINMAXINFO*>(lp);
        const SIZE f = FrameSize();
        mm->ptMinTrackSize = {uiW_ / 2 + f.cx, uiH_ / 2 + f.cy};
        MONITORINFO mi{sizeof(mi)};
        if (GetMonitorInfoW(MonitorFromWindow(hwnd_, MONITOR_DEFAULTTONEAREST), &mi)) {
            const RECT& w = mi.rcWork;
            int cw = w.right - w.left - f.cx, ch = w.bottom - w.top - f.cy;
            FitAspect(&cw, &ch);
            mm->ptMaxSize = {cw + f.cx, ch + f.cy};
            mm->ptMaxPosition = {w.left - mi.rcMonitor.left + (w.right - w.left - cw - f.cx) / 2,
                                 w.top - mi.rcMonitor.top + (w.bottom - w.top - ch - f.cy) / 2};
        }
        return 0;
    }
    case WM_WINDOWPOSCHANGING: { // snap layouts and other programmatic sizes
        auto* wpos = reinterpret_cast<WINDOWPOS*>(lp);
        if (!(wpos->flags & SWP_NOSIZE) && !IsIconic(hwnd_) && !IsZoomed(hwnd_) && wpos->cx > 0 && wpos->cy > 0) {
            const SIZE f = FrameSize();
            int cw = wpos->cx - f.cx, ch = wpos->cy - f.cy;
            FitAspect(&cw, &ch);
            wpos->cx = cw + f.cx;
            wpos->cy = ch + f.cy;
        }
        return DefWindowProcW(hwnd_, msg, wp, lp);
    }
    // Windows runs its own message loop while the window is moved or resized: keep frames coming
    // from a timer, or the VR overlay and the window would freeze for the whole drag.
    case WM_ENTERSIZEMOVE:
        SetTimer(hwnd_, 1, 33, nullptr);
        return 0;
    case WM_EXITSIZEMOVE:
        KillTimer(hwnd_, 1);
        return 0;
    case WM_TIMER:
        if (wp == 1 && cb_.tick) cb_.tick();
        return 0;
    case WM_DPICHANGED: { // moved to a monitor with another scale: keep the same logical size
        const RECT* r = reinterpret_cast<const RECT*>(lp);
        SetWindowPos(hwnd_, nullptr, r->left, r->top, r->right - r->left, r->bottom - r->top, SWP_NOZORDER | SWP_NOACTIVATE);
        return 0;
    }
    case kShowRequest:
        if (cb_.showRequested) cb_.showRequested();
        SetVisible(true);
        if (IsIconic(hwnd_)) ShowWindow(hwnd_, SW_RESTORE);
        SetForegroundWindow(hwnd_);
        return 0;
    case WM_CLOSE:
        SetVisible(false);
        if (cb_.closed) cb_.closed();
        return 0;
    case WM_ERASEBKGND:
        return 1;
    default:
        return DefWindowProcW(hwnd_, msg, wp, lp);
    }
}

} // namespace ots
