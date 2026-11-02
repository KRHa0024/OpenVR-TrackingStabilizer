#include "Gfx.h"

#include "RmlUi_Renderer_DX11.h"

#include <RmlUi/Core.h>

#include <windows.h>
#include <wincodec.h>

#include <chrono>
#include <cstdio>
#include <mutex>

namespace ots {

// RmlUi's DX11 renderer only reads TGA files; decode everything else (PNG) with WIC.
class RmlRenderer : public RenderInterface_DX11 {
public:
    RmlRenderer(ID3D11Device* device, std::filesystem::path base) : RenderInterface_DX11(device), base_(std::move(base)) {}

    Rml::TextureHandle LoadTexture(Rml::Vector2i& dimensions, const Rml::String& source) override {
        if (source.size() < 4 || _stricmp(source.c_str() + source.size() - 4, ".tga") == 0)
            return RenderInterface_DX11::LoadTexture(dimensions, source);
        std::vector<Rml::byte> pixels;
        const std::filesystem::path p(std::u8string(source.begin(), source.end()));
        if (!DecodeImage(p.is_absolute() ? p : base_ / p, &pixels, &dimensions)) {
            Rml::Log::Message(Rml::Log::LT_WARNING, "could not decode image %s", source.c_str());
            return {};
        }
        return GenerateTexture(pixels, dimensions);
    }

private:
    // RmlUi wants premultiplied RGBA.
    static bool DecodeImage(const std::filesystem::path& path, std::vector<Rml::byte>* out, Rml::Vector2i* size) {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        IWICImagingFactory* wic = nullptr;
        IWICBitmapDecoder* dec = nullptr;
        IWICBitmapFrameDecode* frame = nullptr;
        IWICFormatConverter* conv = nullptr;
        bool ok = SUCCEEDED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&wic))) &&
                  SUCCEEDED(wic->CreateDecoderFromFilename(path.c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &dec)) &&
                  SUCCEEDED(dec->GetFrame(0, &frame)) && SUCCEEDED(wic->CreateFormatConverter(&conv)) &&
                  SUCCEEDED(conv->Initialize(frame, GUID_WICPixelFormat32bppPRGBA, WICBitmapDitherTypeNone, nullptr, 0, WICBitmapPaletteTypeCustom));
        UINT w = 0, h = 0;
        if (ok) ok = SUCCEEDED(conv->GetSize(&w, &h)) && w > 0 && h > 0;
        if (ok) {
            out->resize(size_t(w) * h * 4);
            ok = SUCCEEDED(conv->CopyPixels(nullptr, w * 4, UINT(out->size()), out->data()));
            *size = {int(w), int(h)};
        }
        if (conv) conv->Release();
        if (frame) frame->Release();
        if (dec) dec->Release();
        if (wic) wic->Release();
        return ok;
    }

    std::filesystem::path base_;
};

// Relative paths resolve against the exe folder; paths are UTF-8, so folders with non-ASCII names work.
// (Document URLs must stay relative: RmlUi would read "F:" in an absolute path as a URL scheme.)
class RmlFiles : public Rml::FileInterface {
public:
    explicit RmlFiles(std::filesystem::path base) : base_(std::move(base)) {}
    Rml::FileHandle Open(const Rml::String& path) override {
        const std::filesystem::path p(std::u8string(path.begin(), path.end()));
        FILE* f = _wfopen((p.is_absolute() ? p : base_ / p).c_str(), L"rb");
        return Rml::FileHandle(f);
    }
    void Close(Rml::FileHandle file) override { std::fclose(reinterpret_cast<FILE*>(file)); }
    size_t Read(void* buffer, size_t size, Rml::FileHandle file) override { return std::fread(buffer, 1, size, reinterpret_cast<FILE*>(file)); }
    bool Seek(Rml::FileHandle file, long offset, int origin) override { return std::fseek(reinterpret_cast<FILE*>(file), offset, origin) == 0; }
    size_t Tell(Rml::FileHandle file) override { return size_t(std::ftell(reinterpret_cast<FILE*>(file))); }

private:
    std::filesystem::path base_;
};

class RmlSystem : public Rml::SystemInterface {
public:
    double GetElapsedTime() override {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }
    bool LogMessage(Rml::Log::Type type, const Rml::String& message) override {
        if (type <= Rml::Log::LT_WARNING) {
            const char* tag = type == Rml::Log::LT_WARNING ? "warning" : "error";
            std::fprintf(stderr, "[rmlui %s] %s\n", tag, message.c_str());
            std::lock_guard lock(m_);
            if (log_.size() < 500) log_.push_back(std::string(tag) + ": " + message);
        }
        return true;
    }
    std::vector<std::string> TakeLog() {
        std::lock_guard lock(m_);
        return std::exchange(log_, {});
    }

private:
    const std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
    std::mutex m_;
    std::vector<std::string> log_;
};

Gfx::Gfx() = default;
Gfx::~Gfx() { Shutdown(); }

bool Gfx::Init(int width, int height, const std::filesystem::path& baseDir) {
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
    if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, D3D11_CREATE_DEVICE_BGRA_SUPPORT, levels, 2,
                                 D3D11_SDK_VERSION, &device_, nullptr, &context_)))
        return false;
    if (!CreateTarget(width, height)) return false;

    system_ = std::make_unique<RmlSystem>();
    files_ = std::make_unique<RmlFiles>(baseDir);
    renderer_ = std::make_unique<RmlRenderer>(device_, baseDir);
    renderer_->SetViewport(width, height);
    Rml::SetSystemInterface(system_.get());
    Rml::SetFileInterface(files_.get());
    Rml::SetRenderInterface(renderer_.get());
    if (!Rml::Initialise()) return false;
    rmlInitialised_ = true;
    return LoadFonts();
}

bool Gfx::CreateTarget(int width, int height) {
    if (rtv_) rtv_->Release(), rtv_ = nullptr;
    if (texture_) texture_->Release(), texture_ = nullptr;
    width_ = width;
    height_ = height;
    D3D11_TEXTURE2D_DESC td{};
    td.Width = UINT(width);
    td.Height = UINT(height);
    td.MipLevels = 1;
    td.ArraySize = 1;
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = 1;
    td.Usage = D3D11_USAGE_DEFAULT;
    td.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    return SUCCEEDED(device_->CreateTexture2D(&td, nullptr, &texture_)) && SUCCEEDED(device_->CreateRenderTargetView(texture_, nullptr, &rtv_));
}

bool Gfx::Resize(int width, int height) {
    if (width == width_ && height == height_) return true;
    if (!CreateTarget(width, height)) return false;
    renderer_->SetViewport(width, height); // RmlUi's own layers follow on the next frame
    return true;
}

void Gfx::Shutdown() {
    if (rmlInitialised_) {
        Rml::Shutdown();
        rmlInitialised_ = false;
    }
    renderer_.reset();
    files_.reset();
    system_.reset();
    if (rtv_) rtv_->Release(), rtv_ = nullptr;
    if (texture_) texture_->Release(), texture_ = nullptr;
    if (context_) context_->Release(), context_ = nullptr;
    if (device_) device_->Release(), device_ = nullptr;
}

// System fonts, registered under the family names the stylesheet uses: "ui" (Japanese capable) and "mono".
bool Gfx::LoadFonts() {
    auto exists = [](const char* p) { return GetFileAttributesA(p) != INVALID_FILE_ATTRIBUTES; };
    using Rml::Style::FontStyle;
    using Rml::Style::FontWeight;
    // Regular / bold candidates, first one present wins (Noto Sans JP is a variable font: both weights).
    struct Pair {
        const char* regular;
        const char* bold;
    };
    const Pair candidates[] = {{"C:\\Windows\\Fonts\\NotoSansJP-VF.ttf", "C:\\Windows\\Fonts\\NotoSansJP-VF.ttf"},
                               {"C:\\Windows\\Fonts\\YuGothM.ttc", "C:\\Windows\\Fonts\\YuGothB.ttc"},
                               {"C:\\Windows\\Fonts\\meiryo.ttc", "C:\\Windows\\Fonts\\meiryob.ttc"}};
    bool ui = false;
    for (const Pair& p : candidates) {
        if (!exists(p.regular)) continue;
        ui = Rml::LoadFontFace(p.regular, "ui", FontStyle::Normal, FontWeight::Normal);
        Rml::LoadFontFace(p.bold, "ui", FontStyle::Normal, FontWeight::Bold);
        // Also the fallback for glyphs other faces lack (Japanese inside monospaced text, for example).
        Rml::LoadFontFace(p.regular, true, FontWeight::Normal);
        if (ui) break;
    }
    if (exists("C:\\Windows\\Fonts\\seguisym.ttf")) Rml::LoadFontFace("C:\\Windows\\Fonts\\seguisym.ttf", true);
    Rml::LoadFontFace("C:\\Windows\\Fonts\\consola.ttf", "mono", FontStyle::Normal, FontWeight::Normal);
    Rml::LoadFontFace("C:\\Windows\\Fonts\\consolab.ttf", "mono", FontStyle::Normal, FontWeight::Bold);
    return ui;
}

void Gfx::Render(Rml::Context* rmlContext) {
    const float clear[4] = {0, 0, 0, 1};
    context_->ClearRenderTargetView(rtv_, clear);
    renderer_->BeginFrame();
    rmlContext->Render();
    renderer_->EndFrame(rtv_);
    context_->Flush();
}

std::vector<std::string> Gfx::TakeLog() { return system_ ? system_->TakeLog() : std::vector<std::string>{}; }

bool Gfx::SavePng(const std::wstring& path) {
    D3D11_TEXTURE2D_DESC td;
    texture_->GetDesc(&td);
    td.Usage = D3D11_USAGE_STAGING;
    td.BindFlags = 0;
    td.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    ID3D11Texture2D* staging = nullptr;
    if (FAILED(device_->CreateTexture2D(&td, nullptr, &staging))) return false;
    context_->CopyResource(staging, texture_);
    D3D11_MAPPED_SUBRESOURCE map;
    if (FAILED(context_->Map(staging, 0, D3D11_MAP_READ, 0, &map))) {
        staging->Release();
        return false;
    }
    std::vector<uint8_t> pixels(size_t(width_) * height_ * 4);
    for (int y = 0; y < height_; ++y)
        memcpy(pixels.data() + size_t(y) * width_ * 4, static_cast<uint8_t*>(map.pData) + size_t(y) * map.RowPitch, size_t(width_) * 4);
    context_->Unmap(staging, 0);
    staging->Release();

    bool ok = false;
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    IWICImagingFactory* wic = nullptr;
    IWICStream* stream = nullptr;
    IWICBitmapEncoder* enc = nullptr;
    IWICBitmapFrameEncode* frame = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&wic))) &&
        SUCCEEDED(wic->CreateStream(&stream)) && SUCCEEDED(stream->InitializeFromFilename(path.c_str(), GENERIC_WRITE)) &&
        SUCCEEDED(wic->CreateEncoder(GUID_ContainerFormatPng, nullptr, &enc)) && SUCCEEDED(enc->Initialize(stream, WICBitmapEncoderNoCache)) &&
        SUCCEEDED(enc->CreateNewFrame(&frame, nullptr)) && SUCCEEDED(frame->Initialize(nullptr)) &&
        SUCCEEDED(frame->SetSize(UINT(width_), UINT(height_)))) {
        WICPixelFormatGUID fmt = GUID_WICPixelFormat32bppRGBA;
        frame->SetPixelFormat(&fmt); // the PNG encoder may answer with BGRA instead
        if (fmt == GUID_WICPixelFormat32bppBGRA)
            for (size_t i = 0; i < pixels.size(); i += 4) std::swap(pixels[i], pixels[i + 2]);
        ok = SUCCEEDED(frame->WritePixels(UINT(height_), UINT(width_) * 4, UINT(pixels.size()), pixels.data())) &&
             SUCCEEDED(frame->Commit()) && SUCCEEDED(enc->Commit());
    }
    if (frame) frame->Release();
    if (enc) enc->Release();
    if (stream) stream->Release();
    if (wic) wic->Release();
    return ok;
}

} // namespace ots
