#pragma once
// Direct3D 11 device plus RmlUi: the UI is rendered into one texture, which the SteamVR overlay and the
// desktop window both show.
#include <d3d11.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace Rml {
class Context;
}

namespace ots {

class RmlRenderer;
class RmlSystem;
class RmlFiles;

class Gfx {
public:
    Gfx();
    ~Gfx();

    // Creates the device and render target, initialises RmlUi and loads the fonts. Relative UI paths
    // (documents, images, icons) resolve against `baseDir`.
    bool Init(int width, int height, const std::filesystem::path& baseDir);
    void Shutdown();
    // New render target size (the old texture is released: callers must not keep it).
    bool Resize(int width, int height);

    void Render(Rml::Context* context);

    ID3D11Device* Device() const { return device_; }
    ID3D11DeviceContext* Context() const { return context_; }
    ID3D11Texture2D* Texture() const { return texture_; }
    int Width() const { return width_; }
    int Height() const { return height_; }

    bool SavePng(const std::wstring& path);
    // RmlUi warnings and errors since the last call (screenshot mode writes them to its report).
    std::vector<std::string> TakeLog();

private:
    bool LoadFonts();
    bool CreateTarget(int width, int height);

    int width_ = 0, height_ = 0;
    ID3D11Device* device_ = nullptr;
    ID3D11DeviceContext* context_ = nullptr;
    ID3D11Texture2D* texture_ = nullptr;
    ID3D11RenderTargetView* rtv_ = nullptr;
    std::unique_ptr<RmlRenderer> renderer_;
    std::unique_ptr<RmlSystem> system_;
    std::unique_ptr<RmlFiles> files_;
    bool rmlInitialised_ = false;
};

} // namespace ots
