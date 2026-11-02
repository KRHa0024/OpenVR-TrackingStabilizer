<#
.SYNOPSIS
  Builds the Release configuration and packs build\dist\OpenVR-TrackingStabilizer-<version>.zip.
.DESCRIPTION
  The zip holds what an end user needs: driver\trackingstabilizer (SteamVR driver), app\ (dashboard app),
  Setup.cmd / setup.ps1 (installer), README.txt, LICENSE and THIRD_PARTY_NOTICES.txt.
.EXAMPLE
  .\scripts\make-release.ps1
  .\scripts\make-release.ps1 -NoBuild   # pack what is already built
#>
param([switch]$NoBuild)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

function Find-CMake {
    $c = Get-Command cmake -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere) {
        $vs = & $vswhere -latest -property installationPath
        $p = Join-Path $vs 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
        if (Test-Path $p) { return $p }
    }
    throw 'cmake not found (install Visual Studio 2022 with the C++ workload)'
}

if (-not $NoBuild) {
    $cmake = Find-CMake
    & $cmake -S $repo -B (Join-Path $repo 'build') -G 'Visual Studio 17 2022' -A x64
    if ($LASTEXITCODE) { throw 'configure failed' }
    & $cmake --build (Join-Path $repo 'build') --config Release
    if ($LASTEXITCODE) { throw 'build failed' }
}

$version = ([regex]::Match((Get-Content (Join-Path $repo 'CMakeLists.txt') -Raw), 'project\(\S+ VERSION ([0-9.]+)')).Groups[1].Value
$name = "OpenVR-TrackingStabilizer-$version"
$dist = Join-Path $repo "build\dist\$name"
if (Test-Path $dist) { Remove-Item -Recurse -Force $dist }
New-Item -ItemType Directory -Force $dist | Out-Null

# Driver (without the 7 MB PDB) and app.
Copy-Item -Recurse (Join-Path $repo 'build\package\trackingstabilizer') (Join-Path $dist 'driver\trackingstabilizer')
Get-ChildItem (Join-Path $dist 'driver') -Recurse -Filter *.pdb | Remove-Item
$appSrc = Join-Path $repo 'build\tools\ots-overlay\Release'
$app = New-Item -ItemType Directory -Force (Join-Path $dist 'app')
foreach ($f in 'ots-overlay.exe', 'openvr_api.dll', 'manifest.vrmanifest') { Copy-Item (Join-Path $appSrc $f) $app }
Copy-Item -Recurse (Join-Path $appSrc 'ui') $app

foreach ($f in 'Setup.cmd', 'setup.ps1', 'README.txt') { Copy-Item (Join-Path $repo "installer\$f") $dist }
Copy-Item (Join-Path $repo 'LICENSE') $dist

# Third-party licenses, straight from the submodules.
$notices = @(
    @('OpenVR SDK (Valve Corporation)', 'external\openvr\LICENSE'),
    @('MinHook (Tsuda Kageyu)', 'external\minhook\LICENSE.txt'),
    @('RmlUi', 'external\RmlUi\LICENSE.txt'),
    @('robin_hood and itlib (in RmlUi)', 'external\RmlUi\Include\RmlUi\Core\Containers\LICENSE.txt'),
    @('FreeType - portions of this software are copyright (c) The FreeType Project (www.freetype.org). All rights reserved.', 'external\freetype\docs\FTL.TXT'),
    @('lunasvg', 'external\lunasvg\LICENSE'),
    @('plutovg', 'external\lunasvg\plutovg\LICENSE'),
    @('Lucide icons', 'tools\ots-overlay\ui\icons\LICENSE.txt')
)
$text = "OpenVR Tracking Stabilizer uses the following software.`r`n"
foreach ($n in $notices) {
    $path = Join-Path $repo $n[1]
    if (-not (Test-Path $path)) { throw "license file missing: $($n[1])" }
    $text += "`r`n" + ('=' * 78) + "`r`n$($n[0])`r`n" + ('=' * 78) + "`r`n" + (Get-Content $path -Raw) + "`r`n"
}
[IO.File]::WriteAllText((Join-Path $dist 'THIRD_PARTY_NOTICES.txt'), $text, (New-Object Text.UTF8Encoding $false))

$zip = Join-Path $repo "build\dist\$name.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path (Join-Path $dist '*') -DestinationPath $zip
Write-Host "Wrote $zip"
Get-ChildItem $dist -Recurse -File | Measure-Object Length -Sum | ForEach-Object { "{0} files, {1:N1} MB unpacked" -f $_.Count, ($_.Sum / 1MB) }
