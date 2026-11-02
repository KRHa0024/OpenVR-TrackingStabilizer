<#
.SYNOPSIS
  Copies build/package/trackingstabilizer to a stable location and registers it with SteamVR.
.DESCRIPTION
  Installing a copy (instead of registering the build folder) keeps rebuilds possible while
  SteamVR has the DLL loaded. SteamVR must be closed while installing or removing.
.EXAMPLE
  .\scripts\install-driver.ps1            # install / update
  .\scripts\install-driver.ps1 -Uninstall # unregister and delete the installed copy
#>
param([switch]$Uninstall)
$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$package = Join-Path $repo 'build\package\trackingstabilizer'
$installRoot = Join-Path $env:LOCALAPPDATA 'OpenVR-TrackingStabilizer\driver'
$installDir = Join-Path $installRoot 'trackingstabilizer'

if (Get-Process vrserver -ErrorAction SilentlyContinue) {
    throw 'SteamVR (vrserver) is running. Close SteamVR first.'
}

$vrpaths = Get-Content (Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath') -Raw -Encoding UTF8 | ConvertFrom-Json
$vrpathreg = Join-Path $vrpaths.runtime[0] 'bin\win64\vrpathreg.exe'
if (-not (Test-Path $vrpathreg)) { throw "vrpathreg.exe not found at $vrpathreg" }

if ($Uninstall) {
    & $vrpathreg removedriver $installDir
    if (Test-Path $installDir) { Remove-Item -Recurse -Force $installDir }
    Write-Host "Uninstalled $installDir"
    return
}

if (-not (Test-Path (Join-Path $package 'bin\win64\driver_trackingstabilizer.dll'))) {
    throw "Build the project first (cmake --build build --config Release). Missing: $package"
}
New-Item -ItemType Directory -Force $installRoot | Out-Null
if (Test-Path $installDir) { Remove-Item -Recurse -Force $installDir }
Copy-Item -Recurse $package $installDir

& $vrpathreg adddriver $installDir
Write-Host "Installed to $installDir"
& $vrpathreg show | Select-String -SimpleMatch 'trackingstabilizer'
