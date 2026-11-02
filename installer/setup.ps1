<#
.SYNOPSIS
  Installs, updates or removes OpenVR Tracking Stabilizer (SteamVR driver + dashboard app) for the current user.
.DESCRIPTION
  Run from the release folder (next to driver\ and app\), usually through Setup.cmd. Nothing needs
  administrator rights: files go to %LOCALAPPDATA%\OpenVR-TrackingStabilizer.

  Install / update:
    - copies the driver and registers it with SteamVR (vrpathreg adddriver)
    - copies the app, registers its manifest with SteamVR and turns on auto-launch with SteamVR
    - adds a Start menu shortcut
  Uninstall: undoes all of that. Settings and recordings are kept unless you choose to delete them.

  SteamVR must be closed, because it rewrites its configuration files when it exits.
.EXAMPLE
  .\setup.ps1             # menu
  .\setup.ps1 -Install    # install or update without the menu
  .\setup.ps1 -Uninstall
#>
param([switch]$Install, [switch]$Uninstall)
$ErrorActionPreference = 'Stop'

$AppKey = 'ots.trackingstabilizer.manager'
$Root = Join-Path $env:LOCALAPPDATA 'OpenVR-TrackingStabilizer'
$DriverDir = Join-Path $Root 'driver\trackingstabilizer'
$AppDir = Join-Path $Root 'app'
$Shortcut = Join-Path ([Environment]::GetFolderPath('Programs')) 'OpenVR Tracking Stabilizer.lnk'
$Utf8 = New-Object System.Text.UTF8Encoding $false

function Say([string]$ja, [string]$en) { Write-Host "$ja`n  $en" }

function Wait-SteamVrClosed {
    $names = 'vrserver', 'vrmonitor', 'vrcompositor', 'vrdashboard', 'vrstartup', 'vrwebhelper'
    while (Get-Process -Name $names -ErrorAction SilentlyContinue) {
        Say 'SteamVR が起動しています。SteamVR を終了してから Enter を押してください（中止は Q）。' `
            'SteamVR is running. Close SteamVR, then press Enter (Q to cancel).'
        if ((Read-Host) -match '^[qQ]') { exit 1 }
    }
    Get-Process -Name 'ots-overlay' -ErrorAction SilentlyContinue | Stop-Process -Force
}

function Get-OpenVrPaths {
    $file = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
    if (-not (Test-Path $file)) {
        Say 'SteamVR が見つかりません。SteamVR を一度起動してから、もう一度実行してください。' `
            'SteamVR was not found. Start SteamVR once, then run this again.'
        exit 1
    }
    $p = Get-Content -LiteralPath $file -Raw -Encoding UTF8 | ConvertFrom-Json
    [pscustomobject]@{
        Runtime   = $p.runtime[0]
        Config    = $p.config[0]
        VrPathReg = Join-Path $p.runtime[0] 'bin\win64\vrpathreg.exe'
    }
}

# SteamVR's list of app manifests (config\appconfig.json). Drops every manifest of this app (old
# installs, build folders) and adds $add when given.
function Set-AppManifest([string]$config, [string]$add) {
    $file = Join-Path $config 'appconfig.json'
    $paths = @()
    if (Test-Path $file) {
        $json = Get-Content -LiteralPath $file -Raw -Encoding UTF8 | ConvertFrom-Json
        Copy-Item -LiteralPath $file "$file.bak" -Force
        foreach ($m in @($json.manifest_paths)) {
            if (-not $m) { continue }
            $ours = (Test-Path -LiteralPath $m) -and ((Get-Content -LiteralPath $m -Raw -Encoding UTF8) -match [regex]::Escape("`"$AppKey`""))
            if (-not $ours) { $paths += $m }
        }
    } else {
        $json = [pscustomobject]@{}
    }
    if ($add) { $paths += $add }
    $json | Add-Member -NotePropertyName manifest_paths -NotePropertyValue $paths -Force
    [IO.File]::WriteAllText($file, ($json | ConvertTo-Json -Depth 10), $Utf8)
}

function Set-AutoLaunch([string]$config, [bool]$on) {
    $dir = Join-Path $config 'vrappconfig'
    $file = Join-Path $dir "$AppKey.vrappconfig"
    if ($on) {
        New-Item -ItemType Directory -Force $dir | Out-Null
        [IO.File]::WriteAllText($file, "{`n   `"autolaunch`" : true,`n   `"last_launch_time`" : `"0`"`n}`n", $Utf8)
    } elseif (Test-Path $file) {
        Remove-Item $file
    }
}

function Install-All {
    $src = $PSScriptRoot
    foreach ($need in 'driver\trackingstabilizer\bin\win64\driver_trackingstabilizer.dll', 'app\ots-overlay.exe') {
        if (-not (Test-Path (Join-Path $src $need))) { throw "Missing $need next to setup.ps1. Run setup from the extracted release folder." }
    }
    Wait-SteamVrClosed
    $vr = Get-OpenVrPaths

    Say 'ドライバをコピーしています…' 'Copying the driver…'
    if (Test-Path $DriverDir) { Remove-Item -Recurse -Force $DriverDir }
    New-Item -ItemType Directory -Force (Split-Path $DriverDir) | Out-Null
    Copy-Item -Recurse (Join-Path $src 'driver\trackingstabilizer') $DriverDir
    & $vr.VrPathReg adddriver $DriverDir | Out-Null

    Say 'アプリをコピーしています…' 'Copying the app…'
    if (Test-Path $AppDir) { Remove-Item -Recurse -Force $AppDir }
    Copy-Item -Recurse (Join-Path $src 'app') $AppDir
    Set-AppManifest $vr.Config (Join-Path $AppDir 'manifest.vrmanifest')
    Set-AutoLaunch $vr.Config $true

    $shell = New-Object -ComObject WScript.Shell
    $lnk = $shell.CreateShortcut($Shortcut)
    $lnk.TargetPath = Join-Path $AppDir 'ots-overlay.exe'
    $lnk.WorkingDirectory = $AppDir
    $lnk.Description = 'OpenVR Tracking Stabilizer'
    $lnk.Save()

    Say "`nインストールしました。SteamVR を起動すると、ダッシュボードに「Tracking Stabilizer」が出ます。" `
        "Installed. Start SteamVR and open `"Tracking Stabilizer`" in the dashboard."
    Say "場所: $Root" "Location: $Root"
}

function Uninstall-All {
    Wait-SteamVrClosed
    $vr = Get-OpenVrPaths
    & $vr.VrPathReg removedriver $DriverDir | Out-Null
    Set-AppManifest $vr.Config $null
    Set-AutoLaunch $vr.Config $false
    if (Test-Path $Shortcut) { Remove-Item $Shortcut }
    foreach ($d in $DriverDir, $AppDir) { if (Test-Path $d) { Remove-Item -Recurse -Force $d } }
    Say 'ドライバとアプリを削除しました。' 'The driver and the app have been removed.'
    Say '設定（overlay.json）とレコーディング（sessions）も削除しますか？ [y/N]' 'Also delete settings (overlay.json) and recordings (sessions)? [y/N]'
    if ((Read-Host) -match '^[yY]') {
        Remove-Item -Recurse -Force $Root -ErrorAction SilentlyContinue
        Say '削除しました。' 'Deleted.'
    }
}

if ($Install) { Install-All; exit 0 }
if ($Uninstall) { Uninstall-All; exit 0 }

Write-Host 'OpenVR Tracking Stabilizer'
Say '  [1] インストール / 更新' '      Install / update'
Say '  [2] アンインストール' '      Uninstall'
Say '  [Q] 終了' '      Quit'
switch -Regex (Read-Host '>') {
    '^1' { Install-All }
    '^2' { Uninstall-All }
    default { }
}
