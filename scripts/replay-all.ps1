<#
.SYNOPSIS
  Replays the Phase 0 sessions through the filter and prints the acceptance metrics (docs/phase1.md).
.EXAMPLE
  .\scripts\replay-all.ps1
  .\scripts\replay-all.ps1 -Set "probation=0.5","suspectWindow=0.07"
#>
# RestingDevice: serial of the device that rested still during the occlusion trials of your recordings.
param([string[]]$Set = @(), [string]$RestingDevice = '')
$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$replay = Join-Path $repo 'build\tools\ots-replay\Release\ots-replay.exe'
$eval = Join-Path $repo 'tools\analysis\eval_filter.py'
$sessions = Join-Path $env:LOCALAPPDATA 'OpenVR-TrackingStabilizer\sessions'
$outDir = Join-Path $sessions 'filtered'
New-Item -ItemType Directory -Force $outDir | Out-Null

# session, trial marker name, device that rested still during the trials (or '')
$cases = @(
    @('20261024-155027_static', '', ''),
    @('20261024-160435_occlude', 'occlude', $RestingDevice),
    @('20261024-162244_timed', '', ''),
    @('20261024-163912_bs_block', '', ''),
    @('20261024-165535_impact', '', ''),
    @('20261024-170712_impact', '', ''),
    @('20261024-170855_body', '', ''),
    # Phase 1 live recordings (raw input is replayed; the driver's own filter output is dropped)
    @('20261024-180946_occlude', 'occlude', $RestingDevice),
    @('20261024-182503_body', '', '')
)
$setArgs = @(); foreach ($s in $Set) { $setArgs += @('--set', $s) }

foreach ($c in $cases) {
    $in = Join-Path $sessions "$($c[0]).otsrec"
    $out = Join-Path $outDir "$($c[0]).otsrec"
    if (-not (Test-Path $in)) { Write-Host "skip (missing): $in"; continue }
    & $replay $in -o $out @setArgs | Out-Null
    $evalArgs = @($eval, $in, $out)
    if ($c[1] -and $c[2]) { $evalArgs += @('--trials', $c[1], '--static-device', $c[2]) }
    & python @evalArgs
    Write-Host ''
}
