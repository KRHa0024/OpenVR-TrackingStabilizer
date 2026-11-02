# Renders the overlay UI with demo data (ots-overlay --screenshot) and prints the RmlUi warnings, grouped.
param([string]$Out = "$PSScriptRoot\..\build\shots")
$exe = "$PSScriptRoot\..\build\tools\ots-overlay\Release\ots-overlay.exe"
$out = [IO.Path]::GetFullPath($Out)
if (Test-Path $out) { Get-ChildItem $out -File | Remove-Item }
$p = Start-Process $exe -ArgumentList '--screenshot', $out -PassThru -Wait
"exit $($p.ExitCode)"
$report = Join-Path $out 'report.txt'
if (Test-Path $report) {
    Get-Content $report | Where-Object { $_ -like 'wrote*' -or $_ -like 'FAILED*' }
    Get-Content $report | Where-Object { $_ -notlike 'wrote*' -and $_ -notlike 'FAILED*' } |
        Group-Object | Sort-Object Count -Descending | Select-Object -First 40 Count, Name | Format-Table -AutoSize -Wrap | Out-String -Width 220
}
