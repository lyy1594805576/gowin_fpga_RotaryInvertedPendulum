param(
    [string]$GowinShell = $env:GOWIN_SH,
    [string]$PythonExe = 'python'
)
$ErrorActionPreference = 'Stop'
if (-not $GowinShell) {
    $tool = Get-Command gw_sh.exe,gw_sh -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $tool) { throw 'Set GOWIN_SH or pass -GowinShell with the path to gw_sh.' }
    $GowinShell = $tool.Source
}
Push-Location $PSScriptRoot
try {
    & $GowinShell 'build.tcl'
    if ($LASTEXITCODE -ne 0) { throw "Gowin build failed: $LASTEXITCODE" }
    & $PythonExe './verify_build.py'
    if ($LASTEXITCODE -ne 0) { throw 'Build verification or 50 MHz timing failed.' }
} finally {
    Pop-Location
}
