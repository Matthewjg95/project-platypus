# ota_push.ps1 - Build + flash the Tab5 over WiFi, in one command.
#
# WHY THIS EXISTS: `pio run -e tab5-ota -t upload` breaks in this project.
# PlatformIO's package manager tries to install tool-riscv32-esp-elf-gdb and
# dies with "VCS: Unknown repository type ..." (it misreads the symlink://
# toolchain workaround in platformio.ini). Building is unaffected, so we
# build with PlatformIO and hand the .bin to espota.py directly — no package
# manager in the upload path.
#
# Usage:   .\tools\ota_push.ps1              (default 192.168.1.32)
#          .\tools\ota_push.ps1 -Target platypus-tab5.local
#          .\tools\ota_push.ps1 -SkipBuild   (re-send the last build)
#
# THE DEVICE MUST BE ON THE M5VIEW HOME SCREEN. OTA is home-screen-only by
# design so it never fights the applets that own the radio (antenna, RF
# survey). "No response from device" almost always means you are still
# inside an applet — tap Back and retry.

param(
    [string]$Target = "192.168.1.32",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$env:PYTHONIOENCODING = "utf-8"

$root = Split-Path $PSScriptRoot -Parent
Set-Location $root

if (-not $SkipBuild) {
    Write-Host "==> building..." -ForegroundColor Cyan
    pio run -e tab5
    if ($LASTEXITCODE -ne 0) { throw "build failed - not uploading" }
}

$bin = Join-Path $root ".pio\build\tab5\firmware.bin"
if (-not (Test-Path $bin)) { throw "no firmware.bin - run without -SkipBuild" }
$kb = [math]::Round((Get-Item $bin).Length / 1KB)
Write-Host "==> uploading $kb KB to $Target" -ForegroundColor Cyan

$espota = Join-Path $env:USERPROFILE ".platformio\packages\framework-arduinoespressif32\tools\espota.py"
if (-not (Test-Path $espota)) { throw "espota.py not found at $espota" }

python $espota -i $Target -p 3232 -f $bin -r
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "UPLOAD FAILED. Checklist:" -ForegroundColor Yellow
    Write-Host "  1. Is the Tab5 on the HOME SCREEN? (green 'OTA ready' chip)"
    Write-Host "  2. Same WiFi network as this PC?"
    Write-Host "  3. Try: ping $Target"
    exit 1
}
Write-Host "==> done - device is rebooting" -ForegroundColor Green
