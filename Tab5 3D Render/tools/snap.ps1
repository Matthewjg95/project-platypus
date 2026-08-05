# snap.ps1 - one screenshot from the Tab5 straight into assets/screens as PNG.
#
#   .\tools\snap.ps1 s03_sweep_dial
#   .\tools\snap.ps1 s03_sweep_dial -Burst 6 -EveryMs 2500    # transient screens
#
# Needs the USB-C cable (COM6) — this is the serial screenshot channel, it
# works in every applet state and captures the real framebuffer, not a photo.

param(
    [Parameter(Mandatory = $true)][string]$Name,
    [int]$Burst = 1,
    [int]$EveryMs = 2500,
    [string]$Port = "COM6"
)

$tools = $PSScriptRoot
$out   = Join-Path (Split-Path (Split-Path $tools -Parent) -Parent) "assets\screens"
New-Item -ItemType Directory -Force $out | Out-Null

for ($i = 1; $i -le $Burst; $i++) {
    $label = if ($Burst -eq 1) { $Name } else { "{0}_{1:d2}" -f $Name, $i }
    $bmp   = Join-Path $env:TEMP "$label.bmp"
    $png   = Join-Path $out "$label.png"

    # one retry: the first read after a reboot occasionally comes up short
    python (Join-Path $tools "tab5_screenshot.py") $Port $bmp *> $null
    if (-not (Test-Path $bmp)) {
        python (Join-Path $tools "tab5_screenshot.py") $Port $bmp *> $null
    }
    if (Test-Path $bmp) {
        python -c "from PIL import Image; Image.open(r'$bmp').save(r'$png')"
        Write-Host "saved $png" -ForegroundColor Green
    } else {
        Write-Host "capture failed ($label)" -ForegroundColor Yellow
    }
    if ($i -lt $Burst) { Start-Sleep -Milliseconds $EveryMs }
}
