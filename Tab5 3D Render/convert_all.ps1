# ============================================================
# convert_all.ps1 — Batch STL to .mesh converter for M5View
#
# Usage:
#   1. Drop .stl files into the 'input' folder
#   2. Run: powershell -ExecutionPolicy Bypass -File ".\convert_all.ps1"
#   3. Converted .mesh files appear in the 'output' folder
#   4. Copy output contents to /models/ on your SD card
# ============================================================

# ---- Configuration (edit these) ----------------------------
$PY_VER     = "-3.12"
$SCRIPT     = "$PSScriptRoot\tools\stl_to_mesh.py"
$INPUT_DIR  = "$PSScriptRoot\input"
$OUTPUT_DIR = "$PSScriptRoot\output"
$DECIMATE   = 7500   # Set to 0 to skip decimation

# ---- Setup -------------------------------------------------
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  M5View - Batch STL to .mesh Converter" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

if (-not (Test-Path $INPUT_DIR))  { New-Item -ItemType Directory $INPUT_DIR  | Out-Null }
if (-not (Test-Path $OUTPUT_DIR)) { New-Item -ItemType Directory $OUTPUT_DIR | Out-Null }

if (-not (Test-Path $SCRIPT)) {
    Write-Host "ERROR: stl_to_mesh.py not found at $SCRIPT" -ForegroundColor Red
    exit 1
}

# ---- Find STL files ----------------------------------------
$stl_files = Get-ChildItem $INPUT_DIR -Filter "*.stl"

if ($stl_files.Count -eq 0) {
    Write-Host "No .stl files found in: $INPUT_DIR" -ForegroundColor Yellow
    Write-Host "Drop your STL files into that folder and run again." -ForegroundColor White
    exit 0
}

Write-Host "Found $($stl_files.Count) STL file(s):" -ForegroundColor Green
foreach ($s in $stl_files) { Write-Host "  - $($s.Name)" }
Write-Host ""

if ($DECIMATE -gt 0) {
    Write-Host "Decimation target: $DECIMATE faces" -ForegroundColor Cyan
} else {
    Write-Host "Decimation: disabled" -ForegroundColor Cyan
}
Write-Host ""

# ---- Convert each file -------------------------------------
$success = 0
$failed  = 0

foreach ($stl in $stl_files) {
    $out_name = $stl.BaseName + ".mesh"
    $out_path = Join-Path $OUTPUT_DIR $out_name

    Write-Host "Converting: $($stl.Name)" -ForegroundColor White
    Write-Host "       To:  $out_name"    -ForegroundColor Gray

    if ($DECIMATE -gt 0) {
        $output = & py $PY_VER $SCRIPT $stl.FullName $out_path --decimate $DECIMATE 2>&1
    } else {
        $output = & py $PY_VER $SCRIPT $stl.FullName $out_path 2>&1
    }

    foreach ($line in $output) {
        Write-Host "  $line" -ForegroundColor DarkGray
    }

    if ((Test-Path $out_path)) {
        $size_kb = [math]::Round((Get-Item $out_path).Length / 1KB, 1)
        Write-Host "  OK - $size_kb KB" -ForegroundColor Green
        $success++
    } else {
        Write-Host "  FAILED" -ForegroundColor Red
        $failed++
    }
    Write-Host ""
}

# ---- Summary -----------------------------------------------
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Done: $success converted, $failed failed" -ForegroundColor Cyan
Write-Host "  Output: $OUTPUT_DIR" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor White
Write-Host "  1. Copy .mesh files from 'output' to /models/ on your SD card"
Write-Host "  2. Insert SD card into Tab5 and open 3D Viewer"
Write-Host ""