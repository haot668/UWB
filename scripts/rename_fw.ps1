# rename_fw.ps1 — Generate versioned .hex + .bin for Keil build output
#
# Usage in Keil → Options → User → After Build → Run #1:
#   powershell -ExecutionPolicy Bypass -File "..\scripts\rename_fw.ps1" "#H" "#L" "Project" "..\shared\shared_data\firmware_version.h"
#
#   #H = Keil macro → .hex file path
#   #L = Keil macro → .axf file path (for fromelf → .bin)

param(
    [string]$HexFile,
    [string]$AxfFile,
    [string]$ProjName,
    [string]$VerHeader
)

if (-not $HexFile -or -not (Test-Path $HexFile)) {
    Write-Host "[rename_fw] ERROR: hex file not found: $HexFile"
    exit 1
}

# Extract version from header
$major = 0; $minor = 0
if ($VerHeader -and (Test-Path $VerHeader)) {
    $content = Get-Content $VerHeader -Raw
    if ($content -match 'FW_VERSION_MAJOR\s+(\d+)') { $major = $Matches[1] }
    if ($content -match 'FW_VERSION_MINOR\s+(\d+)') { $minor = $Matches[1] }
}

# Output directory: <project>\Firmware\
$projDir = Split-Path (Split-Path $HexFile -Parent) -Parent
$fwDir   = Join-Path $projDir "Firmware"
if (-not (Test-Path $fwDir)) { New-Item -ItemType Directory -Path $fwDir -Force | Out-Null }

$baseName = "${ProjName}_V${major}.${minor}"

# 1. Copy .hex
$hexOut = Join-Path $fwDir "$baseName.hex"
Copy-Item $HexFile $hexOut -Force
Write-Host "[rename_fw] $baseName.hex"

# 2. Generate .bin via fromelf (bundled with Keil)
# Auto-detect fromelf.exe — check known install paths
$fromelfPaths = @(
    "D:\Tools\keil5\ARM\ARMCC\bin\fromelf.exe",
    "D:\Tools\keil5\ARM\ARMCLANG\bin\fromelf.exe",
    "C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe",
    "C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe"
)
$fromelf = $null
foreach ($p in $fromelfPaths) {
    if (Test-Path $p) { $fromelf = $p; break }
}
if (-not $fromelf) { $fromelf = "fromelf.exe" }  # fallback: hope it's in PATH

if ($AxfFile -and (Test-Path $AxfFile)) {
    $binOut = Join-Path $fwDir "$baseName.bin"
    $cmd = "`"$fromelf`" --bin --output `"$binOut`" `"$AxfFile`""
    $result = cmd /c $cmd 2>&1
    if ($LASTEXITCODE -eq 0 -and (Test-Path $binOut)) {
        Write-Host "[rename_fw] $baseName.bin"
    } else {
        Write-Host "[rename_fw] WARNING: fromelf failed — $result"
    }
} else {
    Write-Host "[rename_fw] WARNING: .axf not found, .bin skipped"
}

Write-Host "[rename_fw] -> $fwDir\"
