@echo off
rem ============================================================================
rem rename_fw.bat — Rename Keil output to include firmware version
rem
rem Usage in Keil → Options for Target → User → After Build/Rebuild → Run #1:
rem   ..\..\scripts\rename_fw.bat "#H" "Gateway" "..\shared\shared_data\firmware_version.h"
rem
rem   #H = Keil macro → full path to .hex file
rem   Arg2 = project name prefix (e.g. "Gateway", "Master", "Slave", "Tag")
rem   Arg3 = relative path to firmware_version.h
rem
rem Output: builds/<Project>_V<M>.<m>.hex
rem ============================================================================

setlocal enabledelayedexpansion

set HEXFILE=%~1
set PROJNAME=%~2
set VERFILE=%~3

if "%HEXFILE%"=="" (
    echo [rename_fw] ERROR: no hex file specified
    exit /b 1
)

if not exist "%HEXFILE%" (
    echo [rename_fw] ERROR: hex file not found: %HEXFILE%
    exit /b 1
)

rem --- Extract version from firmware_version.h ---
set MAJOR=0
set MINOR=0

if not exist "%VERFILE%" (
    echo [rename_fw] WARNING: version header not found, using V0.0
    goto :copy
)

for /f "tokens=3" %%a in ('findstr /c:"FW_VERSION_MAJOR" "%VERFILE%"') do set MAJOR=%%a
for /f "tokens=3" %%a in ('findstr /c:"FW_VERSION_MINOR" "%VERFILE%"') do set MINOR=%%a

rem Strip trailing 'u' if present (e.g. "0u" → "0")
set MAJOR=%MAJOR:u=%
set MINOR=%MINOR:u=%

:copy
rem --- Create output directory ---
set OUTDIR=%~dp0..\builds
if not exist "%OUTDIR%" mkdir "%OUTDIR%"

set OUTNAME=%PROJNAME%_V%MAJOR%.%MINOR%.hex

echo [rename_fw] %PROJNAME% V%MAJOR%.%MINOR%
copy /y "%HEXFILE%" "%OUTDIR%\%OUTNAME%" >nul
if %errorlevel% equ 0 (
    echo [rename_fw] → %OUTDIR%\%OUTNAME%
) else (
    echo [rename_fw] ERROR: copy failed
    exit /b 1
)

endlocal
