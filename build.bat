@echo off
REM ===========================================================================
REM  VRUM / engine-sim  ---  one-shot build script (Windows)
REM ---------------------------------------------------------------------------
REM  Provisions every dependency locally (nothing is installed system-wide) and
REM  builds engine-sim-app.exe with the V12 Gintani loaded by default.
REM
REM  Requires (already on this machine): Visual Studio 2022 (C++ workload),
REM  CMake, Git.  Everything else (SDL2, SDL2_image, Boost.filesystem, Flex,
REM  Bison, vcpkg) is downloaded automatically the first time you run this.
REM
REM  The FIRST build compiles SDL2 + Boost from source via vcpkg and can take
REM  15-40 minutes. Subsequent builds are fast.  When done, launch with run.bat.
REM ===========================================================================
setlocal enableextensions
cd /d "%~dp0"

echo(
echo === [0/6] Checking toolchain ===
where cmake >nul 2>&1 || (echo ERROR: cmake not found on PATH.& goto :fail)
where git   >nul 2>&1 || (echo ERROR: git not found on PATH.& goto :fail)

REM --- Locate Visual Studio via vswhere and prep the MSVC environment ---
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (echo ERROR: Visual Studio Installer/vswhere not found.& goto :notools)
set "VSPATH="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH goto :notools
echo Visual Studio C++ toolset: %VSPATH%

echo(
echo === [1/6] Initializing git submodules ===
git submodule update --init --recursive || goto :fail

REM  The submodules are pristine upstream; VRUM's fixes to them ship as patch
REM  files (Boost 1.91 path API, release-only D3DX libs, shared UI font).
REM  Skip a patch that is already applied so re-running the build is safe.
for %%P in (delta-studio piranha) do (
    git -C "dependencies\submodules\%%P" apply --reverse --check "%~dp0patches\%%P.patch" >nul 2>&1
    if errorlevel 1 (
        echo Applying patches\%%P.patch
        git -C "dependencies\submodules\%%P" apply "%~dp0patches\%%P.patch" || goto :fail
    ) else (
        echo patches\%%P.patch already applied
    )
)

echo(
echo === [2/6] Provisioning Flex/Bison (winflexbison) ===
set "TOOLS=%~dp0dependencies\tools"
set "WFB=%TOOLS%\winflexbison"
if not exist "%WFB%\win_flex.exe" (
    if not exist "%TOOLS%" mkdir "%TOOLS%"
    echo Downloading winflexbison 2.5.25 ...
    curl -L -o "%TOOLS%\wfb.zip" https://github.com/lexxmark/winflexbison/releases/download/v2.5.25/win_flex_bison-2.5.25.zip || goto :fail
    if not exist "%WFB%" mkdir "%WFB%"
    tar -xf "%TOOLS%\wfb.zip" -C "%WFB%" || goto :fail
    del "%TOOLS%\wfb.zip" >nul 2>&1
)
if not exist "%WFB%\win_flex.exe"  (echo ERROR: win_flex.exe missing after extract.& goto :fail)
if not exist "%WFB%\win_bison.exe" (echo ERROR: win_bison.exe missing after extract.& goto :fail)
set "PATH=%WFB%;%PATH%"
echo Flex/Bison ready: %WFB%

echo(
echo === [3/6] Provisioning vcpkg ===
set "VCPKG=%~dp0dependencies\vcpkg"
if not exist "%VCPKG%\vcpkg.exe" (
    if not exist "%VCPKG%\.git" (
        git clone https://github.com/microsoft/vcpkg "%VCPKG%" || goto :fail
    )
    call "%VCPKG%\bootstrap-vcpkg.bat" -disableMetrics || goto :fail
)
echo vcpkg ready: %VCPKG%

echo(
echo === [4/6] Installing SDL2 / SDL2_image / Boost.filesystem (first run is slow) ===
"%VCPKG%\vcpkg.exe" install sdl2 sdl2-image boost-filesystem --triplet x64-windows || goto :fail

echo(
echo === [5/6] Configuring CMake ===
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE="%VCPKG%\scripts\buildsystems\vcpkg.cmake" ^
    -DVCPKG_TARGET_TRIPLET=x64-windows ^
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
    -DDISCORD_ENABLED=OFF ^
    -DDTV=OFF ^
    -DFLEX_EXECUTABLE="%WFB%\win_flex.exe" ^
    -DBISON_EXECUTABLE="%WFB%\win_bison.exe" || goto :fail

echo(
echo === [6/6] Building engine-sim-app + engine-sim-bench (Release) ===
cmake --build build --config Release --target engine-sim-app || goto :fail
REM  The headless bench shares the whole simulation and audio path with the app,
REM  so it MUST be rebuilt in lockstep -- otherwise you measure one binary and
REM  listen to another.
cmake --build build --config Release --target engine-sim-bench || goto :fail

echo(
echo === Publishing VRUM.exe to the project root ===
REM  The app used to require build\ as the working directory, because it looked
REM  for "../assets" and "../es". It now resolves those from delta.conf next to
REM  the executable, so a copy at the root is double-clickable. The DLLs have to
REM  sit beside it -- Windows resolves them relative to the exe, not the CWD.
copy /y "build\Release\engine-sim-app.exe" "VRUM.exe" >nul || goto :fail
copy /y "build\Release\engine-sim-bench.exe" "VRUM-bench.exe" >nul
for %%D in ("build\Release\*.dll") do copy /y "%%D" "." >nul

REM  Paths are relative to this file's folder (where VRUM.exe now lives).
> "delta.conf" echo dependencies/submodules/delta-studio/engines/basic
>>"delta.conf" echo assets

echo(
echo ===========================================================================
echo  BUILD SUCCEEDED
echo  Double-click:  VRUM.exe          (in this folder -- no run.bat needed)
echo  Also at:       build\Release\engine-sim-app.exe
echo  Bench:         VRUM-bench.exe    (headless render-to-WAV, run from build\)
echo ===========================================================================
goto :eof

:notools
echo(
echo ***************************************************************************
echo  MISSING: the Visual C++ compiler (MSVC toolset) is not installed.
echo  VRUM is a C++ app and cannot be built without it.
echo(
echo  Fix it with EITHER of these, then re-run build.bat:
echo(
echo   A) Visual Studio Installer (GUI): install the
echo      "Desktop development with C++" workload.
echo      https://visualstudio.microsoft.com/downloads/  (Build Tools is enough)
echo(
echo   B) Chocolatey (run this in an ADMIN terminal):
echo      choco install -y visualstudio2022buildtools --package-parameters ^
echo        "--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
echo(
echo  The download is several GB, so it takes a while the first time.
echo ***************************************************************************
exit /b 1

:fail
echo(
echo ***************************************************************************
echo  BUILD FAILED (see the error above).  Fix it and re-run build.bat --
echo  the script is resumable and won't re-download things it already has.
echo ***************************************************************************
exit /b 1
