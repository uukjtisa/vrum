@echo off
REM ===========================================================================
REM  Launch VRUM / engine-sim.
REM  The app resolves its scripts with paths like "../assets/main.mr" and
REM  "../es/", so it MUST be launched with the build\ folder as the working
REM  directory -- which is exactly what this script does.  Double-clicking the
REM  exe directly will fail to find the engine script.
REM ===========================================================================
setlocal
cd /d "%~dp0build"

if not exist "Release\engine-sim-app.exe" (
    echo engine-sim-app.exe not found. Run build.bat first.
    exit /b 1
)

echo Starting VRUM -- Lamborghini V12 [Gintani Straight-Pipe]
echo Controls: A = ignition, S = starter (hold), Q/W/E/R = throttle, Up/Down = gears
Release\engine-sim-app.exe
