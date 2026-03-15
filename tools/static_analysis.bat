@echo off
setlocal EnableDelayedExpansion

rem Usage: tools\static_analysis.bat [preset]
rem Run clang-tidy using the compile_commands.json from build\<preset>
rem Presets: release (default), debug, debug-asan, debug-tsan, release-lto

set PRESET=%1
if "%PRESET%"=="" set PRESET=release
set BUILD_DIR=build\%PRESET%
set LOG_DIR=%BUILD_DIR%\logs
set LOG_FILE=%LOG_DIR%\static_analysis.log

cd /d "%~dp0\.."

if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"

echo Writing full static analysis output to: !LOG_FILE!
call :run %PRESET% > "!LOG_FILE!" 2>&1
set SCRIPT_EXIT=%ERRORLEVEL%
echo Static analysis finished with exit code !SCRIPT_EXIT!. Log: !LOG_FILE!
exit /b !SCRIPT_EXIT!

:run
set PRESET=%1
if "%PRESET%"=="" set PRESET=release
set BUILD_DIR=build\%PRESET%

where clang-tidy >nul 2>nul
if errorlevel 1 (
    echo clang-tidy not found in PATH.
    exit /b 1
)

where cmake >nul 2>nul
if errorlevel 1 (
    echo cmake not found in PATH.
    exit /b 1
)

cd /d "%~dp0\.."

echo Configuring preset '%PRESET%' for static analysis...
cmake --preset %PRESET%
if errorlevel 1 exit /b 1

if not exist "%BUILD_DIR%\compile_commands.json" (
    echo Generating compile_commands.json for '%PRESET%'...
    cmake --build --preset %PRESET%
    if errorlevel 1 exit /b 1
)

echo Running clang-tidy over src\ and tests\ (*.cpp,*.hpp,*.h) using %BUILD_DIR%\compile_commands.json...
powershell -NoProfile -Command "$files = Get-ChildItem -Path src,tests -Include *.cpp,*.hpp,*.h -Recurse | Sort-Object FullName; $failed = $false; foreach ($f in $files) { Write-Host ('Analyzing: ' + $f.FullName); clang-tidy -p '%BUILD_DIR%' $f.FullName; if ($LASTEXITCODE -ne 0) { $failed = $true } }; if ($failed) { exit 1 }"
if errorlevel 1 (
    echo.
    echo Static analysis issues found.
    exit /b 1
)

echo Static analysis complete.

endlocal