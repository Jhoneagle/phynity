@echo off
setlocal

rem Usage: tools\build.bat [preset]
rem Presets: debug (default), release
rem Env overrides:
rem   PHYNITY_SANITIZERS=ON|OFF|auto (default: auto)
rem   PHYNITY_WARNINGS_AS_ERRORS=ON|OFF (default: ON)
rem   VCPKG_TARGET_TRIPLET=<triplet> (default: x64-windows)
rem   CMAKE_EXTRA_FLAGS="-DVAR=value" for additional CMake flags
rem   CLEAN=true to remove build\<preset> before configuring

rem Limit vcpkg parallelism to avoid file system contention
set VCPKG_MAX_CONCURRENCY=4

set PRESET=%1
if "%PRESET%"=="" set PRESET=debug

set TRIPLET=%VCPKG_TARGET_TRIPLET%
if "%TRIPLET%"=="" set TRIPLET=x64-windows

rem Sanitizers default to OFF on Windows (MSVC sanitizer support is limited)
set SANITIZERS=%PHYNITY_SANITIZERS%
if "%SANITIZERS%"=="" set SANITIZERS=auto
if "%SANITIZERS%"=="auto" set SANITIZERS=OFF

rem Keep local builds strict by default to mirror CI behavior
set WERROR=%PHYNITY_WARNINGS_AS_ERRORS%
if "%WERROR%"=="" set WERROR=ON

rem Optional clean
if /i "%CLEAN%"=="true" (
  if exist "build\%PRESET%" (
    echo Cleaning build\%PRESET%
    rmdir /s /q "build\%PRESET%"
  )
)

rem Configure + build
cmake --preset %PRESET% -DPHYNITY_ENABLE_SANITIZERS=%SANITIZERS% -DPHYNITY_WARNINGS_AS_ERRORS=%WERROR% -DVCPKG_TARGET_TRIPLET=%TRIPLET% %CMAKE_EXTRA_FLAGS%
if errorlevel 1 exit /b 1

cmake --build --preset %PRESET%
if errorlevel 1 exit /b 1

echo Build finished: build\%PRESET%

endlocal
