@echo off
setlocal

rem Usage: tools\test.bat [preset] [regex]
set PRESET=%1
if "%PRESET%"=="" set PRESET=debug
set FILTER=%2

rem Configure + build with CMake presets
cmake --preset %PRESET%
cmake --build --preset %PRESET%

rem Try CTest first
if "%FILTER%"=="" (
  ctest --preset %PRESET% --output-on-failure
) else (
  ctest --preset %PRESET% -R %FILTER% --output-on-failure
)

rem If CTest didn't find tests or had issues, run core tests directly
set CORE_DIR=build\%PRESET%\tests\core
if exist "%CORE_DIR%\vec3_test.exe" (
  echo Running vec3_test.exe
  "%CORE_DIR%\vec3_test.exe"
)
if exist "%CORE_DIR%\particle_test.exe" (
  echo Running particle_test.exe
  "%CORE_DIR%\particle_test.exe"
)

endlocal
