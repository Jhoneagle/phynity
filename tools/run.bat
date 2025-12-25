@echo off
setlocal

rem Usage: tools\run.bat [preset]
set PRESET=%1
if "%PRESET%"=="" set PRESET=debug

rem Ensure built
cmake --preset %PRESET%
cmake --build --preset %PRESET%

set EXE=build\%PRESET%\src\app\phynity_demo.exe
if not exist "%EXE%" (
  echo Executable not found: %EXE%
  exit /b 1
)
"%EXE%"

endlocal
