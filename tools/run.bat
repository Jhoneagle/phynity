@echo off
setlocal

rem Usage: tools\run.bat [preset]
set PRESET=%1
if "%PRESET%"=="" set PRESET=debug

set EXE=build\%PRESET%\src\app\phynity_demo.exe
if not exist "%EXE%" (
  echo Executable not found: %EXE%
  echo Build first with: tools\build.bat %PRESET%
  exit /b 1
)
"%EXE%"

endlocal
