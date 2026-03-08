@echo off
setlocal

rem Usage: tools\test.bat [preset] [filter]
rem Filters:
rem   unit              -> ^unit\.
rem   validation        -> ^validation\.
rem   golden-compare    -> golden tests only (compare mode)
rem   golden            -> golden tests only (capture mode)
rem   <other>           -> passed directly to ctest -R as regex
rem Env overrides:
rem   VCPKG_TARGET_TRIPLET=<triplet> (default: x64-windows)
set PRESET=%1
if "%PRESET%"=="" set PRESET=debug
set FILTER=%2
set TRIPLET=%VCPKG_TARGET_TRIPLET%
if "%TRIPLET%"=="" set TRIPLET=x64-mingw-static

rem Map friendly filters to ctest regex
set FILTER_REGEX=%FILTER%
set CMAKE_EXTRA_FLAGS=
if /i "%FILTER%"=="unit" set FILTER_REGEX=^unit\.
if /i "%FILTER%"=="validation" set FILTER_REGEX=^validation\.
if /i "%FILTER%"=="golden-compare" set FILTER_REGEX=golden
if /i "%FILTER%"=="golden" (
  set FILTER_REGEX=[golden]
  set CMAKE_EXTRA_FLAGS=-DGOLDEN_CAPTURE_MODE=ON
)
cmake --preset %PRESET% -DVCPKG_TARGET_TRIPLET=%TRIPLET% %CMAKE_EXTRA_FLAGS%
if errorlevel 1 exit /b 1

cmake --build --preset %PRESET%
if errorlevel 1 exit /b 1

if "%FILTER%"=="" (
  ctest --preset %PRESET% --output-on-failure
) else (
  ctest --preset %PRESET% -R "%FILTER_REGEX%" --output-on-failure
)
if errorlevel 1 (
  set CTEST_STATUS=1
  goto fallback
)

endlocal & exit /b 0

:fallback
set TESTS_ROOT=build\%PRESET%\tests
set FALLBACK_FAILED=0
if exist "%TESTS_ROOT%" (
  echo Fallback: scanning %TESTS_ROOT% for test executables...
  for /r "%TESTS_ROOT%" %%F in (*_test.exe) do (
    set "RUN_THIS=1"
    if not "%FILTER%"=="" (
      echo %%F | findstr /i /c:"%FILTER%" >nul || set "RUN_THIS="
      if /i "%FILTER%"=="unit" (
        echo %%F | findstr /i /c:"\UnitTests\" >nul || set "RUN_THIS="
      )
      if /i "%FILTER%"=="validation" (
        echo %%F | findstr /i /c:"\ValidationTests\" >nul || set "RUN_THIS="
      )
    )
    if defined RUN_THIS (
      echo Running %%F
      call "%%F"
      if errorlevel 1 set FALLBACK_FAILED=1
    )
  )
  if "%FALLBACK_FAILED%"=="1" (
    endlocal & exit /b 1
  )
  endlocal & exit /b 0
)

endlocal & exit /b %CTEST_STATUS%


