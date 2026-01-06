@echo off
setlocal

rem Usage: tools\test.bat [preset] [regex]
set PRESET=%1
if "%PRESET%"=="" set PRESET=debug
set FILTER=%2

rem Map friendly filters to ctest regex
set FILTER_REGEX=%FILTER%
if /i "%FILTER%"=="unit" set FILTER_REGEX=^unit\.
if /i "%FILTER%"=="validation" set FILTER_REGEX=^validation\.
cmake --preset %PRESET%
cmake --build --preset %PRESET%
if "%FILTER%"=="" (
  ctest --preset %PRESET% --output-on-failure
) else (
  ctest --preset %PRESET% -R "%FILTER_REGEX%" --output-on-failure
)
if errorlevel 1 goto fallback
goto done

:fallback
set TESTS_ROOT=build\%PRESET%\tests
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
    )
  )
)

:done
endlocal


