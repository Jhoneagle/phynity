@echo off
setlocal

rem Usage: tools\format.bat [--check]
rem Format all C++ source files in src/ and tests/ directories
rem --check: Only check formatting without modifying files (exit code 1 if changes needed)

set CHECK_ONLY=0
if "%1"=="--check" set CHECK_ONLY=1

cd /d "%~dp0\.."

if %CHECK_ONLY%==1 (
    echo Checking code formatting...
    powershell -NoProfile -Command "$files = Get-ChildItem -Path src,tests -Include *.cpp,*.hpp,*.h -Recurse; $failed = $false; foreach ($f in $files) { clang-format --dry-run -Werror $f.FullName; if ($LASTEXITCODE -ne 0) { $failed = $true } }; if ($failed) { exit 1 }"
    if errorlevel 1 (
        echo.
        echo Code formatting issues found. Run 'tools\format.bat' to fix them.
        exit /b 1
    )
    echo All files are properly formatted.
) else (
    echo Formatting all C++ files...
    powershell -NoProfile -Command "$files = Get-ChildItem -Path src,tests -Include *.cpp,*.hpp,*.h -Recurse; foreach ($f in $files) { clang-format -i $f.FullName; Write-Host ('Formatted: ' + $f.FullName) }"
    echo Done.
)
