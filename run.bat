@echo off
setlocal
cd /d "%~dp0"
set "VIEWER_EXE=build\Release\fbik_viewer.exe"
if not exist "%VIEWER_EXE%" (
    echo ERROR: Run build.bat first.
    exit /b 1
)
"%VIEWER_EXE%" %*
exit /b %errorlevel%
