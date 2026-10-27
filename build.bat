@echo off
setlocal
cd /d "%~dp0"
set "BUILD_DIR=build"
set "VSINSTALL="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
        set "VSINSTALL=%%i"
    )
)
if not defined VSINSTALL (
    echo ERROR: Visual Studio with C++ tools not found.
    exit /b 1
)
set "VCVARS=%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat"
set "NINJA_DIR=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
set "CMAKE_DIR=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
if not exist "%VCVARS%" (
    echo ERROR: vcvars64.bat not found.
    exit /b 1
)
if not exist "%NINJA_DIR%\ninja.exe" (
    echo ERROR: Visual Studio's bundled Ninja not found.
    exit /b 1
)
set "PATH=%CMAKE_DIR%;%NINJA_DIR%;%PATH%"
call "%VCVARS%"
if errorlevel 1 exit /b 1

echo Configuring...
cmake -S . -B "%BUILD_DIR%" -G "Ninja Multi-Config" %*
if errorlevel 1 exit /b 1
echo Building Debug...
cmake --build "%BUILD_DIR%" --config Debug
if errorlevel 1 exit /b 1
echo Building Release...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b 1
echo Done. Run run.bat to open the viewer.
endlocal
