@echo off
setlocal

set PRESET=%1
if "%PRESET%"=="" set PRESET=RelWithDebInfo

if exist build rmdir /Q /S build

cmake --preset %PRESET% -B build

REM Build default target (works for Ninja and VS)
cmake --build build --config %PRESET%

endlocal