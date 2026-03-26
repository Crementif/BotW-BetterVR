@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ZIP_NAME=BetterVR.zip"
set "TEMP_DIR=temp_package"

REM Clean up previous run
if exist "%TEMP_DIR%" rmdir /S /Q "%TEMP_DIR%"
if exist "%ZIP_NAME%" del "%ZIP_NAME%"

REM Create directory structure
mkdir "%TEMP_DIR%"
mkdir "%TEMP_DIR%\BreathOfTheWild_BetterVR"

REM -------------------------
REM Helper: locate a file
REM Usage: call :findfile VAR "filename"
REM -------------------------
call :findfile DLL_PATH "BetterVR_Layer.dll"
call :findfile JSON_PATH "BetterVR_Layer.json"
call :findfile LIB_PATH "BetterVR_Layer.lib"
call :findfile PDB_PATH "BetterVR_Layer.pdb"

if not defined DLL_PATH  (echo Error: BetterVR_Layer.dll not found.& goto :error)
if not defined JSON_PATH (echo Error: BetterVR_Layer.json not found.& goto :error)
if not defined LIB_PATH  (echo Error: BetterVR_Layer.lib not found.& goto :error)
if not defined PDB_PATH  (echo Error: BetterVR_Layer.pdb not found.& goto :error)

echo Using:
echo   DLL:  "!DLL_PATH!"
echo   JSON: "!JSON_PATH!"
echo   LIB:  "!LIB_PATH!"
echo   PDB:  "!PDB_PATH!"

copy "!DLL_PATH!"  "%TEMP_DIR%\" >nul || goto :error
copy "!JSON_PATH!" "%TEMP_DIR%\" >nul || goto :error
copy "!LIB_PATH!"  "%TEMP_DIR%\" >nul || goto :error
copy "!PDB_PATH!"  "%TEMP_DIR%\" >nul || goto :error

REM Copy launch scripts
call :copyreq "resources\BetterVR LAUNCH CEMU IN VR.bat" "%TEMP_DIR%\BetterVR LAUNCH CEMU IN VR.bat"
call :copyreq "resources\BetterVR LAUNCH CEMU IN VR - COMPATIBILITY MODE.bat" "%TEMP_DIR%\BetterVR LAUNCH CEMU IN VR - COMPATIBILITY MODE.bat"
call :copyreq "resources\BetterVR UNINSTALL.bat" "%TEMP_DIR%\BetterVR UNINSTALL.bat"

REM Copy Graphic Packs
if exist "resources\BreathOfTheWild_BetterVR" (
    xcopy /E /I /Y "resources\BreathOfTheWild_BetterVR" "%TEMP_DIR%\BreathOfTheWild_BetterVR" >nul || goto :error
) else (
    echo Error: resources\BreathOfTheWild_BetterVR not found.
    goto :error
)

REM Create Zip
echo Creating %ZIP_NAME%...
powershell -NoProfile -Command "Compress-Archive -Force -Path '%TEMP_DIR%\*' -DestinationPath '%ZIP_NAME%'" || goto :error

REM Cleanup
rmdir /S /Q "%TEMP_DIR%"
echo Package created successfully: %ZIP_NAME%
exit /b 0

:copyreq
if exist "%~1" (
  copy "%~1" "%~2" >nul || exit /b 1
  exit /b 0
) else (
  echo Error: %~1 not found.
  exit /b 1
)

:findfile
set "%~1="
set "FN=%~2"

REM Preferred locations first (fast)
for %%P in (
  "Cemu\%FN%"
  "bin\%FN%"
  "build\bin\%FN%"
  "build\%FN%"
) do (
  if exist "%%~P" (
    set "%~1=%%~P"
    exit /b 0
  )
)

REM Fallback: search repo (slower, but robust)
for /f "delims=" %%F in ('dir /b /s "%FN%" 2^>nul') do (
  set "%~1=%%F"
  exit /b 0
)
exit /b 0

:error
echo Failed to create package.
if exist "%TEMP_DIR%" rmdir /S /Q "%TEMP_DIR%"
exit /b 1
