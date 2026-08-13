@echo off
setlocal
set "ROOT=%~dp0..\.."
set "GCC=C:\w64devkit\bin\gcc.exe"
if not exist "%~dp0_tmp" mkdir "%~dp0_tmp"
set "TMP=%~dp0_tmp"
set "TEMP=%~dp0_tmp"
"%GCC%" -std=c11 -O2 -Wall -Wextra -Wshadow -Werror ^
  -I"%ROOT%\Engineer\APPLICATION\arm" ^
  "%~dp0arm_path_replay.c" ^
  "%~dp0arm_tool_geometry_host.c" ^
  "%ROOT%\Engineer\APPLICATION\arm\arm_kinematics.c" ^
  -lm -o "%~dp0arm_path_replay.exe" || exit /b 1
pushd "%~dp0"
arm_path_replay.exe
set "RESULT=%ERRORLEVEL%"
popd
exit /b %RESULT%
