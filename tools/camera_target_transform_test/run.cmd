@echo off
setlocal
set "ROOT=%~dp0..\.."
set "GCC=C:\w64devkit\bin\gcc.exe"
if not exist "%~dp0_tmp" mkdir "%~dp0_tmp"
set "TMP=%~dp0_tmp"
set "TEMP=%~dp0_tmp"
"%GCC%" -std=c11 -O2 -Wall -Wextra -Wshadow -Werror ^
  -I"%ROOT%\Engineer\APPLICATION" ^
  "%~dp0camera_target_transform_test.c" ^
  "%ROOT%\Engineer\APPLICATION\camera_target_transform.c" ^
  -lm -o "%~dp0_tmp\camera_target_transform_test.exe" || exit /b 1
"%~dp0_tmp\camera_target_transform_test.exe"
exit /b %ERRORLEVEL%
