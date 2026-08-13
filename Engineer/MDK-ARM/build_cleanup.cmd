@echo off
setlocal
set "CLEANUP_TEMP=C:\Users\11737\Desktop\vison\Engineer\MDK-ARM\tmp"
if not exist "%CLEANUP_TEMP%" mkdir "%CLEANUP_TEMP%"
set "TEMP=%CLEANUP_TEMP%"
set "TMP=%CLEANUP_TEMP%"
rem ARMCC 的依赖文件和临时 .__i 文件在并行重建时可能互相争用，固定单线程构建。
"C:\Keil_v5\UV4\UV4.exe" -r "C:\Users\11737\Desktop\vison\Engineer\MDK-ARM\Engineer.uvprojx" -j1 -o "C:\Users\11737\Desktop\vison\Engineer\MDK-ARM\cleanup_build.log"
exit /b %errorlevel%
