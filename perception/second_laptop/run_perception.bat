@echo off
rem Run on the perception laptop (Windows):  run_perception.bat <ARC laptop IP> [camera index, default 0]
rem The first time, Windows Firewall asks about python.exe: allow it on Private AND Public networks,
rem or the enrol / lead-me keys from the ARC laptop never arrive.
if "%~1"=="" (echo usage: run_perception.bat ^<ARC laptop IP^> [camera index] & exit /b 2)
set CAM=%~2
if "%CAM%"=="" set CAM=0
if "%IMGSZ%"=="" set IMGSZ=416
cd /d "%~dp0arc-perception"
..\.venv\Scripts\python ros\perception_bridge.py --headless --udp %1:5600 --camera %CAM% --width 640 --height 480 --imgsz %IMGSZ% --max-lost-s 4 --debug-image-hz 2
