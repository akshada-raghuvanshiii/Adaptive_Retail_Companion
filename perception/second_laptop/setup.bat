@echo off
rem Run ONCE on the perception laptop (Windows), with internet:  setup.bat
rem Needs Python 3.10-3.12 from python.org (tick "Add python.exe to PATH").
cd /d "%~dp0"
py -m venv .venv || python -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\pip install ultralytics lap
.venv\Scripts\python -c "import ultralytics, cv2, lap; print('[ OK ] perception venv ready')"
.venv\Scripts\python find_camera.py
pause
