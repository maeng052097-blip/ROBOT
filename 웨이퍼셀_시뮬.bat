@echo off
rem wafer_cell simulator launcher: finds the repo, git pull, then starts the server.
set "REPO=%~dp0"
if not exist "%REPO%wafer_cell\server.py" set "REPO=C:\Users\MSY\Desktop\ROBOT\"
if not exist "%REPO%wafer_cell\server.py" set "REPO=C:\Users\MSY\Desktop\ROBOT_wafer\"
if not exist "%REPO%wafer_cell\server.py" (
  echo [ERROR] wafer_cell\server.py not found. Edit the REPO paths at the top of this file.
  pause
  exit /b 1
)
cd /d "%REPO%"
echo repo: %CD%
echo [1/2] git pull origin claude/gracious-cannon-lh630u
git pull origin claude/gracious-cannon-lh630u
if errorlevel 1 (
  echo.
  echo [WARN] git pull failed - starting with the CURRENT local code. Check local changes / network.
  pause
)
echo [2/2] starting server
start "wafer_cell server" cmd /k py -3.13 wafer_cell\server.py --speed 5
timeout /t 3 /nobreak >nul
start "" http://127.0.0.1:8765/
