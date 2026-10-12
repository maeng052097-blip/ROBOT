@echo off
rem wafer_cell simulator launcher: updates the repo first, then starts the server.
cd /d "%~dp0"
if not exist "wafer_cell\server.py" (
  echo [ERROR] run this file from the ROBOT repo folder, or copy it into the repo root.
  pause
  exit /b 1
)
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
