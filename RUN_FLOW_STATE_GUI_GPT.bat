@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" "run_flow_state_gui_GPT.py" %*
) else (
    python "run_flow_state_gui_GPT.py" %*
)
if errorlevel 1 (
  echo.
  echo The demo exited with an error. Check the messages above.
)
echo.
pause
