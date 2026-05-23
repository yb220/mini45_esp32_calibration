@echo off
setlocal
set "PROJECT_DIR=%~dp0"
set "PYTHONPATH=%PROJECT_DIR%.deps_py39;%PROJECT_DIR%;%PYTHONPATH%"
"D:\anaconda3\envs\python3-9\python.exe" -m app.main
endlocal
