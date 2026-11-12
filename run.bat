@echo off
setlocal
cd /d "%~dp0"
python src\app.py %*
if errorlevel 1 pause
