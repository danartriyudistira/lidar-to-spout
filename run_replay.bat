@echo off
setlocal
cd /d "%~dp0"
rem Replay data rekaman untuk development tanpa LiDAR (tidak memegang COM4)
python src\app.py --replay data\live_rec.bin --speed 1 --no-spout %*
if errorlevel 1 pause
