@echo off
setlocal
cd /d "%~dp0"
rem Replay demo sintetis (tanpa LiDAR). Buat ulang dengan: python tools\make_demo.py
if not exist data\demo_room.bin (
  echo Membuat demo sintetis...
  python tools\make_demo.py --out data\demo_room.bin
)
python src\app.py --replay data\demo_room.bin --speed 1 %*
if errorlevel 1 pause
