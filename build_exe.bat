@echo off
setlocal
cd /d "%~dp0"
echo === build exe (PyInstaller) ===
python -m pip install --disable-pip-version-check pyinstaller
python -m PyInstaller --noconfirm --clean --console ^
  --name LidarToSpout ^
  --paths src ^
  --collect-all SpoutGL ^
  --collect-all imgui ^
  --collect-all glfw ^
  --hidden-import OpenGL ^
  src\app.py
if exist config.json copy /y config.json dist\LidarToSpout\config.json >nul
echo.
echo Hasil: dist\LidarToSpout\LidarToSpout.exe
pause
