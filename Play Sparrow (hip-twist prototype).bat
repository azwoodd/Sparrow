@echo off
cd /d "%~dp0game"
"D:\Downloads\project-sparrow\.venv\Scripts\python.exe" play.py --skills skills_hipyaw.json %*
pause

