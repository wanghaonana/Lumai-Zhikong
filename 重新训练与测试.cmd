@echo off
chcp 65001 >nul
cd /d "%~dp0"
runtime\python\python.exe train_validate.py --mode train
if errorlevel 1 goto end
runtime\python\python.exe tests\test_integration.py
if errorlevel 1 goto end
runtime\python\python.exe train_validate.py --mode test
:end
pause
