@echo off
:: Change the working directory to the batch file's folder (gui/desktop)
cd /d "%~dp0"

:: Move two levels up into the project root directory
cd ..\..

:: Activate the virtual environment using "call" to keep this script running
call venv\Scripts\activate

:: Set the PYTHONPATH to the current directory (project root)
set PYTHONPATH=.

:: Run the script using the activated environment
python gui\desktop\main.py

:: Keep the window open to read error logs or output
pause
