@echo off
:: Change the working directory to the batch file's folder (gui/desktop)
cd /d "%~dp0"

:: Move two levels up into the project root directory
cd ..\..

:: Activate the virtual environment using "call" to keep this script running
call venv\Scripts\activate

:: Run the script using the activated environment
python -m gui.desktop.main

:: Keep the window open to read error logs or output
pause
