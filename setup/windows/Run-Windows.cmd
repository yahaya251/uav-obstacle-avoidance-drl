@echo off
setlocal EnableExtensions DisableDelayedExpansion
set "pwsh=%~dp0..\..\.windows\powershell\pwsh.exe"
if exist "%pwsh%" goto run
if exist "%~dp0..\..\..\..\START_HERE.md" (
    set "pwsh=%~dp0..\..\..\..\.windows\powershell\pwsh.exe"
)
if exist "%pwsh%" goto run
set "pwsh=%ProgramFiles%\PowerShell\7\pwsh.exe"
if exist "%pwsh%" goto run
where.exe /q pwsh.exe
if not errorlevel 1 (
    set "pwsh=pwsh.exe"
    goto run
)
echo PowerShell 7 was not found. Place the portable version under .windows\powershell or install PowerShell 7.
set "run_exit=1"
goto finish

:run
"%pwsh%" -NoLogo -NoProfile -File "%~dp0Run-Windows.ps1" %*
set "run_exit=%errorlevel%"

:finish
if defined WIA3002_NO_PAUSE exit /b %run_exit%
for %%a in (%*) do if /i "%%~a"=="-NonInteractive" exit /b %run_exit%
echo Press any key to close this window...
pause >nul
exit /b %run_exit%
