@echo off
setlocal EnableDelayedExpansion
rem Windows compatibility launcher for the repository's AI skills.
rem
rem Codex's native read-only Windows sandbox cannot execute Python installed
rem below a user's AppData directory.  Prefer machine-wide interpreters that
rem remain visible inside that sandbox, then fall back to the normal launcher
rem for Claude Code/opencode and non-sandboxed command prompts.

rem Dependencies are vendored beside this launcher so the machine-wide Python
rem does not need to be modified with administrator rights.
set "PYTHONPATH=%~dp0.codex-python-packages;%PYTHONPATH%"

if exist "%ProgramData%\miniconda3\python.exe" (
    "%ProgramData%\miniconda3\python.exe" %*
    exit /b !ERRORLEVEL!
)

if exist "%ProgramFiles%\Python313\python.exe" (
    "%ProgramFiles%\Python313\python.exe" %*
    exit /b !ERRORLEVEL!
)

if exist "%ProgramFiles%\Python312\python.exe" (
    "%ProgramFiles%\Python312\python.exe" %*
    exit /b !ERRORLEVEL!
)

where py.exe >nul 2>nul
if not errorlevel 1 (
    py.exe -3 %*
    exit /b !ERRORLEVEL!
)

python.exe %*
