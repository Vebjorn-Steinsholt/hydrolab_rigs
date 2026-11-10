@echo off
rem Starts or stops the rigs on the lab's Raspberry Pi over ssh; called by the files next to it:
rem   _rig.bat start wavemaker ^| start oscillator ^| stop
rem Needs this PC's ssh key on the Pi (windows\README.md) and the sudoers rule in rpi\README.md.
rem The sudoers rule matches each command exactly, so each unit gets its own sudo call.
setlocal

rem The lab's Pi: README.md, "Lab computers"
set "PI_USER=ladertanken"
set "PI_HOST=ladertanken-rpi.local"

rem BatchMode: fail instead of asking for a password the window cannot take
set "SSH=ssh -o BatchMode=yes -o ConnectTimeout=10 %PI_USER%@%PI_HOST%"

if /i "%~1"=="stop" goto stop
if /i not "%~1"=="start" goto usage
rem Each rig has its own web page port. The other rig keeps running; the Pi refuses the
rem combinations that cannot run together (rpi\refuse_if_busy.sh) and says why.
if /i "%~2"=="wavemaker" (set "PORT=8080") else if /i "%~2"=="oscillator" (set "PORT=8081") else goto usage
set "UNIT=%~2@%PI_USER%"

echo Starting %UNIT% on %PI_HOST% ...
%SSH% "sudo -n systemctl start %UNIT% && systemctl is-active %UNIT%"
set "RC=%errorlevel%"
if %RC%==255 goto unreachable
if not %RC%==0 (
  echo Start failed, see above. On the Pi: journalctl -u %UNIT% -n 50
  echo If it was refused because the other rig runs: stop_rigs.bat first.
  goto end
)
echo Started. Opening http://%PI_HOST%:%PORT%/
rem The web server needs a few seconds after the start
timeout /t 5 /nobreak >nul
start "" "http://%PI_HOST%:%PORT%/"
goto end

:stop
echo Stopping the wavemaker and the oscillator on %PI_HOST% ...
%SSH% "sudo -n systemctl stop wavemaker@%PI_USER%; w=$?; sudo -n systemctl stop oscillator@%PI_USER%; o=$?; systemctl is-active wavemaker@%PI_USER% oscillator@%PI_USER%; [ $w = 0 ] && [ $o = 0 ]"
set "RC=%errorlevel%"
if %RC%==255 goto unreachable
if not %RC%==0 (
  echo A stop failed, see above.
  goto end
)
echo Both stopped; the drives are off.
goto end

:unreachable
echo Could not log in to %PI_USER%@%PI_HOST% over ssh. Is the Pi on, and is this PC's key on it?
echo See windows\README.md.
goto end

:usage
echo Usage: _rig.bat start wavemaker ^| start oscillator ^| stop
set "RC=2"

:end
pause
exit /b %RC%
