@echo off
rem Stops the wavemaker and the oscillator; each disables its drive. The Pi stays on.
call "%~dp0_rig.bat" stop
