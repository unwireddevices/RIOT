@echo off
title UNWDS FLASHER
color 0A
set /p com="COM port: "
:start
	stm32flash -R -E -w tele2med_wolf.hex COM%com%
	pause
	cls
goto start
