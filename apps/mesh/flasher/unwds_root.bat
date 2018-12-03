@echo off
title UNWDS FLASHER
color 0A
arm-none-eabi-gdb -x gdbinit_erase_mass.txt unwds_root.elf
pause
