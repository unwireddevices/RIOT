IMPROVEMENTS BY UNWIRED DEVICES
===============================

This document describes differences between Unwired Devices code and original RIOT OS code.

CPU support

* Cortex-M: function to check address validity on any Cortex-M CPU
* STM32: optimized EEPROM read/write-support (up to 8x faster, 4x lesser wear)
* STM32: proper GPIO_UNDEF support
* STM32: safer GPIO IRQ support
* STM32: improved GPIO read support
* STM32: gpio_get_status function
* STM32: pwm_start and pwm_stop functions
* STM32: improved power management support
* STM32: RTC millisecond precision support with RTC_SSR
* STM32: RTC backup registers support
* STM32: UART support for extended configuration
* STM32: UART support for on-the-fly bandwidth change
* STM32: ADC conversion to millivolts using factory calibration values
* STM32: ADC VDD and temperature calculation using factory calibration values
* STM32L0: watchdog support
* STM32L1: automatic fallback to HSI on startup if HSE is not available
* STM32L1: fixed flash clock and latency settings
* STM32L1: MSI clock support
* STM32L1: on-the-fly switching to MSI clock source
* STM32L1: automatic memory (flash, RAM, EEPROM) size calculation
* STM32L1: CPU category and model determination
* STM32L1: STOP mode power optimization
* STM32L1: watchdog support
* STM32L1: fixed CPUID support
* STM32L4: watchdog support
* STM32L4: STM32L451 ADC support

Device drivers

* Fixed ADXL345 driver (completely broken in all RIOT release up to and including at least 2018.07)
* AD5308/AD5318/AD5328 DAC driver
* FDC1004 capacitive sensor driver
* FPC1020 fingerprint sensor driver
* Improved HD44780 driver, with support for PCF8574 port expander, OLED displays, and cyrillic font on LED displays
* LIS2HH12 accelerometer driver
* Improved LM75A temperature sensor driver
* LMT01 temperature sensor driver
* LSM6DS3 gyroaccelerometer driver
* M24SR NFC EEPROM driver
* MH-Z19 CO2 sensor driver
* MT3333 GPS driver (Quectel L76, Navia KL3333, Simcom SIM68M)
* Generic 1-Wire driver (both UART and bitbanging mode)
* OPT3001 luminance sensor driver
* SHT21 temperature and humidity sensor driver (polling mode)
* Improved SX127x LoRa transciever driver
* TIC33 LCD display driver
* Ultrasonic rangefinder driver

System services

* rtctimers timers module for second-precision RTC timers
* rtctimers-millis module for millisecond-precision RTC timers
* Improved AES algorithm with 8KB smaller flash footprint
* shell_password command for shell access protection

Core functions

* byteorder.h: universal function to switch byte order
* byteorder.h: portable way to check for endianness
* debug.h: print function name automatically

Other improvements

* Improved Semtech LoRaWAN support
* Unwired Devices LoRaLAN protocol support
* Multiple Unwired Devices modules for various sensors and interfaces
* LoRaWAN application with support for Unwired Devices modules
* Support for Unwired Devices boards