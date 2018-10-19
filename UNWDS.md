IMPROVEMENTS BY UNWIRED DEVICES
===============================

This document describes differences between Unwired Devices code and original RIOT OS code.

CPU support
-----------

* Cortex-M: function to check address validity on any Cortex-M CPU
* STM32: globally available structure with basic processor information
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
* STM32: CPU model identification (full model code except package variant and temperature range, e.g. STM32L072xB)
* STM32L0: watchdog support
* STM32L1: automatic fallback to HSI on startup if HSE is not available
* STM32L1: fixed flash clock and latency settings
* STM32L1: MSI clock support
* STM32L1: on-the-fly switching to MSI clock source
* STM32L1: automatic memory (flash, RAM, EEPROM) size calculation
* STM32L1: STOP mode power optimization
* STM32L1: watchdog support
* STM32L1: fixed CPUID support
* STM32L4: watchdog support
* STM32L4: STM32L451 ADC support

Device drivers
--------------

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
* MT3333 GPS driver
* Generic 1-Wire driver (both UART and bitbanging mode)
* OPT3001 luminance sensor driver
* SHT21 temperature and humidity sensor driver (polling mode)
* Improved SX127x LoRa transciever driver
* TIC33 LCD display driver
* Ultrasonic rangefinder driver

System services
---------------

* rtctimers timers module for second-precision RTC timers
* rtctimers-millis module for millisecond-precision RTC timers
* Improved AES algorithm with 8KB smaller flash footprint
* Semtech LoRaMAC adapted to RIOT AES implementation (faster, no conflicts if some other module needs AES)
* shell_password command for shell access protection

Core functions
--------------

* byteorder.h: universal function to switch byte order
* byteorder.h: portable way to check for endianness
* debug.h: print function name automatically

Other improvements
------------------

* Improved Semtech LoRaWAN support
* Unwired Devices LoRaLAN protocol support
* Multiple Unwired Devices modules for various sensors and interfaces
* LoRaWAN application with support for Unwired Devices modules
* Support for Unwired Devices boards

Unwired Modules
---------------

Unwired Modules are created to be used in user-level modular application
to provide support for various devices, sensors, and interfaces. Modules
are used with both LoRaLAN and LoRaWAN applications.

Please note that Unwired Modules are not based on existing RIOT code and
so is not required to be LGPL licensed. Unwired Modules are distributed
with more permissive MIT-style license.

* umdk-4btn - pushbutton support
* umdk-adc - ADC support
* umdk-config - end-point device remote configuration support
* umdk-counter - pulse counter, both polling and IRQ modes
* umdk-fdc1004 - capacitive sensor (alpha)
* umdk-gpio - GPIO read/write support
* umdk-gps - MT3333-based GPS device support (Quectel L76, Navia KL3333, Simcom SIM68M)
* umdk-hd44780 - HD44780 and compatible LCD/OLED displays support
* umdk-hx711 - HX711 ADC support
* umdk-ibutton - 1-Wire i-Button support
* umdk-inclinometer - accelerometer-based inclinometer (ADXL345, LIS2HH12, LSM6DS3 supported)
* umdk-irblaster - IR blaster (alpha)
* umdk-light - illuminance sensor (OPT3001 supported)
* umdk-lmt01 - LMT01 temperature sensor support (up to 4 sensors by default)
* umdk-meteo - air temperature, humidity and pressure (SHT21, BME280, LPS331, LM75A supported)
* umdk-mhz19 - MH-Z19 CO2 sensor
* umdk-modbus - MODBUS RTU support
* umdk-pawn - PAWN scripting language (experimental, not included in firmwares)
* umdk-pir - PIR presense detector (generic Chinese modules)
* umdk-pwm - GPIO PWM
* umdk-rssiecho - radio link check
* umdk-st95 - NFC card reader and emulator
* umdk-uart - RS232/RS485/UART-over-radio transparent bridge
* umdk-usound - ultrasound rangefinder (not Chinese modules but our own schematics and firmware)

Modules listed below are available in commercial firmware only. These are not open source.

* umdk-cl420 - 4-20 mA current loop using integrated ADC and external analog front-end
* umdk-dali - Digital Addressable Lighting Interface support
* umdk-iec61107 - IEC-61107 electricity meters support (i.e. Energomera meters)
* umdk-m200 - Incotex Mercury M200 electricity meters support (both CAN and RS485)
* umdk-m230 - Incotex Mercury M230 electricity meters support (both CAN and RS485)
* umdk-pacs - i-Button based Physical Access Control System
* umdk-parking - vehicle detection sensor based on ST LIS3MDL magnetometer
* umdk-pulse - water/electricity/etc. meters with pulse output support
* umdk-switch - on/off detection module for dry contacts sensors
* umdk-wiegand - Wiegand PACS interface support

Applications
------------

Please note that Unwired Applications are not based on existing RIOT code and
so is not required to be LGPL licensed. Unwired Applications are distributed
with more permissive MIT-style license.

* loralan - complete LoRaLAN end-point device firmware application
* loralan-gateway - complete LoRaLAN gateway device application
* loralan-wan - complete LoRaWAN end-point device application

Applications are supported by the following modules (apps/unwds-common directory):

* loralan-common - common LoRaLAN-related functions
* loralan-device - LoRaLAN MAC for end-point device
* loralan-gateway - LoRaLAN MAC for gateway device
* loralan-mac - common LoRaLAN MAC functions and definitions
* unwds-common - various useful functions and utilities not related to LoRa MAC implementation
