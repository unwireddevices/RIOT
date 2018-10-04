/* DO NOT EDIT! FILE IS AUTO GENERATED */
#ifndef UMDK_MODULES_H_
#define UMDK_MODULES_H_

#include <umdk-rssiecho.h>
#include <umdk-config.h>
#include <umdk-adc.h>
#include <umdk-hx711.h>
#include <umdk-4btn.h>
#include <umdk-pir.h>
#include <umdk-hd44780.h>
#include <umdk-usound.h>
#include <umdk-light.h>
#include <umdk-counter.h>
#include <umdk-gps.h>
#include <umdk-meteo.h>
#include <umdk-ibutton.h>
#include <umdk-gpio.h>
#include <umdk-uart.h>
#include <umdk-lmt01.h>
#include <umdk-st95.h>
#include <umdk-mhz19.h>
#include <umdk-inclinometer.h>
#include <umdk-pwm.h>
#include <umdk-modbus.h>
#include <umdk-irblaster.h>
#include <umdk-fdc1004.h>

static const unwd_module_t modules[] = {
{ UNWDS_RSSIECHO_MODULE_ID, "rssiecho", umdk_rssiecho_init, umdk_rssiecho_cmd, NULL},
{ UNWDS_CONFIG_MODULE_ID, "config", umdk_config_init, umdk_config_cmd, NULL},
{ UNWDS_ADC_MODULE_ID, "adc", umdk_adc_init, umdk_adc_cmd, NULL},
{ UNWDS_HX711_MODULE_ID, "hx711", umdk_hx711_init, umdk_hx711_cmd, NULL},
{ UNWDS_4BTN_MODULE_ID, "4btn", umdk_4btn_init, umdk_4btn_cmd, NULL},
{ UNWDS_PIR_MODULE_ID, "pir", umdk_pir_init, umdk_pir_cmd, NULL},
{ UNWDS_HD44780_MODULE_ID, "hd44780", umdk_hd44780_init, umdk_hd44780_cmd, NULL},
{ UNWDS_USOUND_MODULE_ID, "usound", umdk_usound_init, umdk_usound_cmd, NULL},
{ UNWDS_LIGHT_MODULE_ID, "light", umdk_light_init, umdk_light_cmd, NULL},
{ UNWDS_COUNTER_MODULE_ID, "counter", umdk_counter_init, umdk_counter_cmd, NULL},
{ UNWDS_GPS_MODULE_ID, "gps", umdk_gps_init, umdk_gps_cmd, NULL},
{ UNWDS_METEO_MODULE_ID, "meteo", umdk_meteo_init, umdk_meteo_cmd, NULL},
{ UNWDS_IBUTTON_MODULE_ID, "ibutton", umdk_ibutton_init, umdk_ibutton_cmd, NULL},
{ UNWDS_GPIO_MODULE_ID, "gpio", umdk_gpio_init, umdk_gpio_cmd, umdk_gpio_broadcast},
{ UNWDS_UART_MODULE_ID, "uart", umdk_uart_init, umdk_uart_cmd, NULL},
{ UNWDS_LMT01_MODULE_ID, "lmt01", umdk_lmt01_init, umdk_lmt01_cmd, NULL},
{ UNWDS_ST95_MODULE_ID, "st95", umdk_st95_init, umdk_st95_cmd, NULL},
{ UNWDS_MHZ19_MODULE_ID, "mhz19", umdk_mhz19_init, umdk_mhz19_cmd, NULL},
{ UNWDS_INCLINOMETER_MODULE_ID, "inclinometer", umdk_inclinometer_init, umdk_inclinometer_cmd, NULL},
{ UNWDS_PWM_MODULE_ID, "pwm", umdk_pwm_init, umdk_pwm_cmd, NULL},
{ UNWDS_MODBUS_MODULE_ID, "modbus", umdk_modbus_init, umdk_modbus_cmd, NULL},
{ UNWDS_IRBLASTER_MODULE_ID, "irblaster", umdk_irblaster_init, umdk_irblaster_cmd, NULL},
{ UNWDS_FDC1004_MODULE_ID, "fdc1004", umdk_fdc1004_init, umdk_fdc1004_cmd, NULL},
{ 0, "", NULL, NULL, NULL } };

#endif
