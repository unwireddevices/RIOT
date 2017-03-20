/* DO NOT EDIT! FILE IS AUTO GENERATED */
#ifndef UMDK_MODULES_H_
#define UMDK_MODULES_H_

#include <umdk-4btn.h>
#include <umdk-adc.h>
#include <umdk-bme280.h>
#include <umdk-counter.h>
#include <umdk-dali.h>
#include <umdk-electro.h>
#include <umdk-gpio.h>
#include <umdk-gps.h>
#include <umdk-lm75.h>
#include <umdk-lmt01.h>
#include <umdk-lps331.h>
#include <umdk-lsm6ds3.h>
#include <umdk-opt3001.h>
#include <umdk-pir.h>
#include <umdk-pwm.h>
#include <umdk-rssiecho.h>
#include <umdk-sht21.h>
#include <umdk-uart.h>

static const unwd_module_t modules[] = {
{ UNWDS_4BTN_MODULE_ID, "4btn", umdk_4btn_init, umdk_4btn_cmd, NULL},
{ UNWDS_ADC_MODULE_ID, "adc", umdk_adc_init, umdk_adc_cmd, NULL},
{ UNWDS_BME280_MODULE_ID, "bme280", umdk_bme280_init, umdk_bme280_cmd, NULL},
{ UNWDS_COUNTER_MODULE_ID, "counter", umdk_counter_init, umdk_counter_cmd, NULL},
{ UNWDS_DALI_MODULE_ID, "dali", umdk_dali_init, umdk_dali_cmd, NULL},
{ UNWDS_ELECTRO_MODULE_ID, "electro", umdk_electro_init, umdk_electro_cmd, NULL},
{ UNWDS_GPIO_MODULE_ID, "gpio", umdk_gpio_init, umdk_gpio_cmd, umdk_gpio_broadcast},
{ UNWDS_GPS_MODULE_ID, "gps", umdk_gps_init, umdk_gps_cmd, NULL},
{ UNWDS_LM75_MODULE_ID, "lm75", umdk_lm75_init, umdk_lm75_cmd, NULL},
{ UNWDS_LMT01_MODULE_ID, "lmt01", umdk_lmt01_init, umdk_lmt01_cmd, NULL},
{ UNWDS_LPS331_MODULE_ID, "lps331", umdk_lps331_init, umdk_lps331_cmd, NULL},
{ UNWDS_LSM6DS3_MODULE_ID, "lsm6ds3", umdk_lsm6ds3_init, umdk_lsm6ds3_cmd, NULL},
{ UNWDS_OPT3001_MODULE_ID, "opt3001", umdk_opt3001_init, umdk_opt3001_cmd, NULL},
{ UNWDS_PIR_MODULE_ID, "pir", umdk_pir_init, umdk_pir_cmd, NULL},
{ UNWDS_PWM_MODULE_ID, "pwm", umdk_pwm_init, umdk_pwm_cmd, NULL},
{ UNWDS_RSSIECHO_MODULE_ID, "rssiecho", umdk_rssiecho_init, umdk_rssiecho_cmd, NULL},
{ UNWDS_SHT21_MODULE_ID, "sht21", umdk_sht21_init, umdk_sht21_cmd, NULL},
{ UNWDS_UART_MODULE_ID, "uart", umdk_uart_init, umdk_uart_cmd, NULL},
{ 0, "", NULL, NULL } };

#endif
