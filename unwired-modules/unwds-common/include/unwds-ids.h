/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		unwds-ids.h
 * @brief       List of UMDK modules unique IDs
 * @author      Oleg Artamonov
 */
#ifndef UNWDS_IDS_H
#define UNWDS_IDS_H

typedef enum {
    UNWDS_GPIO_MODULE_ID = 1,
    UNWDS_4BTN_MODULE_ID = 2, 
    UNWDS_GPS_MODULE_ID = 3,
    UNWDS_LSM6DS3_MODULE_ID = 4,
    UNWDS_LM75_MODULE_ID = 5,
    UNWDS_LMT01_MODULE_ID = 6,
    UNWDS_UART_MODULE_ID = 7,
    UNWDS_SHT21_MODULE_ID = 8,
    UNWDS_PIR_MODULE_ID = 9,
    UNWDS_ADC_MODULE_ID = 10,
    UNWDS_LPS331_MODULE_ID = 11,
    UNWDS_COUNTER_MODULE_ID = 12,
    UNWDS_RSSIECHO_MODULE_ID = 13,
    UNWDS_PWM_MODULE_ID = 14,
    UNWDS_OPT3001_MODULE_ID = 15,
    UNWDS_DALI_MODULE_ID = 16,
    UNWDS_BME280_MODULE_ID = 17,
    UNWDS_MHZ19_MODULE_ID = 18,
    UNWDS_USOUND_MODULE_ID = 19,
    UNWDS_ADXL345_MODULE_ID = 20,
    UNWDS_IBUTTON_MODULE_ID = 21,
    UNWDS_HD44780_MODULE_ID = 22,
    UNWDS_R300_MODULE_ID = 23,
    UNWDS_IRBLASTER_MODULE_ID = 24,
    UNWDS_HX711_MODULE_ID = 25,
    /* Proprietary 50 to 99 */
    UNWDS_M200_MODULE_ID = 50,
    UNWDS_PULSE_MODULE_ID = 51,
    UNWDS_PACS_MODULE_ID = 52,
    UNWDS_SWITCH_MODULE_ID = 53,
    UNWDS_M230_MODULE_ID = 54,	
	UNWDS_IEC61107_MODULE_ID = 55,
    UNWDS_IDCARD_MODULE_UD = 56,
    /* Customer 100 to 125*/
    UNWDS_CUSTOMER_MODULE_ID = 100,
    /* System module 126 */
    UNWDS_CONFIG_MODULE_ID = 126,
} UNWDS_MODULE_IDS_t;

#endif
