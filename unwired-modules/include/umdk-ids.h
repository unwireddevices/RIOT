/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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
    UNWDS_GASSENSOR_MODULE_ID = 4,
    UNWDS_SOIL_MODULE_ID = 5,
    UNWDS_LMT01_MODULE_ID = 6,
    UNWDS_UART_MODULE_ID = 7,
    /* */
    UNWDS_PIR_MODULE_ID = 9,
    UNWDS_ADC_MODULE_ID = 10,
    /* */
    UNWDS_COUNTER_MODULE_ID = 12,
    UNWDS_RSSIECHO_MODULE_ID = 13,
    UNWDS_PWM_MODULE_ID = 14,
    UNWDS_LIGHT_MODULE_ID = 15,
    /* */
    UNWDS_METEO_MODULE_ID = 17,
    UNWDS_MHZ19_MODULE_ID = 18,
    UNWDS_USONIC_MODULE_ID = 19,
    UNWDS_INCLINOMETER_MODULE_ID = 20,
    UNWDS_IBUTTON_MODULE_ID = 21,
    UNWDS_HD44780_MODULE_ID = 22,
    UNWDS_R300_MODULE_ID = 23,
    UNWDS_IRBLASTER_MODULE_ID = 24,
    UNWDS_HX711_MODULE_ID = 25,
    UNWDS_FDC1004_MODULE_ID = 26,
	UNWDS_CL420_MODULE_ID = 27,
	UNWDS_MODBUS_MODULE_ID = 28,
	UNWDS_RADIORELAY_MODULE_ID = 29,
	UNWDS_ST95_MODULE_ID = 30,
    /* Proprietary 50 to 99 */
    UNWDS_M200_MODULE_ID = 50,
    UNWDS_PULSE_MODULE_ID = 51,
    UNWDS_PACS_MODULE_ID = 52,
    UNWDS_SWITCH_MODULE_ID = 53,
    UNWDS_M230_MODULE_ID = 54,	
	UNWDS_IEC61107_MODULE_ID = 55,
    UNWDS_IDCARD_MODULE_ID = 56,
	UNWDS_DALI_MODULE_ID = 57,
	UNWDS_WIEGAND_MODULE_ID = 58,
    UNWDS_COWMON_MODULE_ID = 59,
    UNWDS_PARKING_MODULE_ID = 60,
    UNWDS_GARBAGE_MODULE_ID = 61,
    UNWDS_URF_MODULE_ID = 62,
    UNWDS_COW_MODULE_ID = 63,
    UNWDS_HARDCAP_MODULE_ID = 64,
    /* Customer 100 to 119 */
    UNWDS_CUSTOMER_MODULE_ID = 100,
    /* System module 120 to 126 */
    UNWDS_LORAWAN_SYSTEM_MODULE_ID = 124,
    UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 125,
    UNWDS_CONFIG_MODULE_ID = 126,
} UNWDS_MODULE_IDS_t;

#endif
