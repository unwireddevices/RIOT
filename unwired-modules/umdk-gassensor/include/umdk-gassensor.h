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
 * @file        umdk-gassensor.h
 * @brief       LMP91000-based gas sensor module
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */
#ifndef UMDK_GASSENSOR_H
#define UMDK_GASSENSOR_H

#include "unwds-common.h"

#define UMDK_GASSENSOR_PUBLISH_PERIOD_MIN           1

#define UMDK_GASSENSOR_STACK_SIZE                   1024

#define UMDK_GASSENSOR_ADC_LINE                     ADC_LINE(3)
#define UMDK_GASSENSOR_ADC_RESOLUTION               ADC_RES_12BIT

#define UMDK_GASSENSOR_I2C                          (I2C_DEV(1))
#define UMDK_GASSENSOR_I2C_ADDR                     (0x48)
#define UMDK_GASSENSOR_MODULE_EN_PIN                (UNWD_GPIO_5)

/* Presettings LMP91000 */
#define UMDK_GASSENSOR_EXT_GAIN                     (499000U)           //Ohm
#define UMDK_LMP_TEMP_OFFSET                        (1560)              //mV
#define UMDK_LMP_TEMP_SENSITIVITY                   (8200)              //C/uV

/* Recommended Bias */
#define UMDK_GASSENSOR_ME2CO_BIAS                   (0)                 //mV
#define UMDK_GASSENSOR_CO_BIAS                      (20.5)              //mV
#define UMDK_GASSENSOR_H2S_BIAS                     (0)                 //mV
#define UMDK_GASSENSOR_NO2_BIAS                     (205)               //mV
#define UMDK_GASSENSOR_SO2_BIAS                     (205)               //mV
#define UMDK_GASSENSOR_O3_BIAS                      (20.5)              //mV

#if !defined(UMDK_GASSENSOR_VOLTAGE_REF)
#define UMDK_GASSENSOR_VOLTAGE_REF                  (3300)              //mV
#endif

#if !defined(UMDK_GASSENSOR_REF_SOURCE)
#define UMDK_GASSENSOR_REF_SOURCE                   (LMP91000_REF_SOURCE_INT)
#endif

#if !defined(UMDK_GASSENSOR_BIAS_SIGN)
#define UMDK_GASSENSOR_BIAS_SIGN                    (LMP91000_BIAS_SIGN_NEG)
#endif

#if !defined(UMDK_GASSENSOR_INT_Z)
#define UMDK_GASSENSOR_INT_Z                        (LMP91000_INT_Z_50PCT)
#endif

#if !defined(UMDK_GASSENSOR_RLOAD)
#define UMDK_GASSENSOR_RLOAD                        (LMP91000_RLOAD_10OHM)
#endif

#if !defined(UMDK_GASSENSOR_MODE)
#define UMDK_GASSENSOR_MODE                         (LMP91000_OP_MODE_3_LEAD_AMP_CELL)
#endif

#if !defined(UMDK_GASSENSOR_FET)
#define UMDK_GASSENSOR_FET                          (LMP91000_FET_SHORT_DISABLED)
#endif

/* Temperature Coefficient of Span [%/°C] * 10 */
#define UMDK_GASSENSOR_CO_SPAN_HI                   (3)                 //  10°C to 40 °C
#define UMDK_GASSENSOR_CO_SPAN_LO                   (9)                 // -20°C to 10 °C
/* Temperature Coefficient of Span [%/°C] * 100 */
#define UMDK_GASSENSOR_H2S_SPAN_HI                  (5)                 //  20 °C to 40 °C
#define UMDK_GASSENSOR_H2S_SPAN_LO                  (-33)               // -20 °C to 20 °C
/* Temperature Coefficient of Span [%/°C] * 10 */
#define UMDK_GASSENSOR_NO2_SPAN                     (3)                 // -20 °C to 50 °C
/* Temperature Coefficient of Span [%/°C] * 100 */
#define UMDK_GASSENSOR_SO2_SPAN_HI                  (26)                // 20 °C to 40 °C
#define UMDK_GASSENSOR_SO2_SPAN_LO                  (-33)               // -20 °C to 20 °C
/* Temperature Coefficient of Span [%/°C] * 10 */
#define UMDK_GASSENSOR_O3_SPAN                      (3)                 // -20 °C to 50 °C


#define GASSENSOR_PARAMS_BOARD                         \
    {                                                  \
        .i2c           = UMDK_GASSENSOR_I2C,           \
        .i2c_addr      = UMDK_GASSENSOR_I2C_ADDR,      \
        .module_en_pin = UMDK_GASSENSOR_MODULE_EN_PIN  \
    }

typedef enum {
    UMDK_GASSENSOR_DATA        = 0,
    UMDK_GASSENSOR_CMD_COMMAND = 1,
    UMDK_GASSENSOR_CMD_POLL    = 2,
    UMDK_GASSENSOR_FAIL        = 0xFF,
} umdk_gassensor_cmd_t;

typedef enum {
    UMDK_GASSENSOR_UNKNOWN = 0,
    UMDK_GASSENSOR_CO      = 1,
    UMDK_GASSENSOR_H2S     = 2,
    UMDK_GASSENSOR_NO2     = 3,
    UMDK_GASSENSOR_SO2     = 4,
    UMDK_GASSENSOR_O3      = 5,
    UMDK_GASSENSOR_ME2CO   = 6,
} umdk_gassensor_sensor_t;


void umdk_gassensor_init(uwnds_cb_t *event_callback);
bool umdk_gassensor_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_GASSENSOR_H */
