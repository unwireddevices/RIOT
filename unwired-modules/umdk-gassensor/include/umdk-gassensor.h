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
 * @author      
 */
#ifndef UMDK_GASSENSOR_H
#define UMDK_GASSENSOR_H

#include "unwds-common.h"

#define UMDK_GASSENSOR_PUBLISH_PERIOD_MIN           1

#define UMDK_GASSENSOR_STACK_SIZE                   1024

#define UMDK_GASSENSOR_ADC_LINE                     ADC_LINE(3)
#define UMDK_GASSENSOR_ADC_RESOLUTION               ADC_RES_12BIT
#define UMDK_GASSENSOR_CONVERT_TO_MILLIVOLTS        1

#define UMDK_GASSENSOR_I2C                          (I2C_DEV(1))
#define UMDK_GASSENSOR_I2C_ADDR                     (0x48)
#define UMDK_GASSENSOR_MODULE_EN_PIN                (UNWD_GPIO_5)

#define UMDK_GASSENSOR_EXT_GAIN                     (499000U)

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
} umdk_gassensor_sensor_t;


void umdk_gassensor_init(uwnds_cb_t *event_callback);
bool umdk_gassensor_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_GASSENSOR_H */
