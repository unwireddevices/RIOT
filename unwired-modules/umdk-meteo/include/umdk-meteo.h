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
 * @file		umdk-meteo.h
 * @brief       umdk-meteo driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_METEO_H
#define UMDK_METEO_H

#include "unwds-common.h"

#define UMDK_METEO_STACK_SIZE 1024

#define UMDK_METEO_PUBLISH_PERIOD_MIN 1

#define UMDK_METEO_I2C 1
#define UMDK_METEO_I2C_ADDR        (0x76)
#define METEO_PARAMS_BOARD               \
    {                                      \
        .i2c_dev = UMDK_METEO_I2C,   \
        .i2c_addr = UMDK_METEO_I2C_ADDR, \
        .t_sb = BMX280_SB_0_5,             \
        .filter = BMX280_FILTER_OFF,       \
        .run_mode = BMX280_MODE_FORCED,     \
        .temp_oversample = BMX280_OSRS_X1,  \
        .press_oversample = BMX280_OSRS_X1, \
        .humid_oversample = BMX280_OSRS_X1, \
    }

typedef enum {
    UMDK_METEO_DATA = 0,
	UMDK_METEO_COMMAND = 1,
	UMDK_METEO_POLL = 2,
    UMDK_METEO_FAIL = 0xFF,
} umdk_meteo_cmd_t;

void umdk_meteo_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_meteo_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_METEO_H */
