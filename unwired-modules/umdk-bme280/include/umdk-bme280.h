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
 * @file		umdk-bme280.h
 * @brief       umdk-bme280 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_BME280_H
#define UMDK_BME280_H

#include "unwds-common.h"

#define UMDK_BME280_STACK_SIZE 1024

#define UMDK_BME280_PUBLISH_PERIOD_MIN 1

#define UMDK_BME280_I2C 1
#define UMDK_BME280_I2C_ADDR        (0x76)
#define BME280_PARAMS_BOARD               \
    {                                      \
        .i2c_dev = UMDK_BME280_I2C,   \
        .i2c_addr = UMDK_BME280_I2C_ADDR, \
        .t_sb = BMX280_SB_0_5,             \
        .filter = BMX280_FILTER_OFF,       \
        .run_mode = BMX280_MODE_FORCED,     \
        .temp_oversample = BMX280_OSRS_X1,  \
        .press_oversample = BMX280_OSRS_X1, \
        .humid_oversample = BMX280_OSRS_X1, \
    }

typedef enum {
	UMDK_BME280_CMD_SET_PERIOD = 0,
	UMDK_BME280_CMD_POLL = 1,
	UMDK_BME280_CMD_SET_I2C = 2,
} umdk_bme280_cmd_t;

void umdk_bme280_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_bme280_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_BME280_H */
