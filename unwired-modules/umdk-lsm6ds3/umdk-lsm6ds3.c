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
 * @file		umdk-lsm6ds3.c
 * @brief       umdk-lsm6ds3 module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-lsm6ds3.h"
#include "include/lsm6ds3.h"

#include "thread.h"
#include "xtimer.h"

static uwnds_cb_t *callback;

static lsm6ds3_t lsm6ds3;

void umdk_lsm6ds3_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    lsm6ds3_param_t lsm_params;
    lsm_params.i2c_addr = 0x6A;
    lsm_params.i2c = UMDK_LSM6DS3_I2C;

    /* Configure the default settings */
    lsm_params.gyro_enabled = true;
    lsm_params.gyro_range = LSM6DS3_ACC_GYRO_FS_G_500dps;
    lsm_params.gyro_sample_rate = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
    lsm_params.gyro_bandwidth = LSM6DS3_ACC_GYRO_BW_XL_400Hz;
    lsm_params.gyro_fifo_enabled = true;
    lsm_params.gyro_fifo_decimation = true;

    lsm_params.accel_enabled = true;
    lsm_params.accel_odr_off = true;
    lsm_params.accel_range = LSM6DS3_ACC_GYRO_FS_XL_16g;
    lsm_params.accel_sample_rate = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
    lsm_params.accel_bandwidth = LSM6DS3_ACC_GYRO_BW_XL_400Hz;
    lsm_params.accel_fifo_enabled = true;
    lsm_params.accel_fifo_decimation = true;

    lsm_params.temp_enabled = true;

    lsm_params.comm_mode = 1;

    lsm_params.fifo_threshold = 3000;
    lsm_params.fifo_sample_rate = LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
    lsm_params.fifo_mode_word = 0;

    lsm6ds3.params = lsm_params;
    
    if (lsm6ds3_init(&lsm6ds3) < 0) {
        puts("[umdk-lsm6ds3] LSM6DS3 initialization failed");
    }
}

bool umdk_lsm6ds3_cmd(module_data_t *cmd, module_data_t *reply)
{
	/* Check for empty command */
	if (cmd->length < 1)
		return false;

	umdk_lsm6ds3_cmd_t c = cmd->data[0];

	switch (c) {
	case UMDK_LSM6DS3_CMD_POLL:
	{
		lsm6ds3_data_t acc_data = {};
        lsm6ds3_get_raw(&lsm6ds3, &acc_data);
		uint16_t temp = lsm6ds3_read_temp_c(&lsm6ds3);
		
		reply->length = 1 + sizeof(lsm6ds3_data_t) + 2;
		reply->data[0] = UNWDS_LSM6DS3_MODULE_ID;
		
		memcpy(reply->data + 1, &acc_data, sizeof(lsm6ds3_data_t));
		memcpy(reply->data + 1 + sizeof(lsm6ds3_data_t), &temp, 2);
		
		break;
	}
	default:
		return false;
	}
	
    return true;
}

#ifdef __cplusplus
}
#endif
