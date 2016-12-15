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
 * @file		umdk-acc.c
 * @brief       umdk-acc module implementation
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
#include "umdk-acc.h"

#include "lsm6ds3.c"

#include "thread.h"
#include "xtimer.h"

static uwnds_cb_t *callback;

static lsm6ds3_t lsm6ds3;

void umdk_acc_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    lsm6ds3_param_t lsm_params;
    lsm_params.i2c_addr = 0x6A;
    lsm_params.i2c = UMDK_ACC_I2C;

    if (lsm6ds3_init(&lsm6ds3, &lsm_params) < 0) {
        puts("[umdk-acc] Initialization of LSM6DS3 failed");
    }

    /* Configure the default settings */
    lsm_params.gyro_enabled = true;
    lsm_params.gyro_range = 2000;
    lsm_params.gyro_sample_rate = 416;
    lsm_params.gyro_bandwidth = 400;
    lsm_params.gyro_fifo_enabled = true;
    lsm_params.gyro_fifo_decimation = true;

    lsm_params.accel_enabled = true;
    lsm_params.accel_odr_off = true;
    lsm_params.accel_range = 8;
    lsm_params.accel_sample_rate = 416;
    lsm_params.accel_bandwidth = 400;
    lsm_params.accel_fifo_enabled = true;
    lsm_params.accel_fifo_decimation = true;

    lsm_params.temp_enabled = true;

    lsm_params.comm_mode = 1;

    lsm_params.fifo_threshold = 3000;
    lsm_params.fifo_sample_rate = 10;
    lsm_params.fifo_mode_word = 0;

    if (!lsm6ds3_configure(&lsm6ds3, &lsm_params)) {
        puts("[umdk-acc] Configuration of LSM6DS3 failed");
    }
}

bool umdk_acc_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply)
{
    if (strcmp(argv[1], "get") == 0) {
        char buf[UNWDS_MAX_REPLY_LEN] = { '\0' };

        lsm6ds3_data_t data = {};

        lsm6ds3_get_raw(&lsm6ds3, &data);

        float_t temp = lsm6ds3_read_temp_c(&lsm6ds3);

        snprintf(buf, UNWDS_MAX_REPLY_LEN, "{ax:%.2f,ay:%.2f,az:%.2f,gx:%.2f,gy:%.2f,gz:%.2f,temp:%.2f}",
                 data.acc_x, data.acc_y, data.acc_z,
                 data.gyr_x, data.gyr_y, data.gyr_z,
                 temp);

        strcpy(reply, buf);

        return true;
    }

    strcpy(reply, "{error:true, text:\"invalid params\"}");

    return false;
}

#ifdef __cplusplus
}
#endif
