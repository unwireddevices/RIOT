/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the LIS2DH12 accelerometer driver
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 *
 * @}
 */

#include <stdio.h>

#include "lis2dh12.h"
#include "lptimer.h"

#define SLEEP_DELAY_MS 500

/* Allocate some of the memory for the device descriptor */
static lis2dh12_t lis2dh12;
static lis2dh12_params_t lis2dh12_params;

/* Allocate some of the memory to store the output values of the sensor */
static lis2dh12_data_t acc;
static int16_t deg_celsius;

int main(void)
{
    puts("LIS2DH12 accelerometer driver test application\n");
    /* Setting parameters accelerometer */
    lis2dh12_params.i2c_dev  = I2C_DEV(0);
    lis2dh12_params.i2c_addr = LIS2DH12_I2C_SAD_L;
    lis2dh12_params.scale    = LIS2DH12_SCALE_2G;
    lis2dh12_params.odr      = LIS2DH12_ODR_1HZ;
    lis2dh12_params.res      = LIS2DH12_HR_12BIT;

    puts("Initializing LIS2DH12 sensor... ");

    if (lis2dh12_init(&lis2dh12, &lis2dh12_params) == LIS2DH12_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    while (1) {
        lis2dh12_poweron(&lis2dh12);

        /* read sensor data */
        if (lis2dh12_read_xyz(&lis2dh12, &acc) == LIS2DH12_OK) {
            /* print acc values */
            printf("X: %d[milli-G] Y: %d[milli-G] Z: %d[milli-G]\n", acc.axis_x, acc.axis_y, acc.axis_z);
        } else {
            puts("[Error] unable to retrieve data from sensor, quitting now");
            return 1;
        }

        if (lis2dh12_read_temp(&lis2dh12, &deg_celsius) == LIS2DH12_OK) {
            /* print temp values */
            printf("Temperature %d Celsius\n", deg_celsius);
        } else {
            puts("[Error] unable to retrieve data from sensor, quitting now");
            return 1;
        }

        lis2dh12_poweroff(&lis2dh12);

        lptimer_sleep(SLEEP_DELAY_MS);
    }

    return 0;
}
