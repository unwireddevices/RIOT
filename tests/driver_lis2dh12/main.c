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
#include "lis2dh12_params.h"

/* Allocate some of the memory for the device descriptor */
static lis2dh12_t dev;

/* Allocate some of the memory to store the output values of the sensor */
static lis2dh12_acc_t acc;
static int16_t deg_celsius;

int main(void)
{
    puts("LIS2DH12 accelerometer driver test application\n");

    puts("Initializing LIS2DH12 sensor... ");
    if (lis2dh12_init(&dev, &lis2dh12_params[0]) == LIS2DH12_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    puts("LIS2DH12 power on...");
    if (lis2dh12_power_on(&dev, dev.params.scale, dev.params.rate, dev.params.res) == LIS2DH12_OK) {
        puts("[OK]");
    } else {
        puts("[Failed]\n");
        return 1;
    }

    while (1) {
        /* read sensor data */
        if (lis2dh12_read_xyz(&dev, &acc) == LIS2DH12_OK) {
            /* print acc values */
            printf("X: %d[milli-G] Y: %d[milli-G] Z: %d[milli-G]\n", acc.axis_x, acc.axis_y, acc.axis_z);
        } else {
            puts("[Error] unable to retrieve data from sensor, quitting now");
            return 1;
        }

        if (lis2dh12_read_temp(&dev, &deg_celsius) == LIS2DH12_OK) {
            /* print temp values */
            printf("Temperature %d Celsius\n", deg_celsius);
        } else {
            puts("[Error] unable to retrieve data from sensor, quitting now");
            return 1;
        }
    }

    return 0;
}
