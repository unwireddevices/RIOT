/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */
/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the LIS2HH12 accelerometer driver
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */


#include <stdio.h>

#include "xtimer.h"
#include "lis2hh12.h"
#include "lis2hh12_params.h"


#define DELAY       (5000UL * MS_PER_SEC)

/* allocate device descriptor */
static lis2hh12_t dev;

int main(void)
{
    puts("LIS2HH12 accelerometer driver test application\n");

    puts("Initializing LIS2HH12 sensor... ");
    if (lis2hh12_init(&dev, &lis2hh12_params[0], NULL, NULL) == LIS2HH12_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    xtimer_ticks32_t last_wakeup = xtimer_now();
    while (1) {

        lis2hh12_data_t acc_value;

        /* read sensor data */
        if (lis2hh12_read_xyz(&dev, &acc_value) != LIS2HH12_OK) {
            puts("error: unable to retrieve data from sensor, quitting now");
            return 1;
        }

        /* print data to STDIO */
        printf("Accelerometer [mG]:\tX: %ld\tY: %ld\tZ: %ld\n",
               acc_value.x_axis,
               acc_value.y_axis,
               acc_value.z_axis);

        int16_t temp_value;
        lis2hh12_read_temp(&dev, &temp_value);
        printf("Temperature:\t\t%dC\n", temp_value);


        xtimer_periodic_wakeup(&last_wakeup, DELAY);
    }

    return 0;
}
