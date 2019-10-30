/*
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the LIS3DH accelerometer driver
 *
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "xtimer.h"

#include "lis3dh.h"
#include "lis3dh_params.h"

lis3dh_t dev;
lis3dh_data_t acc_data;
int16_t deg_celsius;

static void _int1_handler(void *arg)
{
    (void)arg;

    gpio_irq_disable(dev.params.int1);

    if (lis3dh_read_xyz(&dev, &acc_data) == 0) {
        /* print values */
        printf("X: %d[milli-G] Y: %d[milli-G] Z: %d[milli-G]\n", acc_data.axis_x, acc_data.axis_y, acc_data.axis_z);
    } else {
        puts("Reading acceleration data... ");
        puts("[Failed]\n");
    }

    if (lis3dh_read_temp(&dev, &deg_celsius) == 0) {
        /* print values */
        printf("Temperature %d Celsius\n", deg_celsius);
    } else {
         puts("Reading temperature... ");
        puts("[Failed]\n");
    }
    gpio_irq_enable(dev.params.int1);
}

int main(void)
{   
    lis3dh_params_t lis3dh_params[] = { 
        {.i2c_dev  = I2C_DEV(0),   
        .i2c_addr  = LIS3DH_I2C_SAD_L,    
        .int1      = GPIO_PIN(PORT_A, 14),  
        .int1_mode = I1_ZYXDA,  
        .scale     = LIS3DH_SCALE_2G,
        .odr       = LIS3DH_ODR_1HZ,
        .res       = LIS3DH_HR_12BIT},
    };

    puts("LIS3DH accelerometer driver test application");

    puts("Initializing LIS3DH sensor... ");
    if (lis3dh_init(&dev, &lis3dh_params[0], _int1_handler, NULL) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }
    puts("LIS3DH init done.\n");

    puts("LIS3DH power on...");
    if (lis3dh_poweron(&dev) != 0) {
        puts("[Failed]\n");
        return 1;
    }
    xtimer_usleep(500 * 1000U);

    while (1) {     
        //do Nothing
    }

    return 0;
}
