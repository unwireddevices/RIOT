/*
 * Copyright (C) 2015 Eistec AB
 *               2017 Freie Universität Berlin
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
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "xtimer.h"
#include "lis3dh.h"
#include "lis3dh_params.h"


#define SLEEP       (500 * 1000U)

lis3dh_t dev;
lis3dh_acceleration_t acc_data;
// int16_t temperature = 0x00;

static void test_int1(void *arg)
{
    (void)arg;
    gpio_irq_disable(dev.params.int1);
    puts("INT1 callback");
    if (lis3dh_read_xyz(&dev, &acc_data) != 0) {
        puts("Reading acceleration data... ");
        puts("[Failed]\n");

    }
    gpio_irq_enable(dev.params.int1);
}

int main(void)
{   

    lis3dh_params_t lis3dh_params[] = { 
        {.i2c      = I2C_DEV(0),   
        .addr      = LIS3DH_I2C_SAD_L,    
        .int1      = GPIO_PIN(PORT_A, 14),  
        .int1_mode = I1_ZYXDA,  
        .scale     = LIS3DH_2g,
        .odr       = LIS3DH_ODR_25Hz,
        .op_mode   = LIS3DH_HR_12bit},

    };

    gpio_init(GPIO_PIN(PORT_B, 0), GPIO_OUT);

    puts("LIS3DH accelerometer driver test application");

    puts("LIS3DH timeout for boot device > 5ms");
    xtimer_usleep(10 * 1000);

    puts("Initializing LIS3DH sensor... ");
    if (lis3dh_init(&dev, &lis3dh_params[0], test_int1, NULL) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    puts("LIS3DH init done.\n");

    puts("LIS3DH power off.\n");
    lis3dh_power_off(&dev);
    xtimer_usleep(SLEEP);

    puts("LIS3DH power on.\n");
    lis3dh_power_on(&dev);

    while (1) {
 
        
        gpio_set(GPIO_PIN(PORT_B, 0));
        xtimer_usleep(SLEEP);
        if (dev.params.int1 == GPIO_UNDEF) {
            printf("Reading measurements\n");
            if (lis3dh_read_xyz(&dev, &acc_data) != 0) {
                puts("Reading acceleration data... ");
                puts("[Failed]\n");
                return 1;
            }
        }
        printf("X: %d Y: %d Z: %d \n", acc_data.axis_x, acc_data.axis_y, acc_data.axis_z);
        gpio_clear(GPIO_PIN(PORT_B, 0));
        xtimer_usleep(SLEEP);
        
    }

    return 0;
}
