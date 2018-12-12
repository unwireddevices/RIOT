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


#define SLEEP       (1000 * 1000U)

#define WATERMARK_LEVEL 16

// static volatile int int1_count = 0;

// static void test_int1(void *arg)
// {
//     volatile int *int1_count_ptr = arg;
//     ++(*int1_count_ptr);
// }

int main(void)
{   

    lis3dh_params_t lis3dh_params[] = { 
        {.i2c      = I2C_DEV(0),   
        .addr      = LIS3DH_I2C_SAD_L,    
        .int1      = GPIO_UNDEF,  
        .int1_mode = I1_DISABLE,  
        .scale     = LIS3DH_2g,
        .odr       = LIS3DH_ODR_1Hz,
        .op_mode   = LIS3DH_HR_12bit},

    };

    lis3dh_t dev;
    lis3dh_acceleration_t acc_data;

    puts("LIS3DH accelerometer driver test application\n");

    puts("Initializing LIS3DH sensor... ");
    if (lis3dh_init(&dev, &lis3dh_params[0]) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    // puts("Enable streaming FIFO mode... ");
    // if (lis3dh_set_fifo(&dev, LIS3DH_FIFO_MODE_STREAM, WATERMARK_LEVEL) == 0) {
    //     puts("[OK]");
    // }
    // else {
    //     puts("[Failed]\n");
    //     return 1;
    // }

    // puts("Enable temperature reading... ");
    // if (lis3dh_set_aux_adc(&dev, 1, 1) == 0) {
    //     puts("[OK]");
    // }
    // else {
    //     puts("[Failed]\n");
    //     return 1;
    // }

    // puts("Set INT1 watermark function... ");
    // if (lis3dh_set_int1(&dev, LIS3DH_CTRL_REG3_I1_WTM_MASK) == 0) {
    //     puts("[OK]");
    // }
    // else {
    //     puts("[Failed]\n");
    //     return 1;
    // }

    // puts("Set INT1 callback");
    // if (gpio_init_int(lis3dh_params[0].int1, GPIO_IN, GPIO_RISING,
    //                   test_int1, (void*)&int1_count) == 0) {
    //     puts("[OK]");
    // }
    // else {
    //     puts("[Failed]\n");
    //     return 1;
    // }

    puts("LIS3DH init done.\n");

    while (1) {
 
        printf("Reading measurements\n");
        if (lis3dh_read_xyz(&dev, &acc_data) != 0) {
                puts("Reading acceleration data... ");
                puts("[Failed]\n");
            }
            // if (lis3dh_read_aux_adc3(&dev, &temperature) != 0) {
            //     puts("Reading temperature data... ");
            //     puts("[Failed]\n");
            //     return 1;
            // }
            // int1 = gpio_read(lis3dh_params[0].int1);
            printf("X: %ld Y: %ld Z: %ld \n", acc_data.axis_x, acc_data.axis_y, acc_data.axis_z);

        xtimer_usleep(SLEEP);
    }

    return 0;
}
