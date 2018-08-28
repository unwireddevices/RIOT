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
 * @brief       Test application for the M24SR NFC memory driver
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include <stdio.h>

#include "board.h"

#include "xtimer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "adxl345.h"
#include "adxl345_params.h"

/* allocate device descriptor */
//static m24sr_t dev;

/* shortcuts for I2C bus parameters */

#define DEV_I2C            (I2C_DEV(1))
#define DEV_ADDR           (ADXL345_ADDR_53)



int main(void)
{
    int status = 0x0;
    uint8_t _data[] = {0x00};

    puts("M24SR NFC memory driver test application\n");
    gpio_init(LED_GREEN, GPIO_OUT);

    // puts("Initializing M24SR memory... ");
    // if (m24sr_init(&dev, &lis2hh12_params[0]) == M24SR_OK) {
    //     puts("[OK]");
    // }
    // else {
    //     puts("[Failed]\n");
    //     return 1;
    // }

    // xtimer_ticks32_t last_wakeup = xtimer_now();
    while (1) {
        // status = _m24sr_release_token();


        status = i2c_write_bytes(DEV_I2C, DEV_ADDR, _data, 0, 0);

        printf("status is %d\n", status);
        gpio_toggle(LED_GREEN);

        xtimer_usleep(500000);
    }

    return 0;
}