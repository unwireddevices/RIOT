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
 * @brief       Test application for the TCA6408A I/O Expander driver
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <stdbool.h>

#include "xtimer.h"
#include "tca6408a.h"
#include "tca6408a_params.h"

#define DELAY_US            (1000U * 1000U)

int main(void)
{
    tca6408a_t tca6408a;


    puts("TCA6408A I/O Expander driver test application\n");
    printf("Initializing TCA6408A at I2C_%i ... ",tca6408a_params[0].i2c_dev);
    if (tca6408a_init(&tca6408a,  &tca6408a_params[0]) != TCA6408A_OK) {
        puts("[ERROR]\n");
        return -1;
    }
    puts("[SUCCESS]\n");

    for (uint8_t pin_num = 0; pin_num < 8; pin_num++) {
        printf("Initializing pin %d as output ...", pin_num);
        if (tca6408a_write_config(&tca6408a, pin_num, TCA6408A_PIN_OUTPUT) == TCA6408A_OK) {
            puts("[SUCCESS]\n");
        } else {
            puts("[ERROR]\n");
            return -1;
        }
    }
    

    while (1) {
        
        for(uint8_t pin_num = 0; pin_num < 8; pin_num++) {
            tca6408a_write_output(&tca6408a, pin_num, true);
            xtimer_usleep(DELAY_US);
        }
        for(uint8_t pin_num = 0; pin_num < 8; pin_num++) {
            tca6408a_write_output(&tca6408a, pin_num, false);
            xtimer_usleep(DELAY_US);
        }
        
        
    }

    return 0;
}
