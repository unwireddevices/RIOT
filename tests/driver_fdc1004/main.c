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
 * @brief       Test application for the FDC1004 4-Channel Capacitance-to-Digital Converter
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 *
 * @}
 */


#include <stdio.h>

#include "xtimer.h"
#include "rtctimers-millis.h"
#include "fdc1004.h"
#include "dsp/int_dia/int_dia.h"


#define DELAY                           (100UL * US_PER_MS)
#define FDC1004_I2C                     (1)
#define FDC1004_CH                      (1)
#define FDC1004_INIT_DATASET_LEN        (128)

#define FDC1004_INTEGRAL_THRESHOLD      7000
#define FDC1004_DERIVATIVE_THRESHOLD    0
#define FDC1004_LEAKAGE_FACTOR          0.99
#define FDC1004_IIR_FILTER              false  

/* allocate device descriptor */
static fdc1004_t dev;
static int_dia_t int_dia;


int main(void)
{
    uint32_t data_set[FDC1004_INIT_DATASET_LEN];

    puts("FDC1004 4-Channel Capacitance-to-Digital Converter driver test application\n");

    dev.i2c = FDC1004_I2C;

    rtctimers_millis_init();
    /* Initialized DIA structure */
    int_dia_init(&int_dia, FDC1004_INTEGRAL_THRESHOLD, FDC1004_DERIVATIVE_THRESHOLD, FDC1004_LEAKAGE_FACTOR, FDC1004_IIR_FILTER);


    printf("Initializing FDC1004 sensor on I2C #%d... ", dev.i2c);
    if (fdc1004_init(&dev) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    /* initialized baseline for DIA */
    if (int_dia.init_baseline) {
        for (int i = 0; i < FDC1004_INIT_DATASET_LEN; i++) {
            data_set[i] = fdc1004_get_raw_data(&dev, FDC1004_CH);
        }
        int_dia_get_baseline(&int_dia, data_set, FDC1004_INIT_DATASET_LEN);
    }

    xtimer_ticks32_t last_wakeup = xtimer_now();

    printf("Sample\tMov_AVG\tMeasure\tDerivative\tIntegral\tWas detected?\n");
    while (1) {

        uint32_t capacitance = 0;
        uint32_t sample = 0;

        /* read sensor data */
        capacitance = fdc1004_get_raw_data(&dev, FDC1004_CH);
        /* print data to STDIO */
        // printf("Capacitance channel %d [F]: %" PRIu32 "\n", FDC1004_CH, capacitance);
        int_dia_main(&int_dia, capacitance);
        /* print data to STDIO */
        printf("%" PRIu32 "\t""%" PRIu32 "\t""%" PRIu32 "\t""%" PRIi32 "\t""%" PRIi32 "\t", 
               sample++, int_dia.moving_avg, capacitance, int_dia.derivative, int_dia.integral);
        printf(/*"Proximity seinsing:*/" %s\n", (int_dia.is_detected == true)?("detected"):("not detected"));

        xtimer_periodic_wakeup(&last_wakeup, DELAY);
    }

    return 0;
}
