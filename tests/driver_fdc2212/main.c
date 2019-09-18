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
 * @brief       Test application for the FDC2212 2-Channel Capacitance-to-Digital Converter
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 *
 * @}
 */


#include <stdio.h>

#include "lptimer.h"
#include "fdc2212.h"
#include "fdc2212_params.h"
#include "dsp/int_dia/int_dia.h"


#define DELAY                                           (100UL * US_PER_MS)
#define FDC2212_I2C_DEV                                 (I2C_DEV(0))
#define FDC2212_I2C_ADDR                                (FDC2212_CAP_I2C_ADDR_L)
#define FDC2212_INIT_DATASET_LEN                        (128)

#define FDC2212_INTEGRAL_THRESHOLD                      (200)
#define FDC2212_DERIVATIVE_THRESHOLD                    (0)
#define FDC2212_LEAKAGE_FACTOR                          (0.99)
#define FDC2212_IIR_FILTER                              (false)

#define PROXIMITY_REFCOUNT                              (40000)
#define PROXIMITY_SETTLECOUNT                           (100)

/* allocate devices descriptor */
static fdc2212_t dev;
static int_dia_t int_dia;



int main(void)
{
    uint32_t data_set[FDC2212_INIT_DATASET_LEN];

    puts("FDC2212 2-Channel Capacitance-to-Digital Converter driver test application\n");

    dev.params.i2c_dev  = FDC2212_I2C_DEV;
    dev.params.i2c_addr = FDC2212_I2C_ADDR;

    dev.ref_count[0]    = PROXIMITY_REFCOUNT;
    dev.ref_count[1]    =  0x00;
    dev.settle_count[0] = PROXIMITY_SETTLECOUNT;
    dev.settle_count[0] = 0x00;
    dev.freq_in_sel[0]  = 0x02;
    dev.freq_in_sel[1]  = 0x02;
    dev.freq_divider[0] = 0x01;
    dev.freq_divider[1] = 0x01;
    dev.idrive[0]       = FDC2212_IDRIVE_0P069;
    dev.idrive[1]       = FDC2212_IDRIVE_0P016;

    /* Initialized DIA structure */
    int_dia_init(&int_dia, FDC2212_INTEGRAL_THRESHOLD, FDC2212_DERIVATIVE_THRESHOLD, FDC2212_LEAKAGE_FACTOR, FDC2212_IIR_FILTER);

    printf("Initializing FDC2212 sensor on I2C #%d... ", dev.params.i2c_dev);
    if (fdc2212_init(&dev, &fdc2212_params[0]) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    /* initialized baseline for DIA */
    if (int_dia.init_baseline) {
        for (int i = 0; i < FDC2212_INIT_DATASET_LEN; i++) {
            while (fdc2212_data_ready(&dev) != FDC2212_OK)
            {
                //do Nothing;
            }
            fdc2212_read_raw_data(&dev, 0, &data_set[i]);
        }
        int_dia_get_baseline(&int_dia, data_set, FDC2212_INIT_DATASET_LEN);
    }

    lptimer_ticks32_t last_wakeup = lptimer_now();

    printf("Sample\tMov_AVG\tMeasure\tDerivative\tIntegral\tWas detected?\n");
    while (1) {

        uint32_t capacitance = 0;
        uint32_t sample = 0;

        /* read sensor data */
        while (fdc2212_data_ready(&dev) != FDC2212_OK)
        {
            //do Nothing;
        }
        fdc2212_read_raw_data(&dev, 0, &capacitance);
        /* print data to STDIO */
        // printf("Capacitance channel %d [F]: %" PRIu32 "\n", FDC1004_CH, capacitance);
        int_dia_main(&int_dia, capacitance);
        /* print data to STDIO */
        printf("%" PRIu32 "\t""%" PRIu32 "\t""%" PRIu32 "\t""%" PRIi32 "\t""%" PRIi32 "\t", 
               sample++, int_dia.moving_avg, capacitance, int_dia.derivative, int_dia.integral);
        printf(/*"Proximity seinsing:*/" %s\n", (int_dia.is_detected == true)?("detected"):("not detected"));

        lptimer_periodic_wakeup(&last_wakeup, DELAY);
    }

    return 0;
}
