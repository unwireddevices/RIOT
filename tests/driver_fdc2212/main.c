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
 * @todo        Attention! Works only with channel null.
 *
 * @}
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "lptimer.h"
#include "fdc2212.h"

#define DELAY                                           (1UL * 1000UL)

#define PROXIMITY_REFCOUNT                              (40000)
#define PROXIMITY_SETTLECOUNT                           (100)

#define PROXIMITY_INTEGRAL_THRESHOLD                    (200)
#define PROXIMITY_INTEGRAL_HYS                          (1.2)
#define PROXIMITY_DERIVATIVE_THRESHOLD                  (0)
#define PROXIMITY_LEAKAGE_FACTOR                        (0.99)

/* allocate memory for variables */
static uint32_t prox_meas_sample     = 0;
static uint32_t moving_avg_prox      = 0;
static uint32_t prev_moving_avg_prox = 0;
static int32_t  derivative_prox      = 0;
static int32_t  integral_prox        = 0;
static int32_t  prev_integral_prox   = 0;

static uint16_t integral_prox_hys = 0;

static bool init_cal = true;
static bool prox_on = false;

static void _init_moving_avg(void);

/* allocate devices descriptor */
static fdc2212_t fdc2212;
static fdc2212_params_t fdc2212_params;

int main(void)
{
    puts("FDC2212 2-Channel Capacitance-to-Digital Converter driver test application\n");
    puts("Attention! Works only with channel null!");

    /* Initialize and configure FDC2212 for only one channel number 0 */
    fdc2212_params.i2c_dev      = I2C_DEV(0);
    fdc2212_params.i2c_addr     = FDC2212_CAP_I2C_ADDR_L;
    fdc2212_params.shutdown_pin = GPIO_UNDEF;

    fdc2212.ref_count[0]    = PROXIMITY_REFCOUNT;
    fdc2212.settle_count[0] = PROXIMITY_SETTLECOUNT;
    fdc2212.freq_in_sel[0]  = 0x02;
    fdc2212.freq_divider[0] = 0x01;
    fdc2212.idrive[0]       = FDC2212_IDRIVE_0P264; /* Sensor drive current: to ensure that the oscillation amplitude is between 1.2V and 1.8V    */
                                                    /* if IDRIVE is equal FDC2212_IDRIVE_0P264 then V(pk) is 1.6V (Datasheet fdc2212: 10.2.3.2.2) */
    printf("Initializing FDC2212 sensor on I2C #%d... ", fdc2212_params.i2c_dev);
    if (fdc2212_init(&fdc2212, &fdc2212_params) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]\n");
        return 1;
    }

    /* Initialize and stabilize moving average variables  */
    _init_moving_avg();
    printf("The calculated baseline is equal to %" PRIu32 "\n", moving_avg_prox);

    lptimer_ticks32_t last_wakeup = lptimer_now();

    while (1) {
        /* Waiting until the data is ready */
        while (fdc2212_data_ready(&fdc2212) != FDC2212_OK)
        {
            //do Nothing;
        }
        /* Collect FDC measurements */
        fdc2212_read_raw_data(&fdc2212, 0, &prox_meas_sample);

        /* Process Data using Derivative/Integration Algorithm
         * Moving Average using IIR filter of 8 samples */
        prev_moving_avg_prox = moving_avg_prox;
        moving_avg_prox = ((moving_avg_prox << 2) - moving_avg_prox + prox_meas_sample) >> 2;

        derivative_prox = moving_avg_prox - prev_moving_avg_prox;

        /* Channel 0 is used as a proximity sensor */
        if((abs(derivative_prox) > PROXIMITY_DERIVATIVE_THRESHOLD)) {
            integral_prox = prev_integral_prox + derivative_prox;
        } else {
            integral_prox = prev_integral_prox;
        }

        if(prox_on) {
            integral_prox_hys = PROXIMITY_INTEGRAL_THRESHOLD / PROXIMITY_INTEGRAL_HYS;
        } else {
            integral_prox_hys = PROXIMITY_INTEGRAL_THRESHOLD * PROXIMITY_INTEGRAL_HYS;
        }

        if(integral_prox_hys <= -(integral_prox))
        {
            /* Object detected for Proximity Sensor */
            prev_integral_prox = integral_prox;
            prox_on = true;
        }
        else
        {
            /* Object not detected */
            prev_integral_prox = integral_prox * PROXIMITY_LEAKAGE_FACTOR;
            prox_on = false;
        }
        printf("Mov_AVG\tMeasure\tDerivative\tIntegral\n");
        printf("%"PRIu32"\t""%"PRIu32"\t""%"PRIi32"\t""%"PRIi32"\t ", 
               moving_avg_prox, prox_meas_sample, derivative_prox, prev_integral_prox);
        printf("The proximity sensor %s the object.\n", (prox_on == true)?("detected"):("didn't detect"));

        lptimer_periodic_wakeup(&last_wakeup, DELAY);
    }

    return 0;
}

static void _init_moving_avg(void)
{
    uint8_t samples = 0;

    /* Collect first 128 samples for moving average to have movingAvgProx == ~proxMeasSample */
    for(samples = 0; samples < 128; samples++)
    {
        /* Waiting until the data is ready */
        while (fdc2212_data_ready(&fdc2212) != FDC2212_OK)
        {
            //do Nothing;
        }

        /* Collect measurements for moving average */
        fdc2212_read_raw_data(&fdc2212, 0, &prox_meas_sample);

        if(init_cal) {
            moving_avg_prox = prox_meas_sample;
            init_cal = false;
        }

        moving_avg_prox = ((moving_avg_prox << 2) - moving_avg_prox + prox_meas_sample) >> 2;
    }
}