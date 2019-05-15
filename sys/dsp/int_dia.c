/*
 * Copyright (C) 2016-2019 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     sys_dsp
 * @{
 *
 * @file
 * @brief       Fixed-point Derivative Integration Algorithm
 * @details     Application Report SNOA939
 *              DavidWang
 *              Derivative Integration Algorithm for Proximity Sensing
 *              September 2015
 *              Texas Instruments
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include "dsp/int_dia.h"

int int_dia_main(void)
{
    //Collect FDC measurements
    if (data_ready)
    {
        measure_sample = read_measure();
        data_ready = false;

        //Process Data using Derivative/Integration Algorithm
        //Moving Average using IIR filter of 8 samples
        prev_moving_avg = moving_avg;

        moving_avg = ((moving_avg << 2) - moving_avg + measure_sample) >> 2;

        derivative = moving_avg - prev_moving_avg;

        if ((abs(derivative) > PROX_DERIVATIVE_THRESHOLD)) {
            integral = prev_integral + derivative;
        } else {
            integral = prev_integral;
        }

        if (is_active) {
            integral_hys = PROX_INTEGRAL_THRESHOLD / PROX_INTEGRAL_HYS;
        } else {
            integral_hys = PROX_INTEGRAL_THRESHOLD * PROX_INTEGRAL_HYS;
        }

        if (integral_hys <= -(integralProx)) {
            //Object detected for Proximity Sensor
            prev_integral = integral;
            is_active = true;
        } else {
            //Object not detected
            prev_integral = integral * LEAKAGE_FACTOR_PROX;
            is_active = false;
        }

        //Clear DRDY bit by reading STATUS REGISTER
        clear_drdy();
    }
}

void int_dia_init(int_dia_t *int_dia)
{
    memset(int_dia, 0x00, sizeof(int_dia_t));

    int_dia->data_ready = false;
    int_dia->init_baseline = true;
    int_dia->is_active = false;
}

void int_dia_get_baseline(int_dia_t *int_dia, uint8_t max_samples)
    //Collect first 128 samples for moving average to have int_dia_baseline == ~measure_sample
    for (uint8_t samples = 0; samples < max_samples; samples++)
    {
        //Wait until data d'ont ready
        while (!data_ready) {
            // do Nothing
        }

        //Collect measurements for moving average
        measure_sample = read_measure();
        data_ready = false;

        if (data_ready)
        {
            moving_avg = measure_sample;
            init_baseline = false;
        }

        moving_avg = ((moving_avg << 2) - moving_avg + measure_sample) >> 2;
        clear_drdy();
    }
}
