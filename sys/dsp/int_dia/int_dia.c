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

#include <math.h>
#include <string.h>

void int_dia_main(int_dia_t *int_dia, uint32_t value)
{
    //Collect measurements
    int_dia->measure_sample = value;

    //Process Data using Derivative/Integration Algorithm
    //Moving Average using IIR filter of 8 samples
    int_dia->prev_moving_avg = int_dia->moving_avg;

    int_dia->moving_avg = ((int_dia->moving_avg << 2) - int_dia->moving_avg + int_dia->measure_sample) >> 2;

    int_dia->derivative = int_dia->moving_avg - int_dia->prev_moving_avg;

    if ((abs(int_dia->derivative) > PROX_DERIVATIVE_THRESHOLD)) {
        int_dia->integral = int_dia->prev_integral + int_dia->derivative;
    } else {
        int_dia->integral = int_dia->prev_integral;
    }

    if (int_dia->is_active) {
        int_dia->integral_hys = PROX_INTEGRAL_THRESHOLD / PROX_INTEGRAL_HYS;
    } else {
        int_dia->integral_hys = PROX_INTEGRAL_THRESHOLD * PROX_INTEGRAL_HYS;
    }

    if (int_dia->integral_hys <= -(int_dia->integral)) {
        //Object detected for Sensor
        int_dia->prev_integral = int_dia->integral;
        int_dia->is_active = true;
    } else {
        //Object not detected
        int_dia->prev_integral = int_dia->integral * LEAKAGE_FACTOR_PROX;
        int_dia->is_active = false;
    }
}

void int_dia_init(int_dia_t *int_dia)
{
    memset(int_dia, 0x00, sizeof(int_dia_t));

    int_dia->init_baseline = true;
    int_dia->is_active     = false;
}

void int_dia_get_baseline(int_dia_t *int_dia, uint32_t *data_set, uint8_t max_samples)
{
    //Collect first 128 samples for moving average to have int_dia_baseline == ~measure_sample
    for (uint8_t sample = 0; sample < max_samples; sample++)
    {
        //Collect measurements for moving average
        int_dia->measure_sample = data_set[sample];

        if (int_dia->init_baseline)
        {
            int_dia->moving_avg = int_dia->measure_sample;
            int_dia->init_baseline = false;
        }

        int_dia->moving_avg = ((int_dia->moving_avg << 2) - int_dia->moving_avg + int_dia->measure_sample) >> 2;
    }
}
