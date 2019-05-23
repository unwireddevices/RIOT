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

#include <math.h>
#include <string.h>

#include "dsp/int_dia/int_dia.h"

#define ENABLE_DEBUG            (0)
#include "debug.h"

void int_dia_main(int_dia_t *int_dia, uint32_t value)
{
    //Collect measurements
    int_dia->measure_sample = value;
    DEBUG("Measure Sample: %" PRIu32 "\n", int_dia->measure_sample);

    //Process Data using Derivative/Integration Algorithm
    //Moving Average using IIR filter
    int_dia->prev_moving_avg = int_dia->moving_avg;

    if (int_dia->with_iir) {
        int_dia->moving_avg = ((int_dia->moving_avg << 2) - int_dia->moving_avg + int_dia->measure_sample) >> 2;
    } else {
        int_dia->moving_avg = int_dia->measure_sample;
    }

    int_dia->derivative = (int32_t)int_dia->moving_avg - (int32_t)int_dia->prev_moving_avg;
    DEBUG("Derivative Value: %" PRIi32 "\n", int_dia->derivative);

    if ((abs(int_dia->derivative) > int_dia->derivative_ths)) {
        int_dia->integral = int_dia->prev_integral + int_dia->derivative;
    } else {
        int_dia->integral = int_dia->prev_integral;
    }
    DEBUG("Integral Value: %" PRIi32 "\n", int_dia->integral);

    if (int_dia->integral_ths <= (int_dia->integral)) {
        //Object detected for Sensor
        int_dia->prev_integral = int_dia->integral;
        int_dia->is_detected = true;
        DEBUG("Is detected\n");
    } else {
        //Object not detected
        int_dia->prev_integral = (float)int_dia->integral * int_dia->leakage_factor;
        int_dia->is_detected = false;
        DEBUG("Is not detected\n");
    }
}

void int_dia_init(int_dia_t *int_dia,  int32_t  integral_ths, int32_t  derivative_ths, int32_t  leakage_factor, bool with_iir)
{
    memset(int_dia, 0x00, sizeof(int_dia_t));

    int_dia->init_baseline  = true;
    int_dia->is_detected    = false;
    int_dia->with_iir       = with_iir;
    int_dia->integral_ths   = integral_ths;
    int_dia->derivative_ths = derivative_ths;
    int_dia->leakage_factor = leakage_factor;

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
    DEBUG("Baseline Value: %" PRIu32 "\n", int_dia->moving_avg);
}
