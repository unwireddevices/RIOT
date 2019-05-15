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
#ifndef INT_DIA_H
#define INT_DIA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PROX_INTEGRAL_THRESHOLD         200
#define PROX_INTEGRAL_HYS               1.2
#define PROX_DERIVATIVE_THRESHOLD       0
#define LEAKAGE_FACTOR_PROX             0.99

typedef struct {
    uint32_t measure_sample;
    uint32_t moving_avg;
    uint32_t prev_moving_avg;
    int32_t  derivative;
    int32_t  integral;
    int32_t  prev_integral;
    uint16_t integral_hys;
    bool data_ready;// = false;
    bool init_baseline;// = true;
    bool is_active;// = false;

    //TODO: Insert a pointer to the prototype of the measurement reading function
    //TODO: Insert a pointer to the prototype of the clear drdy flag function
} int_dia_t;


void int_dia_init(uint8_t max_samples);
int int_dia_main(void);

#ifdef __cplusplus
}
#endif

#endif /* INT_SQRT_H */