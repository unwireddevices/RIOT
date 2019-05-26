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
 * @author      Alexander Ugorelov <info@unwds.com>
 *
 * @}
 */
#ifndef INT_DIA_H
#define INT_DIA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t measure_sample;        /**< current measured value                      */
    uint32_t moving_avg;            /**< current value of the moving average         */
    uint32_t prev_moving_avg;       /**< previous value of the moving average        */
    int32_t  derivative;            /**< current value of the differential component */
    int32_t  integral;              /**< current value of the integral component     */
    int32_t  prev_integral;         /**< previous value of the integral component    */
    int32_t  integral_ths;          /**< integration threshold                       */
    int32_t  derivative_ths;        /**< derivative threshold                        */
    float    leakage_factor;        /**< leakage_factor                              */
    bool     init_baseline;         /**< measure the baseline                        */
    bool     is_detected;           /**< object was detected                         */
    bool     with_iir;              /**< use IRR filer                               */
} int_dia_t;

/**
 * @brief Initialize the Derivative Integration Algorithm state
 * 
 * @param[in/out] int_dia          DIA structure pointer
 * @param[in]     integ_ths        integration threshold
 * @param[in]     deriv_ths        derivative threshold
 * @param[in]     leakage_factor   leakage factor
 * @param[in]     with_iir         use IRR filer
 */
void int_dia_init(int_dia_t *int_dia, int32_t  integ_ths, int32_t  deriv_ths, int32_t  leakage_factor, bool with_iir);

/**
 * @brief Gets the baseline for the Derivative Integration Algorithm
 * 
 * @param[in/out] int_dia    DIA structure pointer
 * @param[in]     data_set   raw data set
 * @param[in]     samples    number of samples of the source data array
 */
void int_dia_get_baseline(int_dia_t *int_dia, uint32_t *data_set, uint8_t samples);

/**
 * @brief Main function of the Derivative Integration Algorithm
 * 
 * @param[in/out] int_dia   DIA structure pointer
 * @param[in]     value     current measured value
 */
void int_dia_main(int_dia_t *int_dia, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* INT_SQRT_H */