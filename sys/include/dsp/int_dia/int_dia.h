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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t measure_sample;        /**< */
    uint32_t moving_avg;            /**< */
    uint32_t prev_moving_avg;       /**< */
    int32_t  derivative;            /**< */
    int32_t  integral;              /**< */
    int32_t  prev_integral;         /**< */
    int32_t  integral_ths;          /**< */
    int32_t  derivative_ths;        /**< */
    float    leakage_factor;        /**< */
    bool     init_baseline;         /**< */
    bool     is_detected;             /**< */
    bool     with_iir;              /**< */
} int_dia_t;

/**
 * @brief 
 * 
 * @param int_dia 
 * @param integral_ths 
 * @param derivative_ths 
 * @param leakage_factor 
 * @param with_iir 
 */
void int_dia_init(int_dia_t *int_dia, int32_t  integral_ths, int32_t  derivative_ths, int32_t  leakage_factor, bool with_iir);

/**
 * @brief 
 * 
 * @param int_dia 
 * @param data_set 
 * @param max_samples 
 */
void int_dia_get_baseline(int_dia_t *int_dia, uint32_t *data_set, uint8_t max_samples);

/**
 * @brief 
 * 
 * @param int_dia 
 * @param value 
 * @return int 
 */
void int_dia_main(int_dia_t *int_dia, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* INT_SQRT_H */