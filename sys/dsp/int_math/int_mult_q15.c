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
 * @brief       Q15 multiplication
 * 
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#include "dsp/int_math/int_mult_q15.h"

/**
 * @brief       Signed Saturate
 * @details     Saturates a signed value.
 * @param[in]   value - Value to be saturated
 * @param[in]   sat   - Bit position to saturate to (1..32)
 * @return      Saturated value
 */
static int32_t __ssat(int32_t value, uint32_t sat) {
    if ((sat >= 1U) && (sat <= 32U)) {
        const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
        const int32_t min = -1 - max;
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        }
    }
    return value;
}


int16_t int_mult_q15(int16_t a, int16_t b) {
    return (int16_t)__ssat((((int32_t)a * (int32_t)b) >> 15), 16);
}