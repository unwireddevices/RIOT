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
#ifndef INT_MULT_Q15_H
#define INT_MULT_Q15_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dsp/dsp_type.h"
    
/**
 * @brief           Q15 multiplication numbers
 * @param[in]       a - first input number
 * @param[in]       b - second input number
 * @return          output numbers
 *
 * <b>Scaling and Overflow Behavior:</b>
 * \par
 * The function uses saturating arithmetic.
 * Results outside of the allowable Q15 range [0x8000 0x7FFF] will be saturated.
 */
q15_t int_mult_q15(int16_t a, int16_t b);


#ifdef __cplusplus
}
#endif

#endif /* INT_MULT_Q15_H */

