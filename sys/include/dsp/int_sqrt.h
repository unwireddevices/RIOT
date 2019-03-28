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
 * @brief       Fixed-point square root
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef INT_SQRT_H
#define INT_SQRT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

 /**
  * @brief Computes square root of 8 bits arg.
  * 
  * @param[in] arg -  fixed point value
  * @return    square root of arg (√arg)
  */
uint8_t int_sqrt_8(uint8_t arg);

/**
 * @brief Computes square root of 16 bits arg.
 * 
 * @param[in] arg - fixed point value
 * @return    square root of arg (√arg)
 */
uint8_t int_sqrt_16(uint16_t arg);

/**
 * @brief Computes square root of 32 bits arg.
 * 
 * @param[in] arg - fixed point value
 * @return    square root of arg (√arg)
 */
uint16_t int_sqrt_32(uint32_t arg);




#ifdef __cplusplus
}
#endif

#endif /* INT_SQRT_H */

