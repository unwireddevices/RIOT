/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       Semtech SX128X ranging correction type
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef __SX128X_RANGING_CORRECT_TYPE_H__
#define __SX128X_RANGING_CORRECT_TYPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUMBER_OF_FACTORS_PER_SFBW                      (160)
#define MAX_POLYNOME_ORDER                              (10)

typedef struct {
    const uint8_t order;
    const double coefficients[MAX_POLYNOME_ORDER];
} sx128x_ranging_correction_polynomes_t;

#ifdef __cplusplus
}
#endif

#endif /* __SX128X_RANGING_CORRECT_TYPE_H__ */