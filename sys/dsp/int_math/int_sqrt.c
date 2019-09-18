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


#include "dsp/int_math/int_sqrt.h"

uint8_t int_sqrt_8(uint8_t arg)
{
    register uint8_t root, remainder, place;

    root = 0;
    remainder = arg;
    place = 0x40;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

uint8_t int_sqrt_16(uint16_t arg)
{
    register uint16_t root, remainder, place;

    root = 0;
    remainder = arg;
    place = 0x4000;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

uint16_t int_sqrt_32(uint32_t arg)
{
    register uint32_t root, remainder, place;

    root = 0;
    remainder = arg;
    place = 0x40000000;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

