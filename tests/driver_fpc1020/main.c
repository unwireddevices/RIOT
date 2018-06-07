/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the FPC1020A fingerprint sensor
 *
 * @author      Oleg Artamonov <info@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "fpc1020.h"
#include "xtimer.h"

// static uint8_t fpc1020_image_buffer[FPC1020_BUFFER_MAX_IMAGES * FPC1020_MAX_IMAGE_SIZE];
static uint8_t fpc1020_image_buffer[24000];

int main(void)
{
    xtimer_init();
    
    fpc1020_t fpc1020;
    
    /* buffer must be initialized before calling fpc1020_init */
    fpc1020.image = fpc1020_image_buffer;
    
    /* device, SPI bus, CS, reset, IRQ */
    fpc1020_init(&fpc1020, 0, UNWD_GPIO_29, UNWD_GPIO_24, UNWD_GPIO_25);
    
    return 0;
}
