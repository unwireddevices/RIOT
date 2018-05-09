/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
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
 * @brief       Test application for the BME280 temperature, pressure
 *              and humidity sensor.
 *
 * @author      Kees Bakker <kees@sodaq.com>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "fpc1020.h"
#include "xtimer.h"

int main(void)
{
    fpc1020_t fpc1020;
    
    fpc1020_init(&fpc1020, 0, GPIO_PIN(PORT_B, 0), GPIO_PIN(PORT_B, 1), GPIO_PIN(PORT_B, 2));
    
    return 0;
}
