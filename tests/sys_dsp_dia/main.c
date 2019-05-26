/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Short test application for 
 *              fixed point Derivative/Integration Algorithm
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "dsp/int_dia/int_dia.h"
#include "dataset.h"

#define PROX_INTEGRAL_THRESHOLD         7000
#define PROX_DERIVATIVE_THRESHOLD       0
#define LEAKAGE_FACTOR_PROX             0.99
#define NELEMS(x)                       (sizeof(x) / sizeof((x)[0]))

int main(void) {
    int_dia_t int_dia;
    uint32_t size_data_set = NELEMS(data_set);
    uint32_t sample = 0;
    uint32_t capacitance = 0;

    puts("Test application for Derivative/Integration Algorithm");
    int_dia_get_baseline(&int_dia, data_set, size_data_set);
    printf("Baseline value is ");
    printf("%"PRIu32"\n", int_dia.moving_avg);

    printf("Sample\tMov_AVG\tMeasure\tDerivative\tIntegral\tWas detected?\n");
    while (sample < size_data_set) {
         capacitance = data_set[sample];
        int_dia_main(&int_dia, capacitance);
        /* print data to STDIO */
        printf("%"PRIu32"\t""%"PRIu32"\t""%"PRIu32"\t""%"PRIi32"\t""%"PRIi32"\t", 
               sample++, int_dia.moving_avg, capacitance, int_dia.derivative, int_dia.integral);
        printf(" %s\n", (int_dia.is_detected == true)?("detected"):("not detected"));
    }

    return 0;
}
