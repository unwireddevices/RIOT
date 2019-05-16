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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "random.h"
#include "dsp/int_dia/int_dia.h"

#define DATA_SET_LEN      128

/**
 * @brief xxd-like printing of a binary buffer
 */
static void print_buffer(const uint8_t * buf, size_t length) {
    static const unsigned int bytes_per_line = 16;
    static const unsigned int bytes_per_group = 16;
    unsigned long i = 0;
    while (i < length) {
        unsigned int col;
        for (col = 0; col < bytes_per_line; ++col) {
            /* Print hex data */
            if (col == 0) {
                printf("\n%08" PRIx32 ": ", i);
            }
            else if ((col % bytes_per_group) == 0) {
                putchar(' ');
            }
            if ((i + col) < length) {
                printf("%02x ", buf[i + col]);
            } else {
                putchar(' ');
                putchar(' ');
            }
        }
        putchar(' ');
        for (col = 0; col < bytes_per_line; ++col) {
            if ((i + col) < length) {
                /* Echo only printable chars */
                if (isprint(buf[i + col])) {
                    putchar(buf[i + col]);
                } else {
                    putchar('.');
                }
            } else {
                putchar(' ');
            }
        }
        i += bytes_per_line;
    }
    /* end with a newline */
    puts("\n");
}

int main(void) {
    int_dia_t int_dia;
    uint32_t data_set[DATA_SET_LEN];

    random_init(0);
    int_dia_init(&int_dia);

    for (uint32_t sample = 0; sample < DATA_SET_LEN; sample++) {
        data_set[sample] = random_uint32_range(0x00000000, 0x0000FFFF);
    }
    puts("Data Set for Baseline:");
    print_buffer((uint8_t *)data_set, (DATA_SET_LEN * sizeof(uint32_t)));

    int_dia_get_baseline(&int_dia, data_set, DATA_SET_LEN);
    printf("Value baseline: %ld", int_dia.moving_avg); 
    return 0;
}
