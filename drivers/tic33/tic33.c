/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		tic33.c
 * @brief       TIC33 LCD driver module definitions
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#include "periph/gpio.h"
#include "assert.h"
#include "xtimer.h"
#include "rtctimers-millis.h"

#include "tic33.h"

static rtctimers_millis_t lclk_toggle_timer;

typedef struct {
    uint8_t symbol;
    uint8_t code;
} tic33_symbol_t;

static const tic33_symbol_t tic33_symbols[] = {
    { '0', 0b01011111 },
    { '1', 0b01010000 },
    { '2', 0b01101101 },
    { '3', 0b01111101 },
    { '4', 0b01110010 },
    { '5', 0b00111011 },
    { '6', 0b00111111 },
    { '7', 0b01010001 },
    { '8', 0b01111111 },
    { '9', 0b01111011 },
    { 'A', 0b01110111 },
    { 'b', 0b01111100 },
    { 'C', 0b00001111 },
    { 'd', 0b00111110 },
    { 'E', 0b00101111 },
    { 'F', 0b00100111 },
    { 'H', 0b01110110 },
    { 'I', 0b00010000 },
    { 'L', 0b00001110 },
    { 'O', 0b00111100 },
    { 'P', 0b01100111 },
    { 'R', 0b00100100 },
    { 'S', 0b00111011 },
    { 'U', 0b01011110 },
    { '-', 0b00100000 },
    { '_', 0b00001000 },
    { ' ', 0b00000000 },
};

static void lclk_toggle(void *arg) {
    tic33_t *dev = (tic33_t *)arg;    
    gpio_toggle(dev->lclk);
    
    rtctimers_millis_set(&lclk_toggle_timer, TIC33_LCLK_PERIOD);
}

int tic33_init(tic33_t *tic33) {
	assert(tic33 != NULL);

    gpio_init(tic33->din, GPIO_OUT);
    gpio_clear(tic33->din);
    
    gpio_init(tic33->dclk, GPIO_OUT);
    gpio_clear(tic33->dclk);
    
    gpio_init(tic33->load, GPIO_OUT);
    gpio_clear(tic33->load);
    
    gpio_init(tic33->lclk, GPIO_OUT);
    gpio_clear(tic33->lclk);
    
    if (tic33->clock_enable) {
        /* auto toggle LCLK in TIC33_LCLK_PERIOD ms */
        lclk_toggle_timer.callback = lclk_toggle;
        lclk_toggle_timer.arg = (void *)tic33;
        rtctimers_millis_set(&lclk_toggle_timer, TIC33_LCLK_PERIOD);
    }

	return 0;
}

static inline void tic33_delay(void) {
    /* wait at least 1 us */
    xtimer_spin(xtimer_ticks_from_usec(1));
}

static void tic33_shift_data(tic33_t *dev, uint8_t *data) {
    /* shift data to TIC33 */
    for (int i = 0; i < TIC33_MAX_CHARACTERS; i++) {
        for (int k = 0; k < 8; k++) {
            gpio_write(dev->din, (data[i] >> k) & 1);
            gpio_set(dev->dclk);
            
            tic33_delay();
            gpio_clear(dev->dclk);
            tic33_delay();
        }
    }
    
    gpio_set(dev->load);
    tic33_delay();
    gpio_clear(dev->load);
}

int tic33_puts(tic33_t *dev, char *str) {
    uint8_t data[TIC33_MAX_CHARACTERS] = { 0 };
    uint8_t total_symbols = sizeof(tic33_symbols)/sizeof(tic33_symbols[0]);
    
    int pos = 0;
    char symbol;
    for (int i = 0; i < 2*TIC33_MAX_CHARACTERS; i++) {
        symbol = str[i];
        
        if ((symbol == 0) || (symbol == '\n')) {
            break;
        }
        
        if (symbol == '.') {
            if (pos != 0) {
                data[pos - 1] |= 1<<7;
            } else {
                /* "." can not be the first symbol */
                return -1;
            }
            continue;
        }
        
        /* convert lowercase letters to uppercase */
        if ((symbol >= 'a') && (symbol <= 'z')) {
            symbol -= ('a' - 'A');
        }

        /* scan LUT */
        for (int k = 0; k < total_symbols; k++) {
            if (tic33_symbols[k].symbol == symbol) {
                data[pos] = tic33_symbols[k].code;
                pos++;
            }
        }
        
        /* illegal symbol */
        return -1;
    }
    
    tic33_shift_data(dev, data);
    
    return 0;
}

void tic33_clear(tic33_t *dev) {
    uint8_t data[TIC33_MAX_CHARACTERS] = { 0 };
    tic33_shift_data(dev, data);
}

void tic33_lclk_toggle(tic33_t *dev) {
    gpio_toggle(dev->lclk);
}

#ifdef __cplusplus
}
#endif
