/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @ingroup     boards_unwd-railwaycar
 * @{
 *
 * @file
 * @brief       Peripheral configuration for the unwd-railwaycar (nRF52832)
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 * @author      Oleg Artamonov
 *
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock configuration
 * @{
 */
#define CLOCK_HFCLK         (32U) /* 32 MHz crystal */
#define CLOCK_LFCLK         (1)  /* 32.786 kHz crystal */
/** @} */

/**
 * @name Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = NRF_TIMER0,
        .channels =  3,
        .bitmode  = TIMER_BITMODE_BITMODE_24Bit,
        .irqn     = TIMER0_IRQn
    },
    {
        .dev      = NRF_TIMER1,
        .channels = 3,
        .bitmode  = TIMER_BITMODE_BITMODE_16Bit,
        .irqn     = TIMER1_IRQn
    },
    {
        .dev      = NRF_TIMER2,
        .channels = 3,
        .bitmode  = TIMER_BITMODE_BITMODE_16Bit,
        .irqn     = TIMER2_IRQn
    }
};

#define TIMER_0_ISR         isr_timer0
#define TIMER_1_ISR         isr_timer1
#define TIMER_2_ISR         isr_timer2

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name    Real time counter configuration
 * @{
 */
#define RTT_NUMOF           (1U)
#define RTT_DEV             (1)             /* NRF_RTC1 */
#define RTT_MAX_VALUE       (0x00ffffff)
#define RTT_FREQUENCY       (1024)
/** @} */

#define LPTIMER_HZ          RTT_FREQUENCY
#define LPTIMER_MAX_VALUE   RTT_MAX_VALUE

/**
 * @name    NFC antenna connection pins
 * @{
 */
#define NFC_NUMOF          (1U)
#define NFC_PIN_1         GPIO_PIN(0, 9)      
#define NFC_PIN_2         GPIO_PIN(0, 10)       
   
/** @} */


/**
 * @name   I2C (TWI) configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = NRF_TWIM1,
        .scl = 6,
        .sda = 7,
        .speed   = I2C_SPEED_NORMAL,
    }
};

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

/**
 * @brief UART configuration
 */
static const uart_conf_t uart_config[] = {
    {
        .rx_pin   = GPIO_PIN(0, 23),
        .tx_pin   = GPIO_PIN(0, 22),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .irqn     = UARTE0_UART0_IRQn,
        .rts_pin  = (uint8_t)GPIO_UNDEF,
        .cts_pin  = (uint8_t)GPIO_UNDEF,
        .dev      = NRF_UARTE0,
    },
    {
        .rx_pin   = GPIO_PIN(0, 12),
        .tx_pin   = GPIO_PIN(0, 13),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .irqn     = UARTE0_UART0_IRQn,
        .rts_pin  = (uint8_t)GPIO_UNDEF,
        .cts_pin  = (uint8_t)GPIO_UNDEF,
        .dev      = NRF_UARTE0,
    },
    {
        .rx_pin   = GPIO_PIN(0, 20),
        .tx_pin   = GPIO_PIN(0, 19),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .irqn     = UARTE0_UART0_IRQn,
        .rts_pin  = (uint8_t)GPIO_UNDEF,
        .cts_pin  = (uint8_t)GPIO_UNDEF,
        .dev      = NRF_UARTE0,
    }
};

#define UART_0_ISR          isr_uart0

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
