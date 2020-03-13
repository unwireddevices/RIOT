/*
 * Copyright (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_common_nrf52
 * @{
 *
 * @file
 * @brief       Peripheral configuration for unwd-nrf
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
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
 *
 * @note    The radio will not work with the internal RC oscillator!
 *
 * @{
 */
#define CLOCK_HFCLK         (32U)           /* set to  0: internal RC oscillator
                                             *        32: 32MHz crystal */
#define CLOCK_LFCLK         (1)             /* set to  0: internal RC oscillator
                                             *         1: 32.768 kHz crystal
                                             *         2: derived from HFCLK */
/** @} */

#define EEPROM_SIZE     FLASHPAGE_SIZE*2

/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = 
{
    {
        .dev     = NRF_PWM0,
        .channel = { { .pin = GPIO_PIN(0, 2) },     /* DIO5 */  
                     { .pin = GPIO_PIN(0, 12) },    /* DIO6 */  
                     { .pin = GPIO_PIN(0, 13) },    /* DIO7 */  
                     { .pin = GPIO_PIN(0, 4) } }    /* DIO24 */  
    },
    {
        .dev      = NRF_PWM1,
        .channel = { { .pin = GPIO_PIN(0, 5) },     /* DIO25 */  
                     { .pin = GPIO_PIN(0, 29) },    /* DIO26 */  
                     { .pin = GPIO_UNDEF },
                     { .pin = GPIO_UNDEF } }
    },
    {
        .dev      = NRF_PWM2,
        .channel = { { .pin = GPIO_UNDEF },
                     { .pin = GPIO_UNDEF },
                     { .pin = GPIO_UNDEF },
                     { .pin = GPIO_UNDEF } }
    }
};

#define PWM_CHAN        (4)
#define PWM_NUMOF       (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name    Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      =  NRF_TIMER0,
        .channels =  3,
        .bitmode  = TIMER_BITMODE_BITMODE_32Bit,
        .irqn     = TIMER0_IRQn
    }
};

#define TIMER_0_ISR         (isr_timer0)

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name    Real time counter configuration
 * @{
 */
#define RTT_NUMOF           (1U)
#define RTT_DEV             (1)                 /* NRF_RTC1 */
#define RTT_MAX_VALUE       (0x00ffffff)        /* 24 bit */  
#define RTT_FREQUENCY       (1024)

#define LPTIMER_HZ          RTT_FREQUENCY
#define LPTIMER_MAX_VALUE   RTT_MAX_VALUE
#define LPTIMER_WIDTH       (24)
/** @} */

/**
 * @name    NFC antenna connection pins
 * @{
 */
#define NFC_NUMOF           (1U)
#define NFC_PIN_1           GPIO_PIN(0, 9)      
#define NFC_PIN_2           GPIO_PIN(0, 10)       
   
/** @} */

/**
 * @brief UART configuration
 */
static const uart_conf_t uart_config[] = {
    {
        .rx_pin   = GPIO_PIN(0, 7),
        .tx_pin   = GPIO_PIN(0, 6),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .irqn     = UARTE0_UART0_IRQn,
        .rts_pin  = (uint8_t)GPIO_UNDEF,
        .cts_pin  = (uint8_t)GPIO_UNDEF,
        .dev      = NRF_UARTE0,
    },
    {
        .rx_pin   = GPIO_PIN(0, 29),
        .tx_pin   = GPIO_PIN(0, 5),
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

/**
 * @name    SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
    {
        .dev  = NRF_SPI0,
        .sclk = GPIO_PIN(0, 2),     /* DIO5 */  
        .mosi = GPIO_PIN(0, 19),    /* DIO29 */  
        .miso = GPIO_PIN(0, 30)     /* DIO27 */  

        /* Alt SPI */
        // .sclk = GPIO_PIN(0,),    /* DIO */  
        // .mosi = GPIO_PIN(0,),    /* DIO */  
        // .miso = GPIO_PIN(0,)     /* DIO */  
    }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name    I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = NRF_TWIM0,
        .scl = GPIO_PIN(0, 20),     /* DIO30 */
        .sda = GPIO_PIN(0, 19)      /* DIO29 */
    },
    {
        .dev = NRF_TWIM1,
        .scl = GPIO_PIN(0, 13),     /* DIO7  */
        .sda = GPIO_PIN(0, 12)      /* DIO6  */
    },
};

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
