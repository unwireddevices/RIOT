/*
 * Copyright (C) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l4-r3
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Unwired Range L4 board
 *
 * @author      Mikhail Churikov
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @author      Alexander Ugorelov <ugorelovan@yandex.ru>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock system configuration
 * @{
 */
/* 0: no external high speed crystal available
 * else: actual crystal frequency [in Hz] */
#define CLOCK_HSE           (0)//(24000000)
/* 0: no external low speed crystal available,
 * 1: external crystal available (always 32.768kHz) */
#define CLOCK_LSE           (32768)
/* 0: enable MSI only if HSE isn't available
 * 1: always enable MSI (e.g. if USB or RNG is used)*/
#define CLOCK_MSI_ENABLE    (1)

#ifndef CLOCK_MSI_LSE_PLL
/* 0: disable Hardware auto calibration with LSE
 * 1: enable Hardware auto calibration with LSE (PLL-mode)
 * Same as with CLOCK_LSE above this defaults to 0 because LSE is
 * mandatory for MSI/LSE-trimming to work */
#define CLOCK_MSI_LSE_PLL   (0)
#endif

/* give the target core clock (HCLK) frequency [in Hz], maximum: 80MHz */
#define CLOCK_CORECLOCK     (80000000U)
/* PLL configuration: make sure your values are legit!
 *
 * compute by: CORECLOCK = (((PLL_IN / M) * N) / R)
 * with:
 * PLL_IN:  input clock, HSE or MSI @ 48MHz
 * M:       pre-divider,  allowed range: [1:8]
 * N:       multiplier,   allowed range: [8:86]
 * R:       post-divider, allowed range: [2,4,6,8]
 *
 * Also the following constraints need to be met:
 * (PLL_IN / M)     -> [4MHz:16MHz]
 * (PLL_IN / M) * N -> [64MHz:344MHz]
 * CORECLOCK        -> 80MHz MAX!
 */
#define CLOCK_PLL_M         (6)
#define CLOCK_PLL_N         (20)
#define CLOCK_PLL_R         (2)
/* peripheral clock setup */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV4
#define CLOCK_APB1          (CLOCK_CORECLOCK / 4)
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV2
#define CLOCK_APB2          (CLOCK_CORECLOCK / 2)
/** @} */

/**
 * @name    Timer configuration
 * @{
 */
static const dac_conf_t dac_config[] = {
    { .pin = GPIO_PIN(PORT_A,  4), .chan = 0 }
};

#define DAC_NUMOF           (sizeof(dac_config) / sizeof(dac_config[0]))
/** @} */

/**
 * @brief Timer configuration
 * @{
 */
#define TIMER_0_MAX_VALUE  0xffff
 
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM15,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB2ENR_TIM15EN,
        .bus      = APB2,
        .irqn     = TIM1_BRK_TIM15_IRQn
    }
};

#define TIMER_0_ISR         isr_tim1_brk_tim15

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))

/**
 * @name    UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev      = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin   = GPIO_PIN(PORT_A, 10),
        .tx_pin   = GPIO_PIN(PORT_A, 9),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB2,
        .irqn     = USART1_IRQn
    },
    {
        .dev      = USART2,
        .rcc_mask = RCC_APB1ENR1_USART2EN,
        .rx_pin   = GPIO_PIN(PORT_A, 3),
        .tx_pin   = GPIO_PIN(PORT_A, 2),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB1,
        .irqn     = USART2_IRQn
    },
    {
        .dev      = USART3,
        .rcc_mask = RCC_APB1ENR1_USART3EN ,
        .rx_pin   = GPIO_PIN(PORT_B, 11),
        .tx_pin   = GPIO_PIN(PORT_B, 10),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB1,
        .irqn     = USART3_IRQn
    } 
};

#define UART_0_ISR          (isr_usart1)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */


/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM3,
        .rcc_mask = RCC_APB1ENR1_TIM3EN,
        .chan     = { { .pin = GPIO_PIN(PORT_B, 4), .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    },
};

#define TIM_0_ISR           isr_tim3

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name    SPI configuration
 *
 * @note    The spi_divtable is auto-generated from
 *          `cpu/stm32_common/dist/spi_divtable/spi_divtable.c`
 * @{
 */
static const uint8_t spi_divtable[2][5] = {
    {       /* for APB1 @ 20000000Hz */
        7,  /* -> 78125Hz */
        5,  /* -> 312500Hz */
        3,  /* -> 1250000Hz */
        1,  /* -> 5000000Hz */
        0   /* -> 10000000Hz */
    },
    {       /* for APB2 @ 40000000Hz */
        7,  /* -> 156250Hz */
        6,  /* -> 312500Hz */
        4,  /* -> 1250000Hz */
        2,  /* -> 5000000Hz */
        1   /* -> 10000000Hz */
    }
};

static const spi_conf_t spi_config[] = {
    {
        .dev      = SPI1,
        .mosi_pin = GPIO_PIN(PORT_A, 7),
        .miso_pin = GPIO_PIN(PORT_A, 6),
        .sclk_pin = GPIO_PIN(PORT_A, 5),
        .cs_pin   = GPIO_UNDEF,
        .af       = GPIO_AF5,
        .rccmask  = RCC_APB2ENR_SPI1EN,
        .apbbus   = APB2
    },
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name    ADC configuration
 * @{
 */
//#define I2C_IRQ_PRIO        CPU_DEFAULT_IRQ_PRIO
#define I2C_APBCLK          (CLOCK_APB1)

/* I2C 0 device configuration */
#define I2C_0_EVT_ISR       isr_i2c1_ev
#define I2C_0_ERR_ISR       isr_i2c1_er

/* I2C 1 device configuration */
#define I2C_1_EVT_ISR       isr_i2c2_ev
#define I2C_1_ERR_ISR       isr_i2c2_er

static const i2c_conf_t i2c_config[] = {
    {
        .dev            = I2C1,
        .speed          = I2C_SPEED_NORMAL,
        .scl        = GPIO_PIN(PORT_B,  8),
        .sda        = GPIO_PIN(PORT_B,  9),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR1_I2C1EN,
        .irqn           = I2C1_ER_IRQn
    },
    {
        .dev            = I2C2,
        .speed          = I2C_SPEED_NORMAL,
        .scl        = GPIO_PIN(PORT_B, 10),
        .sda        = GPIO_PIN(PORT_B, 11),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR1_I2C2EN,
        .irqn           = I2C2_ER_IRQn
    }
};

#define I2C_0_ISR           isr_i2c1_er
#define I2C_1_ISR           isr_i2c2_er

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))

/** @} */

/**
 * @name    RTT configuration
 *
 * On the STM32Lx platforms, we always utilize the LPTIM1.
 * @{
 */
#define ADC_CONFIG {            \
    { GPIO_PIN(PORT_A, 1), 0, 6 },\
    { GPIO_PIN(PORT_A, 2), 0, 7 },\
    { GPIO_PIN(PORT_A, 3), 0, 8 },\
    { GPIO_PIN(PORT_A, 4), 0, 9 },\
    { GPIO_PIN(PORT_A, 5), 0, 10 },\
    { GPIO_PIN(PORT_A, 6), 0, 11 }, \
    { GPIO_PIN(PORT_A, 7), 0, 12 }, \
    { GPIO_UNDEF, 0, ADC_VREF_CHANNEL}, \
    { GPIO_UNDEF, 0, ADC_TEMPERATURE_CHANNEL}, \
    { GPIO_UNDEF, 0, ADC_VBAT_CHANNEL}, \
}

#define ADC_VREF_INDEX          7
#define ADC_TEMPERATURE_INDEX   8
#define ADC_VBAT_INDEX          9

#define ADC_NUMOF               (10)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */