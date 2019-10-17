/*
 * Copyright (C) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l0
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Unwired Range L0 board
 *
 * @author      Oleg Artamonov
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Clock system configuration
 * @{
 **/
#define CLOCK_LSE           (32768)                 /* external low-speed crystal frequency  */
#define CLOCK_HSI           (16000000U)             /* internal high-speed crystal frequency */
#define CLOCK_CORECLOCK     (32000000U)             /* targeted core clock frequency */
/* configuration of PLL prescaler and multiply values */
/* CORECLOCK := HSI / CLOCK_PLL_DIV * CLOCK_PLL_MUL */
#define CLOCK_PLL_DIV       RCC_CFGR_PLLDIV2
#define CLOCK_PLL_MUL       RCC_CFGR_PLLMUL4

/* configuration of peripheral bus clock prescalers */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1      /* AHB clock -> 32MHz */
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV1     /* APB2 clock -> 32MHz */
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV1     /* APB1 clock -> 32MHz */
/* configuration of flash access cycles */
#define CLOCK_FLASH_LATENCY FLASH_ACR_LATENCY
/** @} */

/* bus clocks for simplified peripheral initialization, UPDATE MANUALLY! */
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB2          (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1          (CLOCK_CORECLOCK / 1)
/** @} */

/**
 * @brief Timer configuration
 * @{
 */
#define TIMER_0_MAX_VALUE   (0x0000ffff)
 
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM2,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn
    },
    {
        .dev      = TIM6,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB1ENR_TIM6EN,
        .bus      = APB1,
        .irqn     = TIM6_IRQn
    }
};

#define TIMER_0_ISR         isr_tim2
#define TIMER_1_ISR         isr_tim6

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name Real time counter configuration
 * @{
 */
#define RTC_NUMOF           (0U)

/**
 * @name Basic RTT emulation on top of RTC with 1024 Hz frequency
 * @{
 */
#define RTT_FREQUENCY       (1024)
#define RTT_MAX_VALUE       (0x7ffful)

#define LPTIMER_HZ          RTT_FREQUENCY
#define LPTIMER_MAX_VALUE   RTT_MAX_VALUE
#define LPTIMER_WIDTH       (15)

/**
 * @brief UART configuration
 */
static const uart_conf_t uart_config[] = {
    {
        .dev      = USART2,
        .rcc_mask = RCC_APB1ENR_USART2EN,
        .rx_pin   = GPIO_PIN(PORT_A, 3),
        .tx_pin   = GPIO_PIN(PORT_A, 2),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF4,
        .tx_af    = GPIO_AF4,
        .bus      = APB1,
        .irqn     = USART2_IRQn
    },
    {
        .dev      = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin   = GPIO_PIN(PORT_A, 10),
        .tx_pin   = GPIO_PIN(PORT_A, 9),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF4,
        .tx_af    = GPIO_AF4,
        .bus      = APB2,
        .irqn     = USART1_IRQn
    },
    {
        .dev      = USART4,
        .rcc_mask = RCC_APB1ENR_USART4EN,
        .rx_pin   = GPIO_PIN(PORT_A, 1),
        .tx_pin   = GPIO_PIN(PORT_A, 0),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF6,
        .tx_af    = GPIO_AF6,
        .bus      = APB1,
        .irqn     = USART4_5_IRQn
    }
};

#define UART_0_ISR          (isr_usart2)
#define UART_1_ISR          (isr_usart1)
#define UART_2_ISR          (isr_usart4)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM3,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 6)         , .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_A, 7)         , .cc_chan = 1 },
                      { .pin = GPIO_PIN(PORT_C, 8)         , .cc_chan = 2 },
                      { .pin = GPIO_UNDEF,                   .cc_chan = 0 }
                    },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    },
};

#define TIM_0_ISR           isr_tim3

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name   SPI configuration
 *
 * @note    The spi_divtable is auto-generated from
 *          `cpu/stm32_common/dist/spi_divtable/spi_divtable.c`
 * @{
 */
static const uint8_t spi_divtable[2][5] = {
    {       /* for APB1 @ 32000000Hz */
        7,  /* -> 125000Hz */
        5,  /* -> 500000Hz */
        4,  /* -> 1000000Hz */
        2,  /* -> 4000000Hz */
        1   /* -> 8000000Hz */
    },
    {       /* for APB2 @ 32000000Hz */
        7,  /* -> 125000Hz */
        5,  /* -> 500000Hz */
        4,  /* -> 1000000Hz */
        2,  /* -> 4000000Hz */
        1   /* -> 8000000Hz */
    }
};

static const spi_conf_t spi_config[] = {
    {
        .dev      = SPI1,
        .mosi_pin = GPIO_PIN(PORT_A, 7),
        .miso_pin = GPIO_PIN(PORT_A, 6),
        .sclk_pin = GPIO_PIN(PORT_A, 5),
        .cs_pin   = GPIO_UNDEF,
        .af       = GPIO_AF0,
        .rccmask  = RCC_APB2ENR_SPI1EN,
        .apbbus   = APB2
    }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name I2C configuration
  * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev            = I2C2,
        .speed          = I2C_SPEED_NORMAL,
        .scl            = GPIO_PIN(PORT_B, 13),
        .sda            = GPIO_PIN(PORT_B, 14),
        .scl_af         = GPIO_AF5,
        .sda_af         = GPIO_AF5,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C2EN,
        .irqn           = I2C2_IRQn
    },
    {
        .dev            = I2C1,
        .speed          = I2C_SPEED_NORMAL,
        .scl            = GPIO_PIN(PORT_B,  6),
        .sda            = GPIO_PIN(PORT_B,  7),
        .scl_af         = GPIO_AF1,
        .sda_af         = GPIO_AF1,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .irqn           = I2C1_IRQn
    },
};

#define I2C_0_ISR           isr_i2c2_er
#define I2C_1_ISR           isr_i2c1_er

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

/**
 * @brief   ADC configuration
 *
 * We need to configure the following values:
 * [ pin, channel ]
 * @{
 */
static const adc_conf_t adc_config[] = {
    { .pin = GPIO_PIN(PORT_A, 1), .chan = 1,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_A, 2), .chan = 2,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_A, 3), .chan = 3,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_A, 4), .chan = 4,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_A, 5), .chan = 5,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_A, 6), .chan = 6,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
	{ .pin = GPIO_PIN(PORT_A, 7), .chan = 7,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_B, 0), .chan = 8,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
    { .pin = GPIO_PIN(PORT_B, 1), .chan = 9,                       .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
	{ .pin = GPIO_UNDEF,          .chan = ADC_VREF_CHANNEL,        .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
	{ .pin = GPIO_UNDEF,          .chan = ADC_TEMPERATURE_CHANNEL, .trigger = ADC_EXT_TRIGGER_TIM6TRGO },
};

#define ADC_VREF_INDEX          9
#define ADC_TEMPERATURE_INDEX   10

#define ADC_NUMOF           (sizeof(adc_config) / sizeof(adc_config[0]))
/** @} */
#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
