/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    unwd-soilsensor-f070
 * @ingroup     boards
 * @brief       Soil moisture sensor board, based on STM32F070F6P6
 * @{
 *
 * @file
 * @brief       Soil moisture sensor board, based on STM32F070F6P6
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
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
 */
#define CLOCK_HSE           (24000000U)         /* external oscillator */
#define CLOCK_CORECLOCK     (48000000U)          /* desired core clock frequency */

/* the actual PLL values are automatically generated */
#define CLOCK_PLL_MUL       (2)
#define CLOCK_PLL_DIV       (1)

/* bus clocks for simplified peripheral initialization, UPDATE MANUALLY! */
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB2          (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1          (CLOCK_CORECLOCK / 1)

/* configuration of peripheral bus clock prescalers */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE_DIV1
/** @} */

/**
 * @brief   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM3,
        .max      = 0x0000ffff,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    }
};

#define TIMER_0_ISR         isr_tim3

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @brief   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev        = USART1,
        .rcc_mask   = RCC_APB2ENR_USART1EN,
        .rx_pin     = GPIO_PIN(PORT_A, 10),
        .tx_pin     = GPIO_PIN(PORT_A, 9),
        .tx_mode    = GPIO_OUT,
        .rx_mode    = GPIO_IN,
        .rx_af      = GPIO_AF1,
        .tx_af      = GPIO_AF1,
        .bus        = APB2,
        .irqn       = USART1_IRQn
    },
    {
        .dev        = USART2,
        .rcc_mask   = RCC_APB1ENR_USART2EN,
        .rx_pin     = GPIO_PIN(PORT_A, 3),
        .tx_pin     = GPIO_PIN(PORT_A, 2),
        .tx_mode    = GPIO_OD,
        .rx_mode    = GPIO_IN_PU,
        .rx_af      = GPIO_AF1,
        .tx_af      = GPIO_AF1,
        .bus        = APB1,
        .irqn       = USART2_IRQn
    }
};

#define UART_0_ISR          (isr_usart1)
#define UART_1_ISR          (isr_usart2)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM14,
        .rcc_mask = RCC_APB1ENR_TIM14EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF4,
        .bus      = APB1,
        .irqn     = TIM14_IRQn
    },
    {
        .dev      = TIM17,
        .rcc_mask = RCC_APB2ENR_TIM17EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF5,
        .bus      = APB2,
        .irqn     = TIM17_IRQn
    }
};

#define TIM_0_ISR           isr_tim14
#define TIM_1_ISR           isr_tim17

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @brief   ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    { .pin = GPIO_PIN(PORT_A, 0), .chan = 1,                       /* .trigger = ADC_EXT_TRIGGER_TIM9TRGO */ },
    { .pin = GPIO_PIN(PORT_A, 1), .chan = 2,                       /* .trigger = ADC_EXT_TRIGGER_TIM9TRGO */ },
	{ .pin = GPIO_UNDEF,          .chan = 17,                      /* .trigger = ADC_EXT_TRIGGER_TIM9TRGO */ },
};

#define ADC_NUMOF           (sizeof(adc_config) / sizeof(adc_config[0]))

/** @} */

/**
 * @brief   DAC configuration
 * @{
 */
#define DAC_NUMOF           (0)
/** @} */

/**
 * @name RTC configuration
 * @{
 */
/**
 * Nucleos with MB1136 C-02 or MB1136 C-03 -sticker on it have the required LSE
 * oscillator provided on the X2 slot.
 * See Nucleo User Manual UM1724 section 5.6.2.
 */
#define RTC_NUMOF           (0U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
