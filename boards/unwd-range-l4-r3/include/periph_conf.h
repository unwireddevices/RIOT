/*
 * Copyright (C) 2016-2018 Unwired Devices <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l1-r3
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Unwired Range R170115 board
 *
 * @author      Mikhail Churikov
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
/* 0: no external high speed crystal available
 * else: actual crystal frequency [in Hz] */
#define CLOCK_HSE           (0)//(24000000)
/* 0: no external low speed crystal available,
 * 1: external crystal available (always 32.768kHz) */
#define CLOCK_LSE           (32768)
/* 0: enable MSI only if HSE isn't available
 * 1: always enable MSI (e.g. if USB or RNG is used)*/
#define CLOCK_MSI_ENABLE    (1)
/* 0: disable Hardware auto calibration with LSE
 * 1: enable Hardware auto calibration with LSE (PLL-mode)*/
#define CLOCK_MSI_LSE_PLL   (1)
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
 * @name   DAC configuration
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
#define TIMER_0_MAX_VALUE   (0xffffffff)
 
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM2,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB1ENR1_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn
    }
};

#define TIMER_0_ISR         (isr_tim2)

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))

/**
 * @name Real time counter configuration
 * @{
 */
#define RTC_NUMOF           (1U)

/* STM32 backup registers in use */

#define RTC_REGBACKUP_BOOTLOADER        (0)
#define RTC_REGBACKUP_BOOTMODE          (0)
#define RTC_REGBACKUP_UNWDSMODULE       (1)

#define RTC_REGBACKUP_BOOTLOADER_VALUE  (0xB00710AD)



/**
 * @name UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    // {
    //     .dev      = USART1,
    //     .rcc_mask = RCC_APB2ENR_USART1EN,
    //     .rx_pin   = GPIO_PIN(PORT_A, 10),
    //     .tx_pin   = GPIO_PIN(PORT_A, 9),
    //     .rx_mode  = GPIO_IN_PU,
    //     .tx_mode  = GPIO_OUT,
    //     .rx_af    = GPIO_AF7,
    //     .tx_af    = GPIO_AF7,
    //     .bus      = APB2,
    //     .irqn     = USART1_IRQn
    // },
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
#define UART_1_ISR          (isr_usart2)
#define UART_2_ISR          (isr_usart3)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

// /**
//  * @brief GPIO configuration
//  */
// #define GPIO_0_EN           1
// #define GPIO_1_EN           1
// #define GPIO_2_EN           1
// #define GPIO_3_EN           1
// #define GPIO_4_EN           1
// #define GPIO_5_EN           1
// #define GPIO_6_EN           1
// #define GPIO_7_EN           1
// #define GPIO_8_EN           1
// #define GPIO_9_EN           1
// #define GPIO_10_EN          1
// #define GPIO_11_EN          1
// #define GPIO_12_EN          1
// #define GPIO_13_EN          1
// #define GPIO_14_EN          1
// #define GPIO_15_EN          1
// #define GPIO_IRQ_PRIO       CPU_DEFAULT_IRQ_PRIO

// /* IRQ config */
// #define GPIO_IRQ_0          GPIO_13
// #define GPIO_IRQ_1          GPIO_14
// #define GPIO_IRQ_2          GPIO_7
// #define GPIO_IRQ_3          GPIO_0
// #define GPIO_IRQ_4          GPIO_5
// #define GPIO_IRQ_5          GPIO_12
// #define GPIO_IRQ_6          GPIO_11
// #define GPIO_IRQ_7          GPIO_1
// #define GPIO_IRQ_8          GPIO_3
// #define GPIO_IRQ_9          GPIO_2
// #define GPIO_IRQ_10         GPIO_4
// #define GPIO_IRQ_11         GPIO_6
// #define GPIO_IRQ_12         GPIO_15
// #define GPIO_IRQ_13         GPIO_8
// #define GPIO_IRQ_14         GPIO_9
// #define GPIO_IRQ_15         GPIO_10

// /* GPIO channel 0 config */
// #define GPIO_0_PORT         GPIOA                   /* Used for user button 1 */
// #define GPIO_0_PIN          3
// #define GPIO_0_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_0_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA)
// #define GPIO_0_IRQ          EXTI3_IRQn
// /* GPIO channel 1 config */
// #define GPIO_1_PORT         GPIOC
// #define GPIO_1_PIN          7
// #define GPIO_1_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOCEN))
// #define GPIO_1_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PC)
// #define GPIO_1_IRQ          EXTI9_5_IRQn
// /* GPIO channel 2 config */
// #define GPIO_2_PORT         GPIOA
// #define GPIO_2_PIN          9
// #define GPIO_2_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_2_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PA)
// #define GPIO_2_IRQ          EXTI9_5_IRQn
// /* GPIO channel 3 config */
// #define GPIO_3_PORT         GPIOA
// #define GPIO_3_PIN          8
// #define GPIO_3_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_3_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA)
// #define GPIO_3_IRQ          EXTI9_5_IRQn
// /* GPIO channel 4 config */
// #define GPIO_4_PORT         GPIOB
// #define GPIO_4_PIN          10
// #define GPIO_4_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOBEN))
// #define GPIO_4_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB)
// #define GPIO_4_IRQ          EXTI15_10_IRQn
// /* GPIO channel 5 config */
// #define GPIO_5_PORT         GPIOB
// #define GPIO_5_PIN          4
// #define GPIO_5_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOBEN))
// #define GPIO_5_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB)
// #define GPIO_5_IRQ          EXTI4_IRQn
// /* GPIO channel 6 config */
// #define GPIO_6_PORT         GPIOC
// #define GPIO_6_PIN          11
// #define GPIO_6_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOCEN))
// #define GPIO_6_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC)
// #define GPIO_6_IRQ          EXTI15_10_IRQn
// /* GPIO channel 7 config */
// #define GPIO_7_PORT         GPIOC
// #define GPIO_7_PIN          2
// #define GPIO_7_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOCEN))
// #define GPIO_7_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC)
// #define GPIO_7_IRQ          EXTI2_IRQn
// /* GPIO channel 8 config */
// #define GPIO_8_PORT         GPIOA
// #define GPIO_8_PIN          13
// #define GPIO_8_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_8_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PA)
// #define GPIO_8_IRQ          EXTI15_10_IRQn
// /* GPIO channel 9 config */
// #define GPIO_9_PORT         GPIOA
// #define GPIO_9_PIN          14
// #define GPIO_9_CLKEN()      (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_9_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PA)
// #define GPIO_9_IRQ          EXTI15_10_IRQn
// /* GPIO channel 10 config */
// #define GPIO_10_PORT        GPIOA
// #define GPIO_10_PIN         15
// #define GPIO_10_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_10_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA)
// #define GPIO_10_IRQ         EXTI15_10_IRQn
// /* GPIO channel 11 config */
// #define GPIO_11_PORT        GPIOB   /* SPI CS Pin */
// #define GPIO_11_PIN         6
// #define GPIO_11_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOBEN))
// #define GPIO_11_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB)
// #define GPIO_11_IRQ         EXTI9_5_IRQn
// /* GPIO channel 12 config */
// #define GPIO_12_PORT        GPIOC
// #define GPIO_12_PIN         5
// #define GPIO_12_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOCEN))
// #define GPIO_12_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC)
// #define GPIO_12_IRQ         EXTI9_5_IRQn
// /* GPIO channel 13 config */
// #define GPIO_13_PORT        GPIOA
// #define GPIO_13_PIN         0
// #define GPIO_13_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_13_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA)
// #define GPIO_13_IRQ         EXTI0_IRQn
// /* GPIO channel 14 config */
// #define GPIO_14_PORT        GPIOA
// #define GPIO_14_PIN         1
// #define GPIO_14_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOAEN))
// #define GPIO_14_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA)
// #define GPIO_14_IRQ         EXTI1_IRQn
// /* GPIO channel 15 config */
// #define GPIO_15_PORT        GPIOC
// #define GPIO_15_PIN         12
// #define GPIO_15_CLKEN()     (periph_clk_en(AHB2, RCC_AHB2ENR_GPIOCEN))
// #define GPIO_15_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PC)
// #define GPIO_15_IRQ         EXTI15_10_IRQn

/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    // {
    //     .dev      = TIM2,
    //     .rcc_mask = RCC_APB1ENR1_TIM2EN,
    //     .chan     = { { .pin = GPIO_PIN(PORT_A, 5), .cc_chan = 0 },
    //                   { .pin = GPIO_PIN(PORT_A, 1), .cc_chan = 1 },
    //                   { .pin = GPIO_PIN(PORT_A, 2), .cc_chan = 2 },
    //                   { .pin = GPIO_PIN(PORT_A, 3), .cc_chan = 3 } },
    //     .af       = GPIO_AF1,
    //     .bus      = APB1
    // },
    // {
    //     .dev      = TIM3,
    //     .rcc_mask = RCC_APB1ENR1_TIM3EN,
    //     .chan     = { { .pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0 },
    //                   { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 1 },
    //                   { .pin = GPIO_UNDEF,          .cc_chan = 0 },
    //                   { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
    //     .af       = GPIO_AF2,
    //     .bus      = APB1
    // },
        {
        .dev      = TIM16,
        .rcc_mask = RCC_APB2ENR_TIM16EN,
        .chan     = { { .pin = GPIO_PIN(PORT_B, 8), .cc_chan = 1 },
                      { .pin = GPIO_UNDEF, .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF14,
        .bus      = APB2
    }
};

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
    {
        .dev      = SPI2,
        .mosi_pin = GPIO_PIN(PORT_B, 15),
        .miso_pin = GPIO_PIN(PORT_B, 14),
        .sclk_pin = GPIO_PIN(PORT_B, 13),
        .cs_pin   = GPIO_UNDEF,
        .af       = GPIO_AF5,
        .rccmask  = RCC_APB1ENR1_SPI2EN,
        .apbbus   = APB1
    }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

// /**
//  * @name I2C configuration
//   * @{
//  */
// #define I2C_0_EN            1
// #define I2C_1_EN            1
// #define I2C_NUMOF           (I2C_0_EN + I2C_1_EN)
// #define I2C_IRQ_PRIO        CPU_DEFAULT_IRQ_PRIO
// #define I2C_APBCLK          (CLOCK_APB1)

// /* I2C 0 device configuration */
// #define I2C_0_EVT_ISR       isr_i2c1_ev
// #define I2C_0_ERR_ISR       isr_i2c1_er

// /* I2C 1 device configuration */
// #define I2C_1_EVT_ISR       isr_i2c2_ev
// #define I2C_1_ERR_ISR       isr_i2c2_er

// static const i2c_conf_t i2c_config[] = {
//     /* device, port, scl-, sda-pin-number, I2C-AF, ER-IRQn, EV-IRQn */
// 	{I2C1, GPIO_PIN(PORT_A,  9), GPIO_PIN(PORT_A,  10), GPIO_OD_PU,
//      GPIO_AF4, I2C1_ER_IRQn, I2C1_EV_IRQn},

// 	{I2C2, GPIO_PIN(PORT_B, 10), GPIO_PIN(PORT_B, 11), GPIO_OD_PU,
//      GPIO_AF4, I2C2_ER_IRQn, I2C2_EV_IRQn},
// };

// /** @} */

/**
 * @brief   ADC configuration
 *
 * We need to configure the following values:
 * [ pin, channel ]
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

#define ADC_NUMOF               (0)
/** @} */
#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
