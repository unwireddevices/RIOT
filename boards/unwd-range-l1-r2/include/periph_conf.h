/*
 * Copyright (C) 2016-2018 Unwired Devices <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l1-r2
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Unwired Range R160829 board
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
 **/
#define CLOCK_LSE           (32768)                 /* external low-speed crystal frequency  */
#define CLOCK_HSE           (24000000U)             /* external high-speed crystal frequency */
#define CLOCK_HSI           (16000000U)             /* internal high-speed crystal frequency */
#define CLOCK_CORECLOCK     (32000000U)             /* targeted core clock frequency */
/* configuration of PLL prescaler and multiply values */
/* CORECLOCK := HSI / CLOCK_PLL_DIV * CLOCK_PLL_MUL */
#define CLOCK_PLL_DIV       RCC_CFGR_PLLDIV2
#define CLOCK_PLL_MUL       RCC_CFGR_PLLMUL4

#define CLOCK_PLL_DIV_HSI   RCC_CFGR_PLLDIV2
#define CLOCK_PLL_MUL_HSI   RCC_CFGR_PLLMUL4

#define CLOCK_PLL_DIV_HSE   RCC_CFGR_PLLDIV3
#define CLOCK_PLL_MUL_HSE   RCC_CFGR_PLLMUL4

#define CLOCK_STATUS_BACKUP_REG 1
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
 * @name   DAC configuration
 * @{
 */
static const dac_conf_t dac_config[] = {
    { .pin = GPIO_PIN(PORT_A,  4), .chan = 0 },
    { .pin = GPIO_PIN(PORT_A,  5), .chan = 1 }
};

#define DAC_NUMOF           (sizeof(dac_config) / sizeof(dac_config[0]))
/** @} */

/**
 * @brief Timer configuration
 * @{
 */
#define TIMER_0_MAX_VALUE   (0x0000ffff)
 
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM11,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB2ENR_TIM11EN,
        .bus      = APB2,
        .irqn     = TIM11_IRQn
    }
};

#define TIMER_0_ISR         isr_tim11

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name Real time counter configuration
 * @{
 */
#define RTC_NUMOF           (1U)

#define RTC_IRQ_PRIO        CPU_DEFAULT_IRQ_PRIO

/**
 * @brief UART configuration
 */
static const uart_conf_t uart_config[] = {
    {
        .dev      = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin   = GPIO_PIN(PORT_B, 7),
        .tx_pin   = GPIO_PIN(PORT_B, 6),
        .rx_mode  = GPIO_IN_PU,
        .tx_mode  = GPIO_OUT,
        .rx_af    = GPIO_AF7,
        .tx_af    = GPIO_AF7,
        .bus      = APB2,
        .irqn     = USART1_IRQn
    },
    {
        .dev      = USART2,
        .rcc_mask = RCC_APB1ENR_USART2EN,
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
        .rcc_mask = RCC_APB1ENR_USART3EN,
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

/**
 * @brief GPIO configuration
 */
#define GPIO_0_EN           1
#define GPIO_1_EN           1
#define GPIO_2_EN           1
#define GPIO_3_EN           1
#define GPIO_4_EN           1
#define GPIO_5_EN           1
#define GPIO_6_EN           1
#define GPIO_7_EN           1
#define GPIO_8_EN           1
#define GPIO_9_EN           1
#define GPIO_10_EN          1
#define GPIO_11_EN          1
#define GPIO_12_EN          1
#define GPIO_13_EN          1
#define GPIO_14_EN          1
#define GPIO_15_EN          1
#define GPIO_IRQ_PRIO       CPU_DEFAULT_IRQ_PRIO

/* IRQ config */
#define GPIO_IRQ_0          GPIO_13
#define GPIO_IRQ_1          GPIO_14
#define GPIO_IRQ_2          GPIO_7
#define GPIO_IRQ_3          GPIO_0
#define GPIO_IRQ_4          GPIO_5
#define GPIO_IRQ_5          GPIO_12
#define GPIO_IRQ_6          GPIO_11
#define GPIO_IRQ_7          GPIO_1
#define GPIO_IRQ_8          GPIO_3
#define GPIO_IRQ_9          GPIO_2
#define GPIO_IRQ_10         GPIO_4
#define GPIO_IRQ_11         GPIO_6
#define GPIO_IRQ_12         GPIO_15
#define GPIO_IRQ_13         GPIO_8
#define GPIO_IRQ_14         GPIO_9
#define GPIO_IRQ_15         GPIO_10

/* GPIO channel 0 config */
#define GPIO_0_PORT         GPIOA                   /* Used for user button 1 */
#define GPIO_0_PIN          3
#define GPIO_0_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_0_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA)
#define GPIO_0_IRQ          EXTI3_IRQn
/* GPIO channel 1 config */
#define GPIO_1_PORT         GPIOC
#define GPIO_1_PIN          7
#define GPIO_1_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOCEN))
#define GPIO_1_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PC)
#define GPIO_1_IRQ          EXTI9_5_IRQn
/* GPIO channel 2 config */
#define GPIO_2_PORT         GPIOA
#define GPIO_2_PIN          9
#define GPIO_2_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_2_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PA)
#define GPIO_2_IRQ          EXTI9_5_IRQn
/* GPIO channel 3 config */
#define GPIO_3_PORT         GPIOA
#define GPIO_3_PIN          8
#define GPIO_3_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_3_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA)
#define GPIO_3_IRQ          EXTI9_5_IRQn
/* GPIO channel 4 config */
#define GPIO_4_PORT         GPIOB
#define GPIO_4_PIN          10
#define GPIO_4_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOBEN))
#define GPIO_4_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB)
#define GPIO_4_IRQ          EXTI15_10_IRQn
/* GPIO channel 5 config */
#define GPIO_5_PORT         GPIOB
#define GPIO_5_PIN          4
#define GPIO_5_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOBEN))
#define GPIO_5_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB)
#define GPIO_5_IRQ          EXTI4_IRQn
/* GPIO channel 6 config */
#define GPIO_6_PORT         GPIOC
#define GPIO_6_PIN          11
#define GPIO_6_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOCEN))
#define GPIO_6_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC)
#define GPIO_6_IRQ          EXTI15_10_IRQn
/* GPIO channel 7 config */
#define GPIO_7_PORT         GPIOC
#define GPIO_7_PIN          2
#define GPIO_7_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOCEN))
#define GPIO_7_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC)
#define GPIO_7_IRQ          EXTI2_IRQn
/* GPIO channel 8 config */
#define GPIO_8_PORT         GPIOA
#define GPIO_8_PIN          13
#define GPIO_8_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_8_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PA)
#define GPIO_8_IRQ          EXTI15_10_IRQn
/* GPIO channel 9 config */
#define GPIO_9_PORT         GPIOA
#define GPIO_9_PIN          14
#define GPIO_9_CLKEN()      (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_9_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PA)
#define GPIO_9_IRQ          EXTI15_10_IRQn
/* GPIO channel 10 config */
#define GPIO_10_PORT        GPIOA
#define GPIO_10_PIN         15
#define GPIO_10_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_10_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA)
#define GPIO_10_IRQ         EXTI15_10_IRQn
/* GPIO channel 11 config */
#define GPIO_11_PORT        GPIOB   /* SPI CS Pin */
#define GPIO_11_PIN         6
#define GPIO_11_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOBEN))
#define GPIO_11_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB)
#define GPIO_11_IRQ         EXTI9_5_IRQn
/* GPIO channel 12 config */
#define GPIO_12_PORT        GPIOC
#define GPIO_12_PIN         5
#define GPIO_12_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOCEN))
#define GPIO_12_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC)
#define GPIO_12_IRQ         EXTI9_5_IRQn
/* GPIO channel 13 config */
#define GPIO_13_PORT        GPIOA
#define GPIO_13_PIN         0
#define GPIO_13_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_13_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA)
#define GPIO_13_IRQ         EXTI0_IRQn
/* GPIO channel 14 config */
#define GPIO_14_PORT        GPIOA
#define GPIO_14_PIN         1
#define GPIO_14_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOAEN))
#define GPIO_14_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA)
#define GPIO_14_IRQ         EXTI1_IRQn
/* GPIO channel 15 config */
#define GPIO_15_PORT        GPIOC
#define GPIO_15_PIN         12
#define GPIO_15_CLKEN()     (periph_clk_en(AHB, RCC_AHBENR_GPIOCEN))
#define GPIO_15_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PC)
#define GPIO_15_IRQ         EXTI15_10_IRQn

/**
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 5), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_A, 1), .cc_chan = 1 },
                      { .pin = GPIO_PIN(PORT_A, 2), .cc_chan = 2 },
                      { .pin = GPIO_PIN(PORT_A, 3), .cc_chan = 3 } },
        .af       = GPIO_AF1,
        .bus      = APB1
    },
    {
        .dev      = TIM3,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 1 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF2,
        .bus      = APB1
    },
        {
        .dev      = TIM4,
        .rcc_mask = RCC_APB1ENR_TIM4EN,
        .chan     = { { .pin = GPIO_PIN(PORT_B, 8), .cc_chan = 2 },
                      { .pin = GPIO_PIN(PORT_B, 9), .cc_chan = 3 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF2,
        .bus      = APB1
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
        .rccmask  = RCC_APB1ENR_SPI2EN,
        .apbbus   = APB1
    }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name I2C configuration
  * @{
 */
#define I2C_0_EN            1
#define I2C_1_EN            1
#define I2C_NUMOF           (I2C_0_EN + I2C_1_EN)
#define I2C_IRQ_PRIO        CPU_DEFAULT_IRQ_PRIO
#define I2C_APBCLK          (CLOCK_APB1)

/* I2C 0 device configuration */
#define I2C_0_EVT_ISR       isr_i2c1_ev
#define I2C_0_ERR_ISR       isr_i2c1_er

/* I2C 1 device configuration */
#define I2C_1_EVT_ISR       isr_i2c2_ev
#define I2C_1_ERR_ISR       isr_i2c2_er

static const i2c_conf_t i2c_config[] = {
    /* device, port, scl-, sda-pin-number, I2C-AF, ER-IRQn, EV-IRQn */
	{I2C1, GPIO_PIN(PORT_B,  8), GPIO_PIN(PORT_B,  9), GPIO_OD_PU,
     GPIO_AF4, I2C1_ER_IRQn, I2C1_EV_IRQn, 0},

	{I2C2, GPIO_PIN(PORT_B, 10), GPIO_PIN(PORT_B, 11), GPIO_OD_PU,
     GPIO_AF4, I2C2_ER_IRQn, I2C2_EV_IRQn, 1},
};

/** @} */

/**
 * @brief   ADC configuration
 *
 * We need to configure the following values:
 * [ pin, channel ]
 * @{
 */
#define ADC_CONFIG {            \
    { GPIO_PIN(PORT_A, 1), 1 },\
    { GPIO_PIN(PORT_A, 2), 2 },\
    { GPIO_PIN(PORT_A, 3), 3 },\
    { GPIO_PIN(PORT_A, 4), 4 },\
    { GPIO_PIN(PORT_A, 5), 5 },\
    { GPIO_PIN(PORT_A, 6), 6 }, \
	{ GPIO_PIN(PORT_A, 7), 7 }, \
	{ GPIO_UNDEF, ADC_VREF_CHANNEL}, \
	{ GPIO_UNDEF, ADC_TEMPERATURE_CHANNEL}, \
}

#define ADC_VREF_INDEX 7
#define ADC_TEMPERATURE_INDEX 8

#define ADC_NUMOF           (9)
/** @} */
#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
