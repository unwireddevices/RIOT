/*
 * Copyright (C) 2016 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l1
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Unwired Range L1 R160617 board
 *
 * @author      Eugeny Ponomarev <ep@unwds.com>
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
    },
    {
        .dev      = TIM9,
        .max      = TIMER_0_MAX_VALUE,
        .rcc_mask = RCC_APB2ENR_TIM9EN,
        .bus      = APB2,
        .irqn     = TIM9_IRQn
    }
};
#define TIMER_0_ISR         isr_tim11
#define TIMER_1_ISR         isr_tim9

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

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
 * @brief   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 5), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_A, 1), .cc_chan = 1 },
                      { .pin = GPIO_PIN(PORT_B, 10), .cc_chan = 2 },
                      { .pin = GPIO_PIN(PORT_B, 11), .cc_chan = 3 } },
        .af       = GPIO_AF1,
        .bus      = APB1,
        .irqn     = TIM2_IRQn
    },
    {
        .dev      = TIM3,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 1 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    },
        {
        .dev      = TIM4,
        .rcc_mask = RCC_APB1ENR_TIM4EN,
        .chan     = { { .pin = GPIO_PIN(PORT_B, 8), .cc_chan = 2 },
                      { .pin = GPIO_PIN(PORT_B, 9), .cc_chan = 3 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 },
                      { .pin = GPIO_UNDEF,          .cc_chan = 0 } },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM4_IRQn
    }
};

#define TIM_0_ISR           isr_tim2
#define TIM_1_ISR           isr_tim3
#define TIM_2_ISR           isr_tim4

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
 
#define I2C_APBCLK          (CLOCK_APB1)

static const i2c_conf_t i2c_config[] = {
    {
        .dev            = I2C1,
        .speed          = I2C_SPEED_NORMAL,
        .scl        = GPIO_PIN(PORT_B,  8),
        .sda        = GPIO_PIN(PORT_B,  9),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C1EN,
        .clk            = I2C_APBCLK,
        .irqn           = I2C1_EV_IRQn
    },
    {
        .dev            = I2C2,
        .speed          = I2C_SPEED_NORMAL,
        .scl        = GPIO_PIN(PORT_B, 10),
        .sda        = GPIO_PIN(PORT_B, 11),
        .scl_af         = GPIO_AF4,
        .sda_af         = GPIO_AF4,
        .bus            = APB1,
        .rcc_mask       = RCC_APB1ENR_I2C2EN,
        .clk            = I2C_APBCLK,
        .irqn           = I2C2_EV_IRQn
    }
};

#define I2C_0_ISR           isr_i2c1_ev
#define I2C_1_ISR           isr_i2c2_ev

#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

/**
 * @name    DMA streams configuration
 * @{
 */
#ifdef MODULE_PERIPH_DMA
static const dma_conf_t dma_config[] = {
    { .stream = 0 },    /* DMA1 Channel 1 - ADC1 */
    { .stream = 1 },    /* DMA1 Channel 2 - USART3_TX / SPI1_RX */
    { .stream = 2 },    /* DMA1 Channel 3 - USART3_RX / SPI1_TX */
    { .stream = 3 },    /* DMA1 Channel 4 - USART1_TX / SPI2_RX / I2C2_TX */
    { .stream = 4 },    /* DMA1 Channel 5 - USART1_RX / SPI2_TX / I2C2_RX */
    { .stream = 5 },    /* DMA1 Channel 6 - USART2_RX / I2C1_TX */
    { .stream = 6 },    /* DMA1 Channel 7 - USART2_TX / I2C1_RX */
};

#define DMA_0_ISR  isr_dma1_ch1
#define DMA_1_ISR  isr_dma1_ch2
#define DMA_2_ISR  isr_dma1_ch3
#define DMA_3_ISR  isr_dma1_ch4
#define DMA_4_ISR  isr_dma1_ch5
#define DMA_5_ISR  isr_dma1_ch6
#define DMA_6_ISR  isr_dma1_ch7

#define DMA_NUMOF           (sizeof(dma_config) / sizeof(dma_config[0]))
#endif
/** @} */

/**
 * @brief   ADC configuration
 *
 * We need to configure the following values:
 * [ pin, channel ]
 * @{
 */
static const adc_conf_t adc_config[] = {
    { .pin = GPIO_PIN(PORT_A, 1), .chan = 1,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
    { .pin = GPIO_PIN(PORT_A, 2), .chan = 2,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
    { .pin = GPIO_PIN(PORT_A, 3), .chan = 3,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
    { .pin = GPIO_PIN(PORT_A, 4), .chan = 4,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
    { .pin = GPIO_PIN(PORT_A, 5), .chan = 5,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
    { .pin = GPIO_PIN(PORT_A, 6), .chan = 6,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
	{ .pin = GPIO_PIN(PORT_A, 7), .chan = 7,                       .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
	{ .pin = GPIO_UNDEF,          .chan = ADC_VREF_CHANNEL,        .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
	{ .pin = GPIO_UNDEF,          .chan = ADC_TEMPERATURE_CHANNEL, .trigger = ADC_EXT_TRIGGER_TIM9TRGO },
};

#define ADC_VREF_INDEX          7
#define ADC_TEMPERATURE_INDEX   8

#define ADC_NUMOF           (sizeof(adc_config) / sizeof(adc_config[0]))
/** @} */
#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
