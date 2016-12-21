/*
 * Copyright (C) 2016 Unwired Devices
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
 * @brief       Peripheral MCU configuration for the unwd-range-l1 R160829 board
 *
 * @author      Mikhail Churikov
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
#define CLOCK_HSI           (16000000U)             /* frequency of internal oscillator */
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

/**
 * @brief   DAC configuration
 * @{
 */
#define DAC_NUMOF           (0)
/** @} */

/**
 * @brief Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    /* device, RCC bit, IRQ bit */
    {TIM5, 3, TIM5_IRQn},
};
/* interrupt routines */
#define TIMER_0_ISR         (isr_tim5)
/* number of defined timers */
#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name Real time counter configuration
 * @{
 */
#define RTC_NUMOF           (1U)

/**
 * @brief UART configuration
 */
#define UART_NUMOF          (3U)
#define UART_0_EN           1
#define UART_1_EN           1
#define UART_2_EN           0
#define UART_IRQ_PRIO       1

/* UART 0 (USART1) device configuration */
#define UART_0_DEV          USART1
#define UART_0_CLKEN()      (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define UART_0_CLKDIS()		(RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN)
#define UART_0_CLK          (CLOCK_CORECLOCK)   /* UART clock runs with 32MHz (F_CPU / 1) */
#define UART_0_IRQ          USART1_IRQn
#define UART_0_ISR          isr_usart1
#define UART_0_BUS_FREQ     32000000
/* UART 0 pin configuration */
#define UART_0_RX_PIN       GPIO_PIN(PORT_B, 7)
#define UART_0_TX_PIN       GPIO_PIN(PORT_B, 6)
#define UART_0_AF           GPIO_AF7

/* UART 1 (USART2) device configuration */
#define UART_1_DEV          USART2
#define UART_1_CLKEN()      (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define UART_1_CLKDIS()     (RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN)
#define UART_1_ISON()		(RCC->APB1ENR & RCC_APB1ENR_USART2EN)
#define UART_1_CLK          (CLOCK_CORECLOCK)   /* UART clock runs with 32MHz (F_CPU / 1) */
#define UART_1_IRQ          USART2_IRQn
#define UART_1_ISR          isr_usart2
#define UART_1_BUS_FREQ     32000000
/* UART 1 pin configuration */
#define UART_1_RX_PIN       GPIO_PIN(PORT_A, 3)
#define UART_1_TX_PIN       GPIO_PIN(PORT_A, 2)
#define UART_1_AF           GPIO_AF7

/* UART 2 (USART3) device configuration */
#define UART_2_DEV          USART3
#define UART_2_CLKEN()      (RCC->APB1ENR |= RCC_APB1ENR_USART3EN)
#define UART_2_CLKDIS()     (RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN)
#define UART_2_ISON()		(RCC->APB1ENR & RCC_APB1ENR_USART3EN)
#define UART_2_CLK          (CLOCK_CORECLOCK)   /* UART clock runs with 32MHz (F_CPU / 1) */
#define UART_2_IRQ          USART3_IRQn
#define UART_2_ISR          isr_usart3
#define UART_2_BUS_FREQ     32000000
/* UART 2 pin configuration */
#define UART_2_RX_PIN       GPIO_PIN(PORT_B, 11)
#define UART_2_TX_PIN       GPIO_PIN(PORT_B, 10)
#define UART_2_AF           GPIO_AF7

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
#define GPIO_IRQ_PRIO       1

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
#define GPIO_0_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_0_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA)
#define GPIO_0_IRQ          EXTI3_IRQn
/* GPIO channel 1 config */
#define GPIO_1_PORT         GPIOC
#define GPIO_1_PIN          7
#define GPIO_1_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOCEN)
#define GPIO_1_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PC)
#define GPIO_1_IRQ          EXTI9_5_IRQn
/* GPIO channel 2 config */
#define GPIO_2_PORT         GPIOA
#define GPIO_2_PIN          9
#define GPIO_2_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_2_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PA)
#define GPIO_2_IRQ          EXTI9_5_IRQn
/* GPIO channel 3 config */
#define GPIO_3_PORT         GPIOA
#define GPIO_3_PIN          8
#define GPIO_3_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_3_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA)
#define GPIO_3_IRQ          EXTI9_5_IRQn
/* GPIO channel 4 config */
#define GPIO_4_PORT         GPIOB
#define GPIO_4_PIN          10
#define GPIO_4_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define GPIO_4_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB)
#define GPIO_4_IRQ          EXTI15_10_IRQn
/* GPIO channel 5 config */
#define GPIO_5_PORT         GPIOB
#define GPIO_5_PIN          4
#define GPIO_5_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define GPIO_5_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB)
#define GPIO_5_IRQ          EXTI4_IRQn
/* GPIO channel 6 config */
#define GPIO_6_PORT         GPIOC
#define GPIO_6_PIN          11
#define GPIO_6_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOCEN)
#define GPIO_6_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC)
#define GPIO_6_IRQ          EXTI15_10_IRQn
/* GPIO channel 7 config */
#define GPIO_7_PORT         GPIOC
#define GPIO_7_PIN          2
#define GPIO_7_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOCEN)
#define GPIO_7_EXTI_CFG()   (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC)
#define GPIO_7_IRQ          EXTI2_IRQn
/* GPIO channel 8 config */
#define GPIO_8_PORT         GPIOA
#define GPIO_8_PIN          13
#define GPIO_8_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_8_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PA)
#define GPIO_8_IRQ          EXTI15_10_IRQn
/* GPIO channel 9 config */
#define GPIO_9_PORT         GPIOA
#define GPIO_9_PIN          14
#define GPIO_9_CLKEN()      (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_9_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PA)
#define GPIO_9_IRQ          EXTI15_10_IRQn
/* GPIO channel 10 config */
#define GPIO_10_PORT        GPIOA
#define GPIO_10_PIN         15
#define GPIO_10_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_10_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA)
#define GPIO_10_IRQ         EXTI15_10_IRQn
/* GPIO channel 11 config */
#define GPIO_11_PORT        GPIOB   /* SPI CS Pin */
#define GPIO_11_PIN         6
#define GPIO_11_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define GPIO_11_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB)
#define GPIO_11_IRQ         EXTI9_5_IRQn
/* GPIO channel 12 config */
#define GPIO_12_PORT        GPIOC
#define GPIO_12_PIN         5
#define GPIO_12_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOCEN)
#define GPIO_12_EXTI_CFG()  (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC)
#define GPIO_12_IRQ         EXTI9_5_IRQn
/* GPIO channel 13 config */
#define GPIO_13_PORT        GPIOA
#define GPIO_13_PIN         0
#define GPIO_13_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_13_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA)
#define GPIO_13_IRQ         EXTI0_IRQn
/* GPIO channel 14 config */
#define GPIO_14_PORT        GPIOA
#define GPIO_14_PIN         1
#define GPIO_14_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define GPIO_14_EXTI_CFG()  (SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA)
#define GPIO_14_IRQ         EXTI1_IRQn
/* GPIO channel 15 config */
#define GPIO_15_PORT        GPIOC
#define GPIO_15_PIN         12
#define GPIO_15_CLKEN()     (RCC->AHBENR |= RCC_AHBENR_GPIOCEN)
#define GPIO_15_EXTI_CFG()  (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PC)
#define GPIO_15_IRQ         EXTI15_10_IRQn

/**
 * @brief PWM configuration
 * @{
 */
#define PWM_NUMOF           (3U)
#define PWM_0_EN            1
#define PWM_1_EN            1
#define PWM_2_EN            1

#define PWM_MAX_CHANNELS    4

/* PWM 0 device configuration */
#define PWM_0_DEV           TIM2
#define PWM_0_CHANNELS      4
#define PWM_0_CLK           (32000000U)
#define PWM_0_CLKEN()       (RCC->APB1ENR |= RCC_APB1ENR_TIM2EN)
#define PWM_0_CLKDIS()      (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN))
/* PWM 0 pin configuration */
#define PWM_0_PORT          GPIOA
#define PWM_0_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define PWM_0_PIN_CH0       5
#define PWM_0_PIN_CH1       1
#define PWM_0_PIN_CH2       2
#define PWM_0_PIN_CH3       3
#define PWM_0_PIN_AF        1

/* PWM 1 device configuration */
#define PWM_1_DEV           TIM3
#define PWM_1_CHANNELS      2
#define PWM_1_CLK           (32000000U)
#define PWM_1_CLKEN()       (RCC->APB1ENR |= RCC_APB1ENR_TIM3EN)
#define PWM_1_CLKDIS()      (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN))
/* PWM 1 pin configuration */
#define PWM_1_PORT          GPIOA
#define PWM_1_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define PWM_1_PIN_CH0       6
#define PWM_1_PIN_CH1       7
#define PWM_1_PIN_CH2       0 /* do not use */
#define PWM_1_PIN_CH3       0 /* do not use */
#define PWM_1_PIN_AF        2

/* PWM 2 device configuration */
#define PWM_2_DEV           TIM4
#define PWM_2_CHANNELS      2
#define PWM_2_CLK           (32000000U)
#define PWM_2_CLKEN()       (RCC->APB1ENR |= RCC_APB1ENR_TIM4EN)
#define PWM_2_CLKDIS()      (RCC->APB1ENR &= ~(RCC_APB1ENR_TIM4EN))
/* PWM 2 pin configuration */
#define PWM_2_PORT          GPIOB
#define PWM_2_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define PWM_2_PIN_CH0       0 /* do not use */
#define PWM_2_PIN_CH1       0 /* do not use */
#define PWM_2_PIN_CH2       8
#define PWM_2_PIN_CH3       9
#define PWM_2_PIN_AF        2
/** @} */

/**
 * @brief SPI configuration
 * @{
 */
#define SPI_NUMOF           (2U)
#define SPI_0_EN            1
#define SPI_1_EN			1

/* SPI 0 device configuration */
#define SPI_0_DEV           SPI1
#define SPI_0_CLKEN()       (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#define SPI_0_CLKDIS()      (RCC->APB2ENR  &= ~(RCC_APB2ENR_SPI1EN))
#define SPI_0_IRQ           SPI1_IRQn
#define SPI_0_ISR           isr_spi1
/* SPI 0 pin configuration */
#define SPI_0_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOAEN)
#define SPI_0_PORT          GPIOA
#define SPI_0_PIN_NSS       4
#define SPI_0_PIN_SCK       5
#define SPI_0_PIN_MISO      6
#define SPI_0_PIN_MOSI      7

#define SPI_0_PIN_AF        5

/* SPI 0 device configuration */
#define SPI_1_DEV           SPI2
#define SPI_1_CLKEN()       (RCC->APB1ENR |= RCC_APB1ENR_SPI2EN)
#define SPI_1_ISON()        (RCC->APB1ENR & RCC_APB1ENR_SPI2EN)
#define SPI_1_CLKDIS()      (RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN))
#define SPI_1_IRQ           SPI2_IRQn
#define SPI_1_ISR           isr_spi2
/* SPI 0 pin configuration */
#define SPI_1_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define SPI_1_PORT          GPIOB
#define SPI_1_PIN_NSS       12
#define SPI_1_PIN_SCK       13
#define SPI_1_PIN_MISO      14
#define SPI_1_PIN_MOSI      15 // swapped?

#define SPI_1_PIN_AF        5
/** @} */

/**
 * @name I2C configuration
  * @{
 */
#define I2C_0_EN            1
#define I2C_1_EN            1
#define I2C_NUMOF           (I2C_0_EN + I2C_1_EN)
#define I2C_IRQ_PRIO        1
#define I2C_APBCLK          (36000000U)

/* I2C 0 device configuration */
#define I2C_0_EVT_ISR       isr_i2c2_ev
#define I2C_0_ERR_ISR       isr_i2c2_er

/* I2C 1 device configuration */
#define I2C_1_EVT_ISR       isr_i2c1_ev
#define I2C_1_ERR_ISR       isr_i2c1_er

static const i2c_conf_t i2c_config[] = {
    /* device, port, scl-, sda-pin-number, I2C-AF, ER-IRQn, EV-IRQn */
	{I2C2, GPIO_PIN(PORT_B, 10), GPIO_PIN(PORT_B, 11), GPIO_OD_PU,
     GPIO_AF4, I2C2_ER_IRQn, I2C2_EV_IRQn, 1},

	{I2C1, GPIO_PIN(PORT_B,  8), GPIO_PIN(PORT_B,  9), GPIO_OD_PU,
     GPIO_AF4, I2C1_ER_IRQn, I2C1_EV_IRQn, 0},
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
