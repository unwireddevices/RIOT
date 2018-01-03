/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_stm32f0discovery STM32F0Discovery
 * @ingroup     boards
 * @brief       Support for the STM32F0Discovery board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32F0Discovery evaluation board.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Sebastian Meiling <s@mlng.net>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @name SX1276 configuration
 * @{
 */
#define RF_FREQUENCY                                868900000   // Hz, 868.9MHz
#define TX_OUTPUT_POWER                             10          // dBm

#define LORA_PREAMBLE_LENGTH                        8           // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         10          // Symbols

#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION                           false

#define SX1276_DIO0 GPIO_PIN(PORT_A, 8)
#define SX1276_DIO1 GPIO_PIN(PORT_A, 9)
#define SX1276_DIO2 GPIO_PIN(PORT_A, 10)
#define SX1276_DIO3 GPIO_PIN(PORT_A, 11)

#define SX1276_RESET GPIO_PIN(PORT_C, 6)

/** Antenna mode (RX/TX) switching pin */
#define SX1276_ANTSW GPIO_PIN(PORT_A, 12)

/** SX1276 SPI */

#define USE_SPI_0

#ifdef USE_SPI_1
#define SX1276_SPI SPI_1
#define SX1276_SPI_NSS GPIO_PIN(PORT_B, 12)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_SPI SPI_0
#define SX1276_SPI_NSS GPIO_PIN(PORT_A, 4)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED0_PIN            GPIO_PIN(PORT_C, 9)
#define LED1_PIN            GPIO_PIN(PORT_C, 8)

#define LED_PORT            GPIOC
#define LED0_MASK           (1 << 9)
#define LED1_MASK           (1 << 8)

#define LED0_ON             (LED_PORT->BSRR = LED0_MASK)
#define LED0_OFF            (LED_PORT->BRR  = LED0_MASK)
#define LED0_TOGGLE         (LED_PORT->ODR ^= LED0_MASK)

#define LED1_ON             (LED_PORT->BSRR = LED1_MASK)
#define LED1_OFF            (LED_PORT->BRR  = LED1_MASK)
#define LED1_TOGGLE         (LED_PORT->ODR ^= LED1_MASK)
/** @} */

/**
 * @brief User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_A, 0)
#define BTN0_MODE           GPIO_IN
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
