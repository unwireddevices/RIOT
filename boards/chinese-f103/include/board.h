/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f103 Nucleo-F103
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f103 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f103 board
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "board_common.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @name SX1276 configuration
 * @{
 */
#define RF_FREQUENCY                                868000000 // Hz, 868MHz
#define TX_OUTPUT_POWER                             10        // dBm



#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION							false

#define SX1276_DIO0 GPIO_PIN(PORT_A, 8)
#define SX1276_DIO1 GPIO_PIN(PORT_A, 9)
#define SX1276_DIO2 GPIO_PIN(PORT_A, 10)
#define SX1276_DIO3 GPIO_PIN(PORT_A, 11)

/** Antenna mode (RX/TX) switching pin */
#define SX1276_ANTSW GPIO_PIN(PORT_A, 12)

/** SX1276 SPI */

#define USE_SPI_1

#ifdef USE_SPI_1
#define SX1276_SPI SPI_1
#define SX1276_SPI_NSS SPI_1_NSS_PIN
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_100KHZ
#endif

#ifdef USE_SPI_0
#define SX1276_SPI SPI_0
#define SX1276_SPI_NSS GPIO_PIN(PORT_A, SPI_0_NSS_PIN)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

/** Unused pins */
#define SX1276_DIO4 NULL
#define SX1276_DIO5 NULL
#define SX1276_RESET NULL

/**
 * @brief Use the 2nd UART for STDIO on this board
 */
#define UART_STDIO_DEV      UART_DEV(1)

/**
 * @name xtimer configuration
 */
#define XTIMER_MASK         (0xffff0000)
#define XTIMER_BACKOFF      5
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
