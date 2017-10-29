/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_unwd-range-l1 Unwired Range-l1
 * @ingroup     boards
 * @brief       Board specific files for the unwd-range-l1 board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the unwd-range-l1 R160829 board.
 *
 * @author      Mikhail Churikov
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "board_common.h"
#include "hd44780.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name SX1276 configuration
 * @{
 */
#define RF_FREQUENCY                                868900000 // Hz, 868MHz
#define TX_OUTPUT_POWER                             17        // dBm



#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
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
#define LORA_FIX_LENGTH_PAYLOAD_ON                  true
#define LORA_IQ_INVERSION							false

#define SX1276_DIO0 GPIO_PIN(PORT_A, 15)
#define SX1276_DIO1 GPIO_PIN(PORT_A, 14)
#define SX1276_DIO2 GPIO_UNDEF
#define SX1276_DIO3 GPIO_PIN(PORT_A, 12)
#define SX1276_DIO4 GPIO_PIN(PORT_A, 11)
#define SX1276_DIO5 GPIO_UNDEF

#define SX1276_RESET GPIO_UNDEF

/** RF on/off switching pin */
#define SX1276_RFSWITCH GPIO_PIN(PORT_B, 12)

/** SX1276 SPI */

#define USE_SPI_1

#ifdef USE_SPI_1
#define SX1276_SPI SPI_1
#define SX1276_SPI_NSS GPIO_PIN(PORT_B, 6)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_SPI SPI_0
#define SX1276_SPI_NSS GPIO_PIN(PORT_A, SPI_0_PIN_NSS)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

/** @} */

#define UART_STDIO_DEV              UART_DEV(0)
#define UART_STDIO_BAUDRATE         (115200U)
#define UART_STDIO_RX_BUFSIZE       (64U)

/**
 * @name xtimer configuration
 * @{
 */
#define XTIMER              TIMER_DEV(0)
#define XTIMER_CHAN         (0)
#define XTIMER_OVERHEAD     (6)
#define XTIMER_BACKOFF      (3)
/** @} */

/* umdk-16x2 board uses PCF8574 I2C GPIO expander */ 
/* so below are not MCU GPIOs but PCF8574 GPIO numbers */
#define UMDK_HD44780_PARAMS {    \
    .cols   = 16,       \
    .rows   = 2,        \
    .rs     = 4,        \
    .rw     = 5,        \
    .enable = 6,        \
    .data   = {0, 1, 2, 3,     \
               HD44780_RW_OFF, HD44780_RW_OFF, HD44780_RW_OFF, HD44780_RW_OFF}, \
    .backlight = 7,     \
    .i2c_dev = 1,       \
    .i2c_address = 0x20 \
}

static const hd44780_params_t hd44780_params[] =
{
    UMDK_HD44780_PARAMS,
};

#define UMDK_HD44780_ROWS 2
#define UMDK_HD44780_COLS 16

/* 1 to normal mode, 0 to low-power mode */
#define VOUT_3V3_SLEEP_PIN  GPIO_PIN(PORT_A, 1)

/* 1 to enable, 0 to disable */
#define RS485_POWER_PIN  GPIO_PIN(PORT_C, 13)

/* 1 to enable, 0 to disable */
#define MODEM_POWER_PIN  GPIO_PIN(PORT_A, 13)

/* 1 to 5 V, 0 to 3.8V */
#define MODEM_POWER_SELECT  GPIO_PIN(PORT_A, 6)


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
