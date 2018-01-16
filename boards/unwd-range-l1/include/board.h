/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_unwd-range-l1
 * @ingroup     boards
 * @brief       Board specific files for the Unwired Range R160617 board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Unwired Range R160617 board.
 *
 * @author      Cr0s
 */

#ifndef BOARD_H_
#define BOARD_H_


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name SX127X configuration
 * @{
 */
#define RF_FREQUENCY                                868900000 // Hz, 868MHz
#define TX_OUTPUT_POWER                             14        // dBm



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

#define SX127X_DIO0 GPIO_PIN(PORT_A, 8)
#define SX127X_DIO1 GPIO_PIN(PORT_A, 9)
#define SX127X_DIO2 GPIO_PIN(PORT_A, 10)
#define SX127X_DIO3 GPIO_PIN(PORT_A, 11)
#define SX127X_DIO4 GPIO_UNDEF
#define SX127X_DIO5 GPIO_UNDEF
#define SX127X_RESET GPIO_PIN(PORT_A, 0)
#define SX127X_RFSWITCH GPIO_UNDEF

/** SX127x SPI */
#define SX127X_SPI 1
#define SX127X_SPI_NSS  GPIO_PIN(PORT_B, 12)

/** "Connect" Button */
#define UNWD_USE_CONNECT_BTN	1
#define UNWD_CONNECT_BTN		GPIO_PIN(PORT_A, 1)

/** LEDs */
#define LED_GREEN   GPIO_PIN(PORT_B, 0)
#define LED_RED     GPIO_UNDEF

/** GPIO Ports */
#define UNWD_GPIO_1 GPIO_PIN(PORT_A, 1)
#define UNWD_GPIO_2 GPIO_PIN(PORT_A, 2)
#define UNWD_GPIO_3 GPIO_PIN(PORT_A, 3)
#define UNWD_GPIO_4 GPIO_PIN(PORT_A, 4)
#define UNWD_GPIO_5 GPIO_PIN(PORT_A, 5)
#define UNWD_GPIO_6 GPIO_PIN(PORT_A, 6)
#define UNWD_GPIO_7 GPIO_PIN(PORT_A, 7)
#define UNWD_GPIO_16 GPIO_PIN(PORT_B, 1)
#define UNWD_GPIO_17 GPIO_PIN(PORT_B, 2)

#define UNWD_GPIO_30 GPIO_PIN(PORT_B, 1)
#define UNWD_GPIO_29 GPIO_PIN(PORT_B, 10)
#define UNWD_GPIO_28 GPIO_PIN(PORT_B, 9)
#define UNWD_GPIO_27 GPIO_PIN(PORT_B, 8)
#define UNWD_GPIO_26 GPIO_PIN(PORT_B, 7)
#define UNWD_GPIO_25 GPIO_PIN(PORT_B, 6)
#define UNWD_GPIO_24 GPIO_PIN(PORT_B, 5)

/** @} */

#define UART_STDIO_DEV              UART_DEV(1)
#define UART_STDIO_BAUDRATE         (115200U)
#define UART_STDIO_RX_BUFSIZE       (64U)

#define GATE_COMM_UART              (UART_DEV(0))

#define UMDK_UART_DEV 2
#define UMDK_UART_BAUDRATE_NO 7 /* 115200 */

/**
 * @name xtimer configuration
 * @{
 */
#define XTIMER              TIMER_DEV(0)
#define XTIMER_CHAN         (0)
#define XTIMER_OVERHEAD     (6)
#define XTIMER_BACKOFF      (3)
/** @} */

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
