/*
 * Copyright (C) 2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_unwd-log
 * @ingroup     boards
 * @brief       Board specific files for the UNWD-LOG board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the UNWD-LOG
 *
 * @author      Oleg Artamonov <info@unwds.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/** "Connect" Button */
#define UNWD_USE_CONNECT_BTN	0
#define UNWD_CONNECT_BTN		GPIO_UNDEF

/** LEDs */
#define LED_GREEN   GPIO_UNDEF
#define LED_RED     GPIO_UNDEF

/** GPIO Ports */
#define UNWD_GPIO_1 GPIO_PIN(PORT_C, 0)
#define UNWD_GPIO_2 GPIO_PIN(PORT_C, 1)
#define UNWD_GPIO_3 GPIO_PIN(PORT_C, 2)
#define UNWD_GPIO_4 GPIO_PIN(PORT_C, 3)
#define UNWD_GPIO_5 GPIO_PIN(PORT_C, 4)
#define UNWD_GPIO_6 GPIO_PIN(PORT_C, 5)
#define UNWD_GPIO_7 GPIO_PIN(PORT_C, 6)
#define UNWD_GPIO_16 GPIO_PIN(PORT_C, 7)
#define UNWD_GPIO_17 GPIO_PIN(PORT_C, 15)

#define UNWD_GPIO_30 GPIO_PIN(PORT_C, 14)
#define UNWD_GPIO_29 GPIO_PIN(PORT_C, 13)
#define UNWD_GPIO_28 GPIO_PIN(PORT_C, 12)
#define UNWD_GPIO_27 GPIO_PIN(PORT_C, 11)
#define UNWD_GPIO_26 GPIO_PIN(PORT_C, 10)
#define UNWD_GPIO_25 GPIO_PIN(PORT_C, 9)
#define UNWD_GPIO_24 GPIO_PIN(PORT_C, 8)

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

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
