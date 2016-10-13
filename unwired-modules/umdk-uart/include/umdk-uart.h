/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		umdk-uart.h
 * @brief       umdk-uart module definitions
 * @author      EP
 */
#ifndef UMDK_UART_H
#define UMDK_UART_H

#include "unwds-common.h"

#define UNWDS_UART_MODULE_ID 7

#define UMDK_UART_RXBUF_SIZE 128

#define UMDK_UART_SYMBOL_TIMEOUT_MS 500

/**
 * @brief   DE/RE pins definitions and handlers
 * @{
 */
#define DE_PIN            UNWD_GPIO_30
#define DE_PIN_NUM        10 // Port B

#define DE_MASK           (1 << DE_PIN_NUM)

#if defined(CPU_FAM_STM32F4)
#define DE_CREG            BSRRH
#else
#define DE_CREG            BRR
#endif
#if defined(CPU_FAM_STM32F3) || defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L1)
#define DE_SREG            BSRRL
#else
#define DE_SREG            BSRR
#endif

#define DE_ON             (GPIOB->DE_SREG = DE_MASK)
#define DE_OFF            (GPIOB->DE_CREG = DE_MASK)
#define DE_TOGGLE         (GPIOB->ODR     ^= DE_MASK)

#define DE_ENABLE         (GPIOB->LED_SREG = DE_MASK)
#define DE_DISABLE        (GPIOB->LED_CREG = DE_MASK)

#define RE_PIN            UNWD_GPIO_29 
#define RE_PIN_NUM        11 // Port B

#define RE_MASK           (1 << RE_PIN_NUM)

#if defined(CPU_FAM_STM32F4)
#define RE_CREG            BSRRH
#else
#define RE_CREG            BRR
#endif
#if defined(CPU_FAM_STM32F3) || defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L1)
#define RE_SREG            BSRRL
#else
#define RE_SREG            BSRR
#endif

#define RE_ON             (GPIOB->RE_SREG = RE_MASK)
#define RE_OFF            (GPIOB->RE_CREG = RE_MASK)
#define RE_TOGGLE         (GPIOB->ODR     ^= RE_MASK)

#define RE_DISABLE        (GPIOB->RE_SREG = RE_MASK)
#define RE_ENABLE         (GPIOB->RE_CREG = RE_MASK)

/** @} */

typedef enum {
	UMDK_UART_SEND_ALL = 0,
	UMDK_UART_SET_BAUDRATE = 1,
} umdk_uart_prefix_t;

typedef enum {
	UMDK_UART_REPLY_SENT = 0,
	UMDK_UART_REPLY_RECEIVED = 1,
	UMDK_UART_REPLY_BAUDRATE_SET = 2,
	/* ... */
	UMDK_UART_REPLY_ERR_OVF = 253,	/* RX buffer overflowed */
	UMDK_UART_REPLY_ERR_FMT = 254,
	UMDK_UART_ERR = 255,
} umdk_uart_reply_t;

void umdk_uart_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_uart_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_UART_H */
