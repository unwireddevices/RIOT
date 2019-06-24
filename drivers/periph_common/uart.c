/*
 * Copyright (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_periph_uart
 * @{
 *
 * @file
 * @brief       common UART function fallback implementations
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */
#include "board.h"
#include "periph/uart.h"

#ifdef UART_NUMOF

void __attribute__((weak)) uart_wait(uart_t uart)
{
    (void)uart;
}

#endif /* UART_NUMOF */
