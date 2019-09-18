/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file		iwdg.c
 * @brief       nRF5x Watchdog Implementation
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#include <stdbool.h>

#include "assert.h"

#include "cpu.h"
#include "periph/wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

void wdg_set_prescaler(uint8_t prescaler)
{
    /* do nothing */
    (void) prescaler;
}

void wdg_set_reload(uint32_t seconds)
{
    assert(seconds > 0);

    NRF_WDT->CRV = 32768 * seconds;
}

void wdg_enable(void)
{
    /* Configure Watchdog
     * a) Pause watchdog while the CPU is halted by the debugger.
     * b) Keep the watchdog running while the CPU is sleeping
     */
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos); 
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  /* Enable reload register 0 */
	NRF_WDT->TASKS_START = 1;           /* Start the Watchdog timer */
}

void wdg_reload(void)
{
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

bool wdg_reset_occurred(void)
{
    return (NRF_POWER->RESETREAS & POWER_RESETREAS_DOG_Msk);
}

uint32_t wdg_get_value(void)
{
    return NRF_WDT->CRV/32768;
}

#ifdef __cplusplus
}
#endif
