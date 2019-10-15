/*
 * Copyright (C) 2017 Kaspar Schleiser <kaspar@schleiser.de>
 *               2014-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf5x_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       NRF5x specific power management implementations
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "periph/pm.h"
#include "nrf_clock.h"

#include "periph_conf.h"

static inline void _pm_before(void) 
{
    /* uint32_t i; */
    
    /* Stop UARTs */
    /*
#if defined(CPU_FAM_NRF52)
    for (i = 0; i < UART_NUMOF; i++) {
        uart_config[i].dev->TASKS_STOPRX = 1;
        uart_config[i].dev->TASKS_STOPTX = 1;
    
        uart_config[i].dev->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
    }
#else
    NRF_UART0->TASKS_SUSPEND = 1;
#endif
    */

    /* Stop timers */
    /*
    for (i = 0; i < TIMER_NUMOF; i++) {
        timer_config[i].dev->TASKS_STOP = 1;
    }
    */

    clock_stop_hf();
}

static inline void _pm_after(void) 
{
    /* uint32_t i; */
    
    /* Start UARTs */
    /*
#if defined(CPU_FAM_NRF52)
    for (i = 0; i < UART_NUMOF; i++) {
        uart_config[i].dev->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

        uart_config[i].dev->TASKS_STARTRX = 1;
    }
#else
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
#endif
    */

    /* Stop timers */
    /*
    for (i = 0; i < TIMER_NUMOF; i++) {
        timer_config[i].dev->TASKS_START = 1;
    }
    */
}

void pm_off(void)
{
#ifdef CPU_FAM_NRF51
    NRF_POWER->RAMON = 0;
#else
    for (int i = 0; i < 8; i++) {
        NRF_POWER->RAM[i].POWERCLR = (POWER_RAM_POWERCLR_S1RETENTION_Msk |
                                      POWER_RAM_POWERCLR_S0RETENTION_Msk);
    }
#endif /* CPU_FAM_NRF51 */
    NRF_POWER->SYSTEMOFF = 1;
    while(1) {}
}

enum pm_mode pm_set(enum pm_mode mode)
{
    switch (mode) {
        case PM_POWERDOWN:
            /* Enable SystemOFF mode */	
            NRF_POWER->SYSTEMOFF = 1;
            __WFI();

            break;
        case PM_SLEEP:
            /* Enable constant latency mode */
            // NRF_POWER->TASKS_CONSTLAT = 1;

            /* Enable low power mode (variable latency) */
            NRF_POWER->TASKS_LOWPWR = 1;

            /* Set SLEEPDEEP bit of system control block */
            SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
            
            unsigned state = irq_disable();            
            _pm_before();
            
            #if defined (__CC_ARM)
                __force_stores();
            #endif

            __DSB();
            __WFI();
            
            /* Re-init clock after STOP */
            clock_init_hf();
            
            _pm_after();
            
            irq_restore(state);
            break;
        case PM_IDLE:
            break;
        case PM_ON:
            break;
        case PM_OFF:
            break;
        case PM_UNKNOWN:
            break;
    }

    return PM_UNKNOWN;
}

void pm_init(void) {
    /* do nothing */
}
