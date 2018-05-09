/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cortexm_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       common periph/pm functions
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include "cpu.h"
#include "core_cm0plus.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "periph/pm.h"
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

volatile int pm_prevent_sleep = 1;
volatile int pm_prevent_switch = 1;

volatile int pm_run_mode;

/* put GPIOs in low-power state */
static void pm_before_i_go_to_sleep (void) {
    /* seems nothing to do here */
}


/* restore GPIO settings */
static void pm_when_i_wake_up (void) {
    /* seems nothing to do here */
}

/* Do not change GPIO state in sleep mode */
void pm_add_gpio_exclusion(gpio_t gpio) {
	/* seems nothing to do here, to be removed */
}

/* Change GPIO state to AIN in sleep mode */
void pm_del_gpio_exclusion(gpio_t gpio) {
	/* seems nothing to do here, to be removed */
}

/* Select CPU clocking between default (PM_ON) and medium-speed (PM_IDLE) */
static void pm_select_run_mode(uint8_t pm_mode) {
	switch(pm_mode) {
		case PM_ON:
            DEBUG("Switching to PM_ON");
			clk_init()
			break;
		case PM_IDLE:
            DEBUG("Switching to PM_IDLE");
            /* 115200 bps stdio UART with default 16x oversamplig needs 2 MHz or 4 MHz MSI clock */
            /* at 1 MHz, it will be switched to 8x oversampling with 3.55 % baudrate error */
            /* if you need stdio UART at lower frequencies, change its settings to lower baudrate */
			switch_to_msi(RCC_ICSCR_MSIRANGE_5, RCC_CFGR_HPRE_DIV1);
			break;
		default:
            DEBUG("Switching to PM_IDLE");
			clk_init();
		break;
	}
    
#if defined(XTIMER_PRESENT)
    /* Recalculate xtimer frequency */
    /* NB: default XTIMER_HZ clock is 1 MHz, so CPU clock must be at least 1 MHz for xtimer to work properly */
    xtimer_init();
#endif

#if defined(UART_STDIO_DEV) && defined(UART_STDIO_BAUDRATE)
    /* Recalculate stdio UART baudrate */
    uart_set_baudrate(UART_STDIO_DEV, UART_STDIO_BAUDRATE);
#endif
}

void pm_set_lowest(void)
{
    if (!pm_prevent_sleep) {
        pm_set(PM_POWERDOWN);
    }
}

void pm_init(void)
{
    pm_run_mode = PM_ON;
	
    /* Unlock the RUN_PD bit to change flash settings */  
    FLASH->PDKEYR = FLASH_PDKEY1;
    FLASH->PDKEYR = FLASH_PDKEY2;
    /* Enable flash power down during sleep */
    FLASH->ACR |= FLASH_ACR_SLEEP_PD;
}

enum pm_mode pm_set(enum pm_mode target)
{
    switch (target) {
        case PM_SLEEP:               /* Low-power sleep mode */
            DEBUG("Switching to PM_SLEEP");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Enable low-power mode of the voltage regulator */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Clear SLEEPDEEP bit */
            SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

            irq_disable();
            
            pm_before_i_go_to_sleep();

            /* Switch to 65kHz clock */
            switch_to_msi(RCC_ICSCR_MSIRANGE_0, RCC_CFGR_HPRE_DIV1);

            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
            
            /* Switch back to default speed */
			pm_select_run_mode(pm_run_mode);

            pm_when_i_wake_up();
          
            irq_enable();
            break;

        case PM_POWERDOWN:         /* STOP mode */
            DEBUG("Switching to PM_POWERDOWN");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Regulator in LP mode */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Enable Ultra Low Power mode */
            PWR->CR |= PWR_CR_ULP;
            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;
            
            /* Enable debug in STOP mode */
            /* DBGMCU->CR |= DBGMCU_CR_DBG_STOP; */

            irq_disable();

            pm_before_i_go_to_sleep();
           
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();

            /* Clear SLEEPDEEP bit */
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
            
            /* Restore clocks and PLL */
            /* (MCU is running on MSI clock after STOP) */
			pm_select_run_mode(pm_run_mode);
            
            pm_when_i_wake_up();

			irq_enable();

            break;

        case PM_OFF:               /* Standby mode */
            DEBUG("Switching to PM_OFF");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
        
            /* Select STANDBY mode */
            PWR->CR |= PWR_CR_PDDS;

            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;

            /* Enable Ultra Low Power mode */
            PWR->CR |= PWR_CR_ULP;

            /* This option is used to ensure that store operations are completed */
            #if defined (__CC_ARM)
            __force_stores();
            #endif
            
            irq_disable();
            
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
            break;

        case PM_UNKNOWN:
			break;
        case PM_ON:
            DEBUG("Switching to PM_ON");
			irq_disable();
			pm_run_mode = PM_ON;
			pm_select_run_mode(pm_run_mode);
			irq_enable();
			break;
        case PM_IDLE:
            if (!pm_prevent_switch) {
                DEBUG("Switching to PM_IDLE");
                irq_disable();
                pm_run_mode = PM_IDLE;
                pm_select_run_mode(pm_run_mode);
                irq_enable();
            }
			break;
        default:
            break;
    }

    return 0;
}

enum pm_mode pm_get(void)
{
    return pm_run_mode;
}