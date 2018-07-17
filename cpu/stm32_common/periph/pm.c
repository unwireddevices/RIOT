/*
 * Copyright (C) 2018 OTA keys S.A.
 *               2016 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 Freie Universität Berlin
 *               2015 Engineering-Spirit
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       Implementation of the kernels power management interface
 *
 * @author      Nick v. IJzendoorn <nijzndoorn@engineering-spirit.nl>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include "irq.h"
#include "periph/pm.h"

#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
    defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L4)

#include "stmclk.h"
#include "periph_cpu_common.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef PM_STOP_CONFIG
/**
 * @brief Define config flags for stop mode
 *
 * Available values can be found in reference manual, PWR section, register CR.
 */
#if defined(CPU_FAM_STM32F0)
#define PM_STOP_CONFIG (PWR_CR_LPDS)
#else
#define PM_STOP_CONFIG (PWR_CR_LPDS | PWR_CR_FPDS)
#endif
#endif

static inline const uint32_t _ewup_config(void)
{
    uint32_t tmp = 0;
#ifdef PM_EWUP_CONFIG
    tmp |= PM_EWUP_CONFIG;
#elif defined(PWR_CSR_EWUP)
    tmp |= PWR_CSR_EWUP;
#else
#if defined(PWR_CSR_EWUP8)
    tmp |= PWR_CSR_EWUP8;
#endif
#if defined(PWR_CSR_EWUP7)
    tmp |= PWR_CSR_EWUP7;
#endif
#if defined(PWR_CSR_EWUP6)
    tmp |= PWR_CSR_EWUP6;
#endif
#if defined(PWR_CSR_EWUP5)
    tmp |= PWR_CSR_EWUP5;
#endif
#if defined(PWR_CSR_EWUP4)
    tmp |= PWR_CSR_EWUP4;
#endif
#if defined(PWR_CSR_EWUP3)
    tmp |= PWR_CSR_EWUP3;
#endif
#if defined(PWR_CSR_EWUP2)
    tmp |= PWR_CSR_EWUP2;
#endif
#if defined(PWR_CSR_EWUP1)
    tmp |= PWR_CSR_EWUP1;
#endif
#endif
    return tmp;
}

static uint8_t powermode;

enum pm_mode pm_set(enum pm_mode mode)
{
    int deep = 0;
    
    powermode = mode;

    switch (mode) {
        case PM_POWERDOWN:
#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
    defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L4)
            /* Set PDDS to enter standby mode on deepsleep and clear flags */
            PWR->CR |= (PWR_CR_PDDS | PWR_CR_CWUF | PWR_CR_CSBF);
            
#if defined(CPU_FAM_STM32L0) || defined (CPU_FAM_STM32L1)
            /* Disable Vrefint in standby mode */
            PWR->CR |= PWR_CR_ULP;
            
#if defined(CPU_FAM_STM32L4)
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STANDBY;
#endif

            /* Enable WKUP pin to use for wakeup from standby mode */
<<<<<<< HEAD
            PWR->CSR |= PM_EWUP_CONFIG;

=======
            PWR->CSR |= _ewup_config();
>>>>>>> 5ab8060... fixup! fixup! cpu/stm32f0: add periph_pm support
            /* Set SLEEPDEEP bit of system control block */
            deep = 1;
            
            cortexm_sleep(deep);
            break;
        case PM_SLEEP:
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
            /* Clear PDDS to enter stop mode on */
            PWR->CR &= ~(PWR_CR_PDDS);
            
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            
            /* Voltage regulator in LP mode */
            PWR->CR |= PWR_CR_LPSDSR;
            
            /* Disable Vrefint in stop mode */
            PWR->CR |= PWR_CR_ULP;
#if defined(CPU_FAM_STM32L0)
            /* set to 0 to select MSI as wakeup clock */
            /* set to 1 to select HSI16 */
            RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
#endif
#elif defined(CPU_FAM_STM32L4)
            /* Set STOP 0 mode by default */
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STOP0;
#else   /* STM32Fxxx series */
            /* Clear PDDS and LPDS bits to enter stop mode on */
            /* deepsleep with voltage regulator on */
            PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS);
            PWR->CR |= PM_STOP_CONFIG;
#endif
            /* Set SLEEPDEEP bit of system control block */
            deep = 1;
            
            cortexm_sleep(deep);
            break;
        case PM_IDLE: {
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
            /* 115200 bps stdio UART with default 16x oversamplig needs 2 MHz or 4 MHz MSI clock */
            /* at 1 MHz, it will be switched to 8x oversampling with 3.55 % baudrate error */
            /* if you need stdio UART at lower frequencies, change its settings to lower baudrate */
            unsigned state = irq_disable();
			switch_to_msi(RCC_ICSCR_MSIRANGE_5, RCC_CFGR_HPRE_DIV1);
            irq_restore(state);
#else   /* STM32Fxxx series */
#endif
            break;
        }
        case PM_ON:
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
            /* switching back to default speed */
            stmclk_init_sysclk();
#else   /* STM32Fxxx series */
#endif
            break;
        case PM_OFF:
            break;
        case PM_UNKNOWN:
            break;
    }

#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
    defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L4)
    if (deep) {
        /* Re-init clock after STOP */
        stmclk_init_sysclk();
    }
#endif
    return PM_UNKNOWN;
}

enum pm_mode pm_get(void) {
    return powermode;
}

void pm_init(void) {
    /* Nothing to do here yet */
}

#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
    defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L4)
void pm_off(void)
{
    irq_disable();
    pm_set(PM_OFF);
}
#endif
