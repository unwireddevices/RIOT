/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @{
 *
 * @file
 * @brief       Implementation of the kernel's lpm interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "arch/lpm_arch.h"

#include "stm32l1xx.h"
#include "xtimer.h"

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"

#define CR_DS_MASK               ((uint32_t)0xFFFFFFFC)

/* Ultra Low Power mode definitions */
#define PWR_OFFSET               (PWR_BASE - PERIPH_BASE)
#define CR_OFFSET                (PWR_OFFSET + 0x00)
#define ULP_BitNumber           0x09
#define CR_ULP_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (ULP_BitNumber * 4))

static inline void select_msi_range(uint32_t range)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->ICSCR;

    /* Clear MSIRANGE[2:0] bits */
    tmpreg &= ~RCC_ICSCR_MSIRANGE;

    /* Set the MSIRANGE[2:0] bits according to RCC_MSIRange value */
    tmpreg |= (uint32_t)range;

    /* Store the new value */
    RCC->ICSCR = tmpreg;
}

static inline void set_sysclk_msi(void)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR;

    /* Clear SW[1:0] bits */
    tmpreg &= ~RCC_CFGR_SW;

    /* Store the new value */
    RCC->CFGR = tmpreg;
}

static inline void setup_flash_for_msi(void)
{
    uint32_t tmp_acr;

    /*
     * Set the recommended flash settings for <= 2MHz clock.
     *
     * The 3 bits must be programmed strictly sequentially, but it
     * is faster not to read-back the value of the ACR register in
     * the middle of the sequence so use a temporary variable.
     */
    tmp_acr = FLASH->ACR;
    /* Flash 0 wait state */
    tmp_acr &= ~FLASH_ACR_LATENCY;
    FLASH->ACR = tmp_acr;
    /* Disable prefetch Buffer */
    tmp_acr &= ~FLASH_ACR_PRFTEN;
    FLASH->ACR = tmp_acr;
    /* Disable 64-bit access */
    tmp_acr &= ~FLASH_ACR_ACC64;
    FLASH->ACR = tmp_acr;
}

#define TIM TIM5

static void set_xtimer_freq(int freq) {
    /* configure reload and pre-scaler values */
    //TIM->ARR = 0xffffffff;
    //TIM->PSC = freq / XTIMER_USEC_TO_TICKS(1000000ul);

    //TIM->CCR[0] /= 8;
    //TIM->SR &= ~(1 << (0 + 1));
    //TIM->DIER |= (1 << (0 + 1));

    /* trigger update event to make pre-scaler value effective */
    //TIM->EGR = TIM_EGR_UG;
}

void switch_to_msi_1mhz(void)
{
    select_msi_range(RCC_ICSCR_MSIRANGE_6);

    /* Enable MSI */
    RCC->CR |= RCC_CR_MSION;

    /* Disable PLL and HSE and HSI */
    RCC->CR &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_PLLON);

    /* Regulator in LP mode */
    PWR->CR |= PWR_CR_LPSDSR;

    /* Set MSI as system clock */
    set_sysclk_msi();

    /* Configure FLASH */
    setup_flash_for_msi();

    /* Set xtimer for current frequency */
    set_xtimer_freq(1048000); /* 1.048 MHz */
}

void restore_clocks_hsi(void)
{
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set MSION bit */
    RCC->CR |= RCC_CR_MSION;
    /* Reset SW, HPRE, PPRE1, PPRE2, MCOSEL and MCOPRE bits */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL);
    /* Reset HSION, HSEON, CSSON and PLLON bits */
    RCC->CR &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);
    /* Disable all interrupts */
    RCC->CIR = 0x0;

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration */
    /* Enable high speed clock source */
    RCC->CR |= RCC_CR_HSION;
    /* Wait till the high speed clock source is ready
     * NOTE: the MCU will stay here forever if you use an external clock source and it's not connected */
    while (!(RCC->CR & RCC_CR_HSIRDY)) {
    }
    FLASH->ACR |= FLASH_ACR_ACC64;
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    /* Flash 1 wait state */
    FLASH->ACR |= CLOCK_FLASH_LATENCY;
    /* Power enable */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    /* Select the Voltage Range 1 (1.8 V) */
    PWR->CR = PWR_CR_VOS_0;
    /* Wait Until the Voltage Regulator is ready */
    while ((PWR->CSR & PWR_CSR_VOSF) != 0) {
    }
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)CLOCK_AHB_DIV;
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)CLOCK_APB2_DIV;
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)CLOCK_APB1_DIV;
    /*  PLL configuration: PLLCLK = CLOCK_SOURCE / PLL_DIV * PLL_MUL */
    RCC->CFGR &= ~((uint32_t)(RCC_CFGR_PLLSRC | RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI | CLOCK_PLL_DIV | CLOCK_PLL_MUL);
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    /* Wait till PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }
    /* Select PLL as system clock source */
    RCC->CFGR &= ~((uint32_t)(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    set_xtimer_freq(32000000);
}

void lpm_arch_init(void)
{
    /* TODO */
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
    switch (target) {
        case LPM_SLEEP:;                                            /* Sleep mode */
            PWR->CR = (PWR->CR & CR_DS_MASK) | PWR_CR_LPSDSR;
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP);  /* Clear SLEEPDEEP bit of Cortex System Control Register */

            /* Request Wait For Interrupt */
            __disable_irq();
            switch_to_msi_1mhz();

            asm ("DMB");
            __WFI();
            asm ("nop");

            restore_clocks_hsi();
            __enable_irq();
            break;

        case LPM_POWERDOWN:         /* Stop mode */
            /* Regulator in LP mode */
            PWR->CR = (PWR->CR & CR_DS_MASK) | PWR_CR_LPSDSR;

            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= SCB_SCR_SLEEPDEEP;

            /* Wait in sleep mode until interrupt */
            __disable_irq();
            //switch_to_msi_1mhz();

            asm ("DMB");
            __WFI();
            asm ("nop");

            //restore_clocks_hsi();
            __enable_irq();

            /* Clear SLEEPDEEP bit */
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP);
            break;

        case LPM_OFF:               /* Standby mode */
            /* Select STANDBY mode */
            PWR->CR |= PWR_CR_PDDS;

            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= SCB_SCR_SLEEPDEEP;

            /* Enable Ultra Low Power mode */
            *(__IO uint32_t *) CR_ULP_BB = 1;

            /* This option is used to ensure that store operations are completed */
            #if defined (__CC_ARM)
            __force_stores();
            #endif
            /* Request Wait For Interrupt */
            __WFI();
            break;

        /* do nothing here */
        case LPM_UNKNOWN:
        case LPM_ON:
        case LPM_IDLE:
        default:
            break;
    }

    return 0;
}

enum lpm_mode lpm_arch_get(void)
{
    /* TODO */
    return 0;
}

void lpm_arch_awake(void)
{
    /* Disable Ultra Low Power mode */
    *(__IO uint32_t *) CR_ULP_BB = 0;

    PWR->CR &= (uint32_t) ~((uint32_t)PWR_CR_LPRUN);
    PWR->CR &= (uint32_t) ~((uint32_t)PWR_CR_LPSDSR);
}

void lpm_arch_begin_awake(void)
{
    /* TODO */
}

void lpm_arch_end_awake(void)
{
    /* TODO */
}
