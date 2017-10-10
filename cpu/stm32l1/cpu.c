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
 * @brief       Implementation of the kernel cpu functions
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Nick van IJzendoorn <nijzendoorn@engineering-spirit.nl>
 * @author      Víctor Ariño <victor.arino@zii.aero>
 * @author      Oleg Artamonov <info@unwds.com>
 *
 * @}
 */

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/timer.h"
#include "periph/rtc.h"

/* See if we want to use the PLL */
#if defined(CLOCK_PLL_DIV) || defined(CLOCK_PLL_MUL)
#define CLOCK_USE_PLL              1
#else
#define CLOCK_USE_PLL              0
#endif

/* Check the source to be used for the PLL */
#if defined(CLOCK_HSI) && defined(CLOCK_HSE)
#error "Only provide one of two CLOCK_HSI/CLOCK_HSE"
#elif (CLOCK_USE_PLL == 1) && (!defined(CLOCK_PLL_MUL) || !defined(CLOCK_PLL_DIV))
#error "When using PLL both CLOCK_PLL_DIV and CLOCK_PLL_MUL must be provided"

#elif CLOCK_HSI
#define CLOCK_CR_SOURCE            RCC_CR_HSION
#define CLOCK_CR_SOURCE_RDY        RCC_CR_HSIRDY
#define CLOCK_PLL_SOURCE           RCC_CFGR_PLLSRC_HSI
#define CLOCK_DISABLE_OTHERS       (RCC_CR_HSEON | RCC_CR_MSION)

#if (CLOCK_USE_PLL == 0)
#define CLOCK_CFGR_SW              RCC_CFGR_SW_HSI
#define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_HSI
#endif

#elif CLOCK_HSE
#define CLOCK_CR_SOURCE            RCC_CR_HSEON
#define CLOCK_CR_SOURCE_RDY        RCC_CR_HSERDY
#define CLOCK_PLL_SOURCE           RCC_CFGR_PLLSRC_HSE
#define CLOCK_DISABLE_OTHERS       (RCC_CR_HSION | RCC_CR_MSION)

#if (CLOCK_USE_PLL == 0)
#define CLOCK_CFGR_SW              RCC_CFGR_SW_HSE
#define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_HSE
#endif

#elif CLOCK_MSI
#define CLOCK_CR_SOURCE            RCC_CR_MSION
#define CLOCK_CR_SOURCE_RDY        RCC_CR_MSIRDY
#define CLOCK_CFGR_SW              RCC_CFGR_SW_MSI
#define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_MSI
#define CLOCK_DISABLE_OTHERS       (RCC_CR_HSEON | RCC_CR_HSION)
#define CLOCK_MSIRANGE             RCC_ICSCR_MSIRANGE_6

#if (CLOCK_USE_PLL == 1)
#error "PLL can't be used with MSI"
#endif

#else
#error "Please provide CLOCK_HSI or CLOCK_HSE in boards/NAME/includes/perhip_cpu.h"
#endif

#if (CLOCK_USE_PLL == 1)
#define CLOCK_CFGR_SW              RCC_CFGR_SW_PLL
#define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_PLL
#endif

#if (CLOCK_CORECLOCK > 16000000U)
#define CORE_VOLTAGE PWR_CR_VOS_0
#elif (CLOCK_CORECLOCK > 8000000U)
#define CORE_VOLTAGE PWR_CR_VOS_1
#else
#define CORE_VOLTAGE (PWR_CR_VOS_1 | PWR_CR_VOS_0)
#endif

static uint32_t tmpreg;
volatile uint32_t cpu_clock_global;

void (*SysMemBootJump)(void);

void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();
    
    /* Switching to bootloader if there's a magic number in RTC registers */
    /* Should be done before initializing anything but RTC */
    rtc_poweron();
    if (rtc_restore_backup(0) == 0xB00710AD) {
        /* clear RTC register */
        rtc_save_backup(0, 0);
        rtc_poweroff();
        
        /* System Memory on STM32L1 is at 0x1FF0 0000*/
        /* point the PC to the System Memory reset vector (+4) */
        SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1ff00004));
        SysMemBootJump();
    } else {
        /* if we need RTC we reenable it later, if we don't - we don't */
        rtc_poweroff();
    }
    
    /* initialize system clocks */
    clk_init();
    /* switch all GPIOs to AIN mode to minimize power consumption */
    uint8_t i;
    GPIO_TypeDef *port;
    for (i = 0; i < CPU_NUMBER_OF_PORTS; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        port->MODER = 0xffffffff;
    }
}

/**
 * @brief Configure the clock system of the stm32f1
 */
void clk_init(void)
{
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set MSION bit */
    RCC->CR |= RCC_CR_MSION;
    /* Reset SW, HPRE, PPRE1, PPRE2, MCOSEL and MCOPRE bits */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR &= ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);
    RCC->CFGR &= ~(RCC_CFGR_SW);
    /* Reset PLL bits */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL);
    /* Reset HSION, HSEON, CSSON and PLLON bits */
    RCC->CR &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);
    /* Clear all interrupts */
    RCC->CIR = 0x0;

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration */
    /* Enable high speed clock source */
    RCC->CR |= CLOCK_CR_SOURCE;
    /* Wait till the clock source is ready
     * NOTE: the MCU will stay here forever if you use an external clock source and it's not connected */
    while (!(RCC->CR & CLOCK_CR_SOURCE_RDY)) {}
    
    /* Choose the most efficient flash configuration */
#if (CLOCK_CORECLOCK > 8000000U)
    /* Enable 64-bit access, prefetch and 1 wait state */
    /* (at F > 8MHz/16MHz WS must be 1) */
    FLASH->ACR |= FLASH_ACR_ACC64;
    FLASH->ACR |= CLOCK_FLASH_LATENCY;
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    /* Wait for flash to become ready */
    while (!(FLASH->SR & FLASH_SR_READY)) {}
#else
    /* Set 0 wait state, 32-bit access and no prefetch */
    /* LATENCY and PRFTEN can be changed with 64-bit access enabled only */
    FLASH->ACR |= FLASH_ACR_ACC64;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR &= ~FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_ACC64;
    /* Wait for flash to become ready */
    while (!(FLASH->SR & FLASH_SR_READY)) {}
#endif

    /* Power domain enable */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    /* Select the Voltage Range */
    tmpreg = PWR->CR;
    tmpreg &= ~PWR_CR_VOS;
    tmpreg |= CORE_VOLTAGE;
    PWR->CR = tmpreg;
    /* Wait until the Voltage Regulator is ready */
    while((PWR->CSR & PWR_CSR_VOSF) != 0) {}

    /* Enable low-power run if permitted */
#if CLOCK_MSI
    if ((CLOCK_MSIRANGE == RCC_ICSCR_MSIRANGE_1) || (msi_range == RCC_ICSCR_MSIRANGE_0)) {
        PWR->CR |= PWR_CR_LPSDSR | PWR_CR_LPRUN;
    }
#endif

    /* set AHB, APB1 and APB2 clock dividers */
    tmpreg = RCC->CFGR;
    tmpreg &= ~RCC_CFGR_HPRE;
    tmpreg |= (uint32_t)CLOCK_AHB_DIV;
    tmpreg &= ~RCC_CFGR_PPRE1;
    tmpreg |= (uint32_t)CLOCK_APB1_DIV;
    tmpreg &= ~RCC_CFGR_PPRE2;
    tmpreg |= (uint32_t)CLOCK_APB2_DIV;
    RCC->CFGR = tmpreg;

#if CLOCK_USE_PLL
    /*  PLL configuration: PLLCLK = CLOCK_SOURCE / PLL_DIV * PLL_MUL */
    RCC->CFGR |= (uint32_t)(CLOCK_PLL_SOURCE | CLOCK_PLL_DIV | CLOCK_PLL_MUL);
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    /* Wait till PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}
#elif CLOCK_MSI
    tmpreg = RCC->ICSCR;
    tmpreg &= ~(RCC_ICSCR_MSIRANGE);
    tmpreg |= CLOCK_MSIRANGE;
    RCC->ICSCR = tmpreg;
#endif

    /* Select system clock source */
    tmpreg = RCC->CFGR;
    tmpreg &= ~RCC_CFGR_SW;
    tmpreg |= (uint32_t)CLOCK_CFGR_SW;;
    RCC->CFGR = tmpreg;
    /* Wait till clock is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != CLOCK_CFGR_SW_RDY) {}

    /* Disable other clock sources */
    RCC->CR &= ~(CLOCK_DISABLE_OTHERS);
    
    cpu_clock_global = CLOCK_CORECLOCK;
}

void switch_to_msi(uint32_t msi_range, uint32_t ahb_divider)
{
    RCC->CR |= RCC_CR_MSION;
    while (!(RCC->CR & RCC_CR_MSIRDY)) {}
    
    tmpreg = RCC->ICSCR;
    tmpreg &= ~RCC_ICSCR_MSIRANGE;
    tmpreg |= msi_range;
    RCC->ICSCR = tmpreg;

    tmpreg = RCC->CFGR;
    tmpreg &= ~RCC_CFGR_HPRE;
    tmpreg |= ahb_divider;
    tmpreg &= ~RCC_CFGR_SW;
    tmpreg |= RCC_CFGR_SW_MSI;
    RCC->CFGR = tmpreg;
    
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) {}

    if ((msi_range == RCC_ICSCR_MSIRANGE_1) || (msi_range == RCC_ICSCR_MSIRANGE_0)) {
        /* Low-power run is only allowed at MSI Range 0 and 1 */
        PWR->CR |= PWR_CR_LPSDSR | PWR_CR_LPRUN;
    } else {
        /* set Voltage Range 3 (1.2V) */
        PWR->CR |= (PWR_CR_VOS_1 | PWR_CR_VOS_0);
        while((PWR->CSR & PWR_CSR_VOSF) != 0) {}
    }

    /* Set latency = 0, disable prefetch and 64-bit access */
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR &= ~FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_ACC64;
    while (!(FLASH->SR & FLASH_SR_READY)) {}
    
    /* Disable high speed clock sources and PLL */
    tmpreg = RCC->CR;
    tmpreg &= ~(RCC_CR_HSION | RCC_CR_HSEON);
    tmpreg &= ~(RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);
    RCC->CR = tmpreg;
    
    cpu_clock_global = 65536 * (1 << (msi_range >> 13));
}

int get_cpu_category(void) {
    switch (ST_DEV_ID) {
        case STM32L1_DEV_ID_CAT1:
            return 1;
            break;
        case STM32L1_DEV_ID_CAT2:
            return 2;
            break;
        case STM32L1_DEV_ID_CAT3:
            return 3;
            break;
        case STM32L1_DEV_ID_CAT4:
            return 4;
            break;
        case STM32L1_DEV_ID_CAT56:
            return 5;
            break;
    }
    return 0;
}

int get_cpu_ram_size(void) {
    return 0;
}

int get_cpu_flash_size(void) {
    return 0;
}

/* doesn't work with STM32L100 */
int get_cpu_eeprom_size(void) {
    switch (ST_DEV_ID) {
        case STM32L1_DEV_ID_CAT1:
            return 4096;
            break;
        case STM32L1_DEV_ID_CAT2:
            return 4096;
            break;
        case STM32L1_DEV_ID_CAT3:
            return 8192;
            break;
        case STM32L1_DEV_ID_CAT4:
            return 12288;
            break;
        case STM32L1_DEV_ID_CAT56:
            return 16384;
            break;
    }
    return 0;
}
