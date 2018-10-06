/*
 * Copyright (C) 2014 Freie Universität Berlin
 *               2017 Inria
 *               2018 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @{
 *
 * @file
 * @brief       Implementation of STM32 clock configuration
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @}
 */

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/init.h"
#include <string.h>

#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)

/* See if we want to use the PLL */
#if defined(CLOCK_PLL_DIV) || defined(CLOCK_PLL_MUL) || \
    defined(CLOCK_PLL_DIV_HSE) || defined(CLOCK_PLL_MUL_HSE) || \
    defined(CLOCK_PLL_DIV_HSI) || defined(CLOCK_PLL_MUL_HSI)
    
    #define CLOCK_USE_PLL              1
#else
    #define CLOCK_USE_PLL              0
#endif

#if !defined(CLOCK_HSI) && !defined(CLOCK_HSE) && !defined(CLOCK_MSI)
    #error "Please provide clock source in board.h or periph_conf.h file"
#endif

/* Check the source to be used for the PLL */
#if defined(CLOCK_HSI) && defined(CLOCK_HSE)
    #define CLOCK_HS_MULTI
#endif

#if !defined(CLOCK_HS_MULTI)
    #if defined(CLOCK_HSI)
        #define CLOCK_CR_SOURCE            RCC_CR_HSION
        #define CLOCK_CR_SOURCE_RDY        RCC_CR_HSIRDY
        #define CLOCK_PLL_SOURCE           RCC_CFGR_PLLSRC_HSI
        #define CLOCK_DISABLE_OTHERS       (RCC_CR_HSEON | RCC_CR_MSION)

        #if (CLOCK_USE_PLL == 0)
            #define CLOCK_CFGR_SW              RCC_CFGR_SW_HSI
            #define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_HSI
        #else
            #define CLOCK_CFGR_SW              RCC_CFGR_SW_PLL
            #define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_PLL
        #endif
    #elif defined(CLOCK_HSE)
        #define CLOCK_CR_SOURCE            RCC_CR_HSEON
        #define CLOCK_CR_SOURCE_RDY        RCC_CR_HSERDY
        #define CLOCK_PLL_SOURCE           RCC_CFGR_PLLSRC_HSE
        #define CLOCK_DISABLE_OTHERS       (RCC_CR_HSION | RCC_CR_MSION)

        #if (CLOCK_USE_PLL == 0)
            #define CLOCK_CFGR_SW              RCC_CFGR_SW_HSE
            #define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_HSE
        #else
            #define CLOCK_CFGR_SW              RCC_CFGR_SW_PLL
            #define CLOCK_CFGR_SW_RDY          RCC_CFGR_SWS_PLL
        #endif
    #endif
#endif

#if (CLOCK_CORECLOCK > 16000000U)
    #define CORE_VOLTAGE PWR_CR_VOS_0
#elif (CLOCK_CORECLOCK > 8000000U)
    #define CORE_VOLTAGE PWR_CR_VOS_1
#else
    #define CORE_VOLTAGE (PWR_CR_VOS_1 | PWR_CR_VOS_0)
#endif

static volatile uint32_t clock_source_rdy = 0;
volatile uint32_t cpu_clock_global;
volatile uint32_t cpu_ports_number = 3;
char cpu_clock_source[10] = { 0 };

/**
 * @brief Configure the controllers clock system
 *
 * The clock initialization make the following assumptions:
 * - the external HSE clock from an external oscillator is used as base clock
 * - the internal PLL circuit is used for clock refinement
 *
 * Use the following formulas to calculate the needed values:
 *
 * SYSCLK = ((HSE_VALUE / CLOCK_PLL_M) * CLOCK_PLL_N) / CLOCK_PLL_P
 * USB, SDIO and RNG Clock =  ((HSE_VALUE / CLOCK_PLL_M) * CLOCK_PLL_N) / CLOCK_PLL_Q
 *
 * The actual used values are specified in the board's `periph_conf.h` file.
 *
 * NOTE: currently there is not timeout for initialization of PLL and other locks
 *       -> when wrong values are chosen, the initialization could stall
 */
void stmclk_init_sysclk(void)
{
    uint32_t tmpreg;
    
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set MSION bit */
    RCC->CR |= RCC_CR_MSION;
    /* Reset SW, HPRE, PPRE1, PPRE2, MCOSEL and MCOPRE bits */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR &= ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);
    RCC->CFGR &= ~(RCC_CFGR_SW);
    /* Reset HSION, HSEON, CSSON and PLLON bits */
    RCC->CR &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);
    /* Disable all interrupts */

#if defined(CPU_FAM_STM32L0)
    RCC->CICR = 0x0;
#elif defined(CPU_FAM_STM32L1)
    RCC->CIR = 0x0;
#else
#error unexpected MCU
#endif

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration */
#if defined(CLOCK_HS_MULTI)   
    /* MCU after reboot or poweron */
    switch (clock_source_rdy) {
        case RCC_CR_HSERDY:
            RCC->CR |= RCC_CR_HSEON;
            break;
        case RCC_CR_HSIRDY:
            RCC->CR |= RCC_CR_HSION;
            break;
        default:
            RCC->CR |= RCC_CR_HSEON;
            clock_source_rdy = RCC_CR_HSERDY;
            
            volatile int timeout = 0;
            while (!(RCC->CR & clock_source_rdy)) {
                timeout++;
                if (timeout > 10000) {
                    RCC->CR |= RCC_CR_HSION;
                    RCC->CR &= ~RCC_CR_HSEON;
                    clock_source_rdy = RCC_CR_HSIRDY;
                    break;
                }
            }
            break;
    }
    
    while (!(RCC->CR & clock_source_rdy)) {};
    
#else
    /* Enable high speed clock source */
    RCC->CR |= CLOCK_CR_SOURCE;
    /* Wait till the clock source is ready
     * NOTE: the MCU will stay here forever if you use an external clock source and it's not connected */
    while (!(RCC->CR & CLOCK_CR_SOURCE_RDY)) {}
#endif

/* Choose the most efficient flash configuration */
#if defined(CPU_FAM_STM32L1)
    FLASH->ACR |= FLASH_ACR_ACC64;
#endif
#if (CLOCK_CORECLOCK > 8000000U)
    /* (at F > 8MHz/16MHz WS must be 1) */    
    FLASH->ACR |= CLOCK_FLASH_LATENCY;
    FLASH->ACR |= FLASH_ACR_PRFTEN;
#else
    /* Set 0 wait state, 32-bit access and no prefetch */
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR &= ~FLASH_ACR_PRFTEN;
#if defined(CPU_FAM_STM32L1)
    FLASH->ACR &= ~FLASH_ACR_ACC64;
#endif
#endif
    /* Wait for flash to become ready */
    while (!(FLASH->SR & FLASH_SR_READY)) {}

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
    tmpreg |= CLOCK_AHB_DIV;
    tmpreg &= ~RCC_CFGR_PPRE1;
    tmpreg |= CLOCK_APB1_DIV;
    tmpreg &= ~RCC_CFGR_PPRE2;
    tmpreg |= CLOCK_APB2_DIV;
    RCC->CFGR = tmpreg;

#if CLOCK_USE_PLL
    /*  PLL configuration: PLLCLK = CLOCK_SOURCE / PLL_DIV * PLL_MUL */
    RCC->CFGR &= ~((uint32_t)(RCC_CFGR_PLLSRC | RCC_CFGR_PLLDIV | RCC_CFGR_PLLMUL));
    #if defined(CLOCK_HS_MULTI)
        if (clock_source_rdy == RCC_CR_HSERDY) {
            RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | CLOCK_PLL_DIV_HSE | CLOCK_PLL_MUL_HSE);
        } else {
            RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI | CLOCK_PLL_DIV_HSI | CLOCK_PLL_MUL_HSI);
        }
    #else
        RCC->CFGR |= (uint32_t)(CLOCK_PLL_SOURCE | CLOCK_PLL_DIV | CLOCK_PLL_MUL);
    #endif
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

#if defined(CLOCK_HS_MULTI)
    uint32_t clock_cfgr_sw;
    uint32_t clock_cfgr_sw_rdy;
    uint32_t clock_disable_clocks;

    if (clock_source_rdy == RCC_CR_HSERDY) {
        clock_cfgr_sw = RCC_CFGR_SW_HSE;
        clock_cfgr_sw_rdy = RCC_CFGR_SWS_HSE;
        clock_disable_clocks = RCC_CR_HSION | RCC_CR_MSION;
    } else {
        clock_cfgr_sw = RCC_CFGR_SW_HSI;
        clock_cfgr_sw_rdy = RCC_CFGR_SWS_HSI;
        clock_disable_clocks = RCC_CR_HSEON | RCC_CR_MSION;
    }
    
    if (CLOCK_USE_PLL) {
        clock_cfgr_sw = RCC_CFGR_SW_PLL;
        clock_cfgr_sw_rdy = RCC_CFGR_SWS_PLL;
    }
    
    tmpreg |= (uint32_t)clock_cfgr_sw;
    RCC->CFGR = tmpreg;
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != clock_cfgr_sw_rdy) {}
    RCC->CR &= ~(clock_disable_clocks);
#else
    tmpreg |= (uint32_t)CLOCK_CFGR_SW;

    RCC->CFGR = tmpreg;
    /* Wait till clock is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != CLOCK_CFGR_SW_RDY) {}

    /* Disable other clock sources */
    RCC->CR &= ~(CLOCK_DISABLE_OTHERS);
#endif
    
    cpu_clock_global = CLOCK_CORECLOCK;
    
#if CLOCK_MSI
    memcpy(cpu_clock_source, "MSI", 3);   
#elif defined(CLOCK_HS_MULTI)
    uint32_t n = 0;
    if (CLOCK_USE_PLL) {
        memcpy(cpu_clock_source, "PLL", 3);
        n += 3;
    }
    if (clock_source_rdy == RCC_CR_HSERDY) {
        memcpy(cpu_clock_source + n, "/HSE", 4);
    } else {
        memcpy(cpu_clock_source + n, "/HSI", 4);
    }
#elif defined(CLOCK_HSI)
    #if CLOCK_USE_PLL
        memcpy(cpu_clock_source, "PLL/HSI", 7);
    #elif
        memcpy(cpu_clock_source, "HSI", 3);
    #endif
#elif defined(CLOCK_HSE)
    #if CLOCK_USE_PLL
        memcpy(cpu_clock_source, "PLL/HSE", 7);
    #elif
        memcpy(cpu_clock_source, "HSE", 3);
    #endif
#endif
}

void switch_to_msi(uint32_t msi_range, uint32_t ahb_divider) {
    uint32_t tmpreg;
    
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
#if defined(CPU_FAM_STM32L1)
    FLASH->ACR &= ~FLASH_ACR_ACC64;
#endif
    while (!(FLASH->SR & FLASH_SR_READY)) {}
    
    /* Disable high speed clock sources and PLL */
    tmpreg = RCC->CR;
    tmpreg &= ~(RCC_CR_HSION | RCC_CR_HSEON);
    tmpreg &= ~(RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);
    RCC->CR = tmpreg;
    
    cpu_clock_global = 65536 * (1 << (msi_range >> 13));
}

static uint32_t cpu_find_memory_size(char *base, uint32_t block, uint32_t maxsize) {
    char *address = base;
    do {
        address += block;
        if (!cpu_check_address(address)) {
            break;
        }
    } while ((uint32_t)(address - base) < maxsize);

    return (uint32_t)(address - base);
}

uint32_t get_cpu_ram_size(void) {
#if defined(CPU_FAM_STM32L0)
    return cpu_find_memory_size((char *)SRAM_BASE, 2*1024, 20*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)SRAM_BASE, 4*1024, 80*1024);
#else
#error unexpected MCU
#endif
}

uint32_t get_cpu_flash_size(void) {
#if defined(CPU_FAM_STM32L0)
    return cpu_find_memory_size((char *)FLASH_BASE, 8*1024, 192*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)FLASH_BASE, 32*1024, 512*1024);
#else
#error unexpected MCU
#endif
}

uint32_t get_cpu_eeprom_size(void) {
#if defined(CPU_FAM_STM32L0)
    return cpu_find_memory_size((char *)DATA_EEPROM_BASE, 512, 6*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)EEPROM_BASE, 2*1024, 16*1024);
#else
#error unexpected MCU
#endif
}

uint32_t get_cpu_category(void) {
#if defined(CPU_FAM_STM32L0)
    switch (ST_DEV_ID) {
        case STM32L0_DEV_ID_CAT1:
            return 1;
        case STM32L0_DEV_ID_CAT2:
            return 2;
        case STM32L0_DEV_ID_CAT3:
            return 3;
        case STM32L0_DEV_ID_CAT5:
            return 5;
        default:
            return 0;
    }
    return 0;
#elif defined(CPU_FAM_STM32L1)
    switch (ST_DEV_ID) {
        case STM32L1_DEV_ID_CAT1:
            return 1;
        case STM32L1_DEV_ID_CAT2:
            return 2;
        case STM32L1_DEV_ID_CAT3:
            return 3;
        case STM32L1_DEV_ID_CAT4:
            return 4;
        case STM32L1_DEV_ID_CAT56:
            return 5;
        default:
            return 0;
    }
    return 0;
#else
#error unexpected MCU
#endif
}

uint32_t get_cpu_name(char *name) {
int series = 0;

#if defined(CPU_FAM_STM32L0)
    #if defined(AES_BASE)
        if (cpu_check_address((char *)AES->CR)) {
            series += 10;
        }
    #endif

    /* STM32L0x1 series doesn't have DAC */
    #if defined(DAC_BASE)
        if (cpu_check_address((char *)DAC->CR)) {
            /* STM32L0x2 or STM32L0x3 */
            series += 1;
        }
    #endif
    /* STM32L0x3 series with LCD interface */
    #if defined(LCD_BASE)
        if (cpu_check_address((char *)LCD->CR)) {
            /* STM32L0x3 */
            series += 1;
        }
    #endif
    
    uint32_t memory = get_cpu_flash_size()/1024;
    char model = 'x';
    switch (memory) {
        case (8):
            model = '3';
            break;
        case (16):
            model = '4';
            break;
        case (32):
            model = '6';
            break;
        case (64):
            model = '8';
            break;
        case (128):
            model = 'B';
            break;
        case (192):
            model = 'Z';
            break;
    }
    
    switch (ST_DEV_ID) {
        case STM32L0_DEV_ID_CAT1:
            series += 11;
            break;
        case STM32L0_DEV_ID_CAT2:
            series += 31;
            break;
        case STM32L0_DEV_ID_CAT3:
            series += 51;
            break;
        case STM32L0_DEV_ID_CAT5:
            series += 71;
            break;
    }

    sprintf(name, "STM32L%03dx%c", series, model);
    
    return 0;
#elif defined(CPU_FAM_STM32L1)
    /* STM32L100xx as default value */
    series = 100;
    
    /* only STM32L16x has AES */
    #if defined(AES_BASE)
        if (cpu_check_address((char *)AES->CR)) {
            /* STM32L16x */
            series += 10;
        }
    #endif

    /* STM32L100 series doesn't have comparators */
    #if defined(COMP_BASE)
        if (cpu_check_address((char *)COMP->CSR)) {
            /* STM32L15x or STM32L16x */
            series += 51;
        }
    #endif
    
    /* only STM32L1x2 series has LCD */
    #if defined(LCD_BASE)
        if (cpu_check_address((char *)LCD->CR)) {
            /* STM32L1x2 */
            series += 1;
        }
    #endif
    
    uint32_t memory = get_cpu_flash_size()/1024;
    char model = 'x';
    switch (memory) {
        case (32):
            model = '6';
            break;
        case (64):
            model = '8';
            break;
        case (128):
            model = 'B';
            break;
        case (256):
            model = 'C';
            break;
        case (384):
            model = 'D';
            break;
        case (512):
            model = 'E';
            break;
    }
    
    if (ST_DEV_ID == STM32L1_DEV_ID_CAT2) {
        sprintf(name, "STM32L%03dx%c-A", series, model);
    } else {
        sprintf(name, "STM32L%03dx%c", series, model);
    }
    return 0;
#else
#error unexpected MCU
#endif
}

#endif /* defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1) */
