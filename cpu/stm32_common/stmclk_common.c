/*
 * Copyright (C) 2017 Freie Universität Berlin
 *               2017 OTA keys S.A.
 *               2018 Unwired Devices LLC <info@unwds.com>
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
 * @brief       Implementation of common STM32 clock configuration functions
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @}
 */

#include "cpu.h"
#include "stmclk.h"
#include "periph_conf.h"

#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7)
#define REG_PWR_CR          CR1
#define BIT_CR_DBP          PWR_CR1_DBP
#else
#define REG_PWR_CR          CR
#define BIT_CR_DBP          PWR_CR_DBP
#endif

#if defined (CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
#define REG_LSE             CSR
#define BIT_LSEON           RCC_CSR_LSEON
#define BIT_LSERDY          RCC_CSR_LSERDY
#else
#define REG_LSE             BDCR
#define BIT_LSEON           RCC_BDCR_LSEON
#define BIT_LSERDY          RCC_BDCR_LSERDY
#endif

#ifndef CLOCK_HSE
#define CLOCK_HSE   (0U)
#endif
#ifndef CLOCK_LSE
#define CLOCK_LSE   (0U)
#endif

void stmclk_enable_hsi(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
}

void stmclk_disable_hsi(void)
{
    /* we only disable the HSI clock if not used as input for the PLL and if
     * not used directly as system clock */
    if (CLOCK_HSE) {
        if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
            RCC->CR &= ~(RCC_CR_HSION);
        }
    }
}

void stmclk_enable_lfclk(void)
{
    if (CLOCK_LSE) {
        stmclk_dbp_unlock();
        RCC->REG_LSE |= BIT_LSEON;
        while (!(RCC->REG_LSE & BIT_LSERDY)) {}
        stmclk_dbp_lock();
    }
    else {
        RCC->CSR |= RCC_CSR_LSION;
        while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}
    }
}

void stmclk_disable_lfclk(void)
{
    if (CLOCK_LSE) {
        stmclk_dbp_unlock();
        RCC->REG_LSE &= ~(BIT_LSEON);
        while (!(RCC->REG_LSE & BIT_LSERDY)) {}
        stmclk_dbp_lock();
    }
    else {
        RCC->CSR &= ~(RCC_CSR_LSION);
    }
}

void stmclk_dbp_unlock(void)
{
    PWR->REG_PWR_CR |= BIT_CR_DBP;
}

void stmclk_dbp_lock(void)
{
    PWR->REG_PWR_CR &= ~(BIT_CR_DBP);
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
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 20*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 80*1024);
#elif defined(CPU_FAM_STM32F0)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 32*1024);
#elif defined(CPU_FAM_STM32F1)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 96*1024);
#elif defined(CPU_FAM_STM32F2)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 128*1024);
#elif defined(CPU_FAM_STM32F3)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 80*1024);
#elif defined(CPU_FAM_STM32F4)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 384*1024);
#elif defined(CPU_FAM_STM32F7)
    return cpu_find_memory_size((char *)SRAM_BASE, 1024, 512*1024);
#else
#error unexpected MCU
#endif
}

uint32_t get_cpu_flash_size(void) {
#if defined(CPU_FAM_STM32L0)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 192*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 512*1024);
#elif defined(CPU_FAM_STM32F0)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 256*1024);
#elif defined(CPU_FAM_STM32F1)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 1024*1024);
#elif defined(CPU_FAM_STM32F2)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 1024*1024);
#elif defined(CPU_FAM_STM32F3)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 512*1024);
#elif defined(CPU_FAM_STM32F4)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 2048*1024);
#elif defined(CPU_FAM_STM32F7)
    return cpu_find_memory_size((char *)FLASH_BASE, 1024, 2048*1024);
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
    return 0;
#endif
}
