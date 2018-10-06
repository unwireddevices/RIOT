/*
 * Copyright (C) 2018 Unwired Devices LLC <info@unwds.com>
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
 * @brief       Implementation of STM32 CPU identification and status data
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include "cpu.h"
#include <string.h>

static cpu_status_t cpu_status;

static uint32_t get_cpu_category(void) {
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
#elif defined(CPU_FAM_STM32F0)
    switch (ST_DEV_ID) {
        case STM32F0_DEV_ID_CAT3:
            return 3;
        case STM32F0_DEV_ID_CAT4:
            return 4;
        case STM32F0_DEV_ID_CAT5:
            return 5;
        case STM32F0_DEV_ID_CAT7:
            return 7;
        case STM32F0_DEV_ID_CAT9:
            return 9;
        default:
            return 0;
    }
    return 0;
#elif defined(CPU_FAM_STM32F1)
    
#elif defined(CPU_FAM_STM32F2)
    
#elif defined(CPU_FAM_STM32F3)
    
#elif defined(CPU_FAM_STM32F4)
    
#elif defined(CPU_FAM_STM32F7)
    
#else
#error unexpected MCU
#endif
}

static char get_cpu_memory_code(uint32_t memory_size) {
    switch (memory/1024) {
        case (8):
            return '3';
        case (16):
            return '4';
        case (32):
            return '6';
        case (64):
            return '8';
        case (128):
            return 'B';
        case (192):
            return 'Z';
        case (256):
            return 'C';
        case (384):
            return 'D';
        case (512):
            return 'E';
        case (768):
            return 'F';
        case (1024):
            return 'G';
    }
    
    return 'x';
}

static uint32_t get_cpu_name(char *name) {
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
    
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
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
    
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    
    if (ST_DEV_ID == STM32L1_DEV_ID_CAT2) {
        snprintf(name, CPU_NAME_MAX_SIZE, "STM32L%03dx%c-A", series, model);
    } else {
        snprintf(name, CPU_NAME_MAX_SIZE, "STM32L%03dx%c", series, model);
    }
    return 0;
#elif defined(CPU_FAM_STM32F0)
    /* STM32F0x0 is the default series */
    series = 0;

    /* only STM32F0x1 and F0x2 have CAN */
    #if defined(CAN_BASE)
        if (check_cpu_address(CAN->MCR) {
            #if defined (USART8_BASE)
                /* STM32F0x1 has 8 USARTS */
                if (check_cpu_address(USART8->CR) {
                    series = 1;
                } else {
                    /* STM32F0x2 has 4 USARTS */
                    series = 2;
                }
            #else
                /* STM32F0x2 has 4 USARTS */
                series = 2;
            #endif
        } else {
            #if defined (USART8_BASE)
                /* STM32F0x8 has 8 USARTS and no CAN */
                if (check_cpu_address(USART8->CR) {
                    series = 8;
                }
            #endif
        }
    }
    #elif defined(USART8_BASE)
        /* STM32F0x8 has 8 USARTs and no CAN */
        if (check_cpu_address(USART8->CR) {
            series = 8;
        }
    #endif

    series += (get_cpu_category() * 10);

    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return 0;
#elif defined(CPU_FAM_STM32F1)
    #if defined(STM32F100xB) || defined(STM32F100xE)
        series = 100;
    #elif defined(STM32F101x6) || defined(STM32F101xB) || \
          defined(STM32F101xE) || defined(STM32F101xG)
        series = 101;
    #elif defined(STM32F102x6) || defined(STM32F102xB)
        series = 102;
    #elif defined(STM32F103x6) || defined(STM32F103xB) || \
          defined(STM32F103xE) || defined(STM32F103xG)
        series = 103;
    #elif defined(STM32F105xC)
        series = 105;
    #elif defined(STM32F107xC)
        series = 107;
        
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return 0;
#elif defined(CPU_FAM_STM32F2)
    
#elif defined(CPU_FAM_STM32F3)
    
#elif defined(CPU_FAM_STM32F4)
    
#elif defined(CPU_FAM_STM32F7)
    
#else
#error unexpected MCU
#endif
}

static size_t cpu_find_memory_size(char *base, uint32_t block, uint32_t maxsize) {
    char *address = base;
    do {
        address += block;
        if (!cpu_check_address(address)) {
            break;
        }
    } while ((size_t)(address - base) < maxsize);

    return (size_t)(address - base);
}

static size_t get_cpu_ram_size(void) {
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

static size_t get_cpu_flash_size(void) {
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

static size_t get_cpu_eeprom_size(void) {
#if defined(CPU_FAM_STM32L0)
    return cpu_find_memory_size((char *)DATA_EEPROM_BASE, 512, 6*1024);
#elif defined(CPU_FAM_STM32L1)
    return cpu_find_memory_size((char *)EEPROM_BASE, 2*1024, 16*1024);
#else
    return 0;
#endif
}

void cpu_init_status(void) {
    cpu_status.ram_size = get_cpu_ram_size();
    cpu_status.flash_size = get_cpu_flash_size();
    cpu_status.eeprom_size = get_cpu_eeprom_size();
    cpu_status.core_clock = cpu_clock_global;
    cpu_status.flashpage_size = FLASHPAGE_SIZE;
    cpu_status.flashpage_num = FLASHPAGE_NUMOF;
    cpu_status.vdd_voltage = INT16_MIN;
    cpu_status.vdda_voltage = INT16_MIN;
    cpu_status.vbat_voltage = INT16_MIN;
    cpu_status.core_temp = INT16_MIN;
    get_cpu_name(cpu_status.name);
    cpu_status.category = get_cpu_category();
}

cpu_status_t cpu_get_status(void) {
    return cpu_status;
}
