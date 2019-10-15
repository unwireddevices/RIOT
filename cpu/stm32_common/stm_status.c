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

#if defined(MODULE_PERIPH_STATUS)
 
#include "cpu.h"
#include "periph_conf.h"
#include <string.h>

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    #include "periph/adc.h"
#endif

cpu_status_t cpu_status;

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
    return cpu_find_memory_size((char *)FLASH_EEPROM_BASE, 2*1024, 16*1024);
#else
    return 0;
#endif
}

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
#elif defined(CPU_FAM_STM32L4)
    /* not a category but CPU series */
    switch (ST_DEV_ID) {
        case STM32L4_DEV_ID_CAT1:
            return 1;
        case STM32L4_DEV_ID_CAT3:
            return 3;
        case STM32L4_DEV_ID_CAT5:
            return 5;
        case STM32L4_DEV_ID_CAT7:
            return 7;
        case STM32L4_DEV_ID_CATR:
            return 9;
        default:
            return 0;
    }
#elif defined(CPU_FAM_STM32F0)
    /* not a category but CPU series */
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
#elif defined(CPU_FAM_STM32F1)
    /* not a category but device density */
    switch (ST_DEV_ID) {
        case STM32F1_DEV_ID_LD:
            return 1;
        case STM32F1_DEV_ID_MD:
            return 2;
        case STM32F1_DEV_ID_MDVL:
            return 3;
        case STM32F1_DEV_ID_HD:
            return 4;
        case STM32F1_DEV_ID_HDVL:
            return 5;
        case STM32F1_DEV_ID_XL:
            return 6;
        case STM32F1_DEV_ID_CD:
            return 7;
        default:
            return 0;
    }
#elif defined(CPU_FAM_STM32F2)
    /* a single ID */
    switch (ST_DEV_ID) {
        case STM32F2_DEV_ID:
            return 1;
        default:
            return 0;
    }
#elif defined(CPU_FAM_STM32F3)
    /* not a category but device model */
    switch (ST_DEV_ID) {
        case STM32F3_DEV_ID_CAT1:
            return 1;
        case STM32F3_DEV_ID_CAT2:
            return 2;
        case STM32F3_DEV_ID_CAT3:
            return 3;
        case STM32F3_DEV_ID_CAT4:
            return 4;
        case STM32F3_DEV_ID_CAT5:
            return 5;
        default:
            return 0;
    }
#elif defined(CPU_FAM_STM32F4)
    /* not a category but device model */
    switch (ST_DEV_ID) {
        case STM32F4_DEV_ID_CAT1:
            return 1;
        case STM32F4_DEV_ID_CAT2:
            return 2;
        case STM32F4_DEV_ID_CAT3:
            return 3;
        case STM32F4_DEV_ID_CAT4:
            return 4;
        case STM32F4_DEV_ID_CAT5:
            return 5;
        case STM32F4_DEV_ID_CAT6:
            return 6;
        case STM32F4_DEV_ID_CAT7:
            return 7;
        case STM32F4_DEV_ID_CAT8:
            return 8;
        case STM32F4_DEV_ID_CAT9:
            return 9;
        case STM32F4_DEV_ID_CAT10:
            return 10;
        default:
            return 0;
    }
#elif defined(CPU_FAM_STM32F7)
    /* not a category but device model */
    switch (ST_DEV_ID) {
        case STM32F7_DEV_ID_CAT1:
            return 1;
        case STM32F7_DEV_ID_CAT2:
            return 2;
        case STM32F7_DEV_ID_CAT3:
            return 3;
        case STM32F7_DEV_ID_CAT4:
            return 4;
        default:
            return 0;
    }
#else
#error unexpected MCU
#endif
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
static char get_cpu_memory_code(uint32_t memory_size) {
    switch (memory_size/1024) {
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
        case (1536):
            return 'H';
        case (2048):
            return 'I';
    }
    
    return 'x';
}
#endif

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
static void get_cpu_name(char *name) {
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
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32L%03dx%c", series, model);
    
    return;

#elif defined(CPU_FAM_STM32L1)
    /* STM32L100xx as default value */
    series = 100;

#if 0 /* somehow this doesn't work */
    /* only STM32L16x has AES */
    #if defined(AES_BASE)
        if (cpu_check_address((char *)AES->CR)) {
            /* STM32L16x */
            series += 10;
        }
    #endif
#endif

    /* STM32L100 series doesn't have comparators */
    #if defined(COMP_BASE)
        if (cpu_check_address((char *)COMP->CSR)) {
            /* STM32L15x or STM32L16x */
            series += 51;
        }
    #endif

#if 0 /* somehow this doesn't work */    
    /* only STM32L1x2 series has LCD */
    #if defined(LCD_BASE)
        if (cpu_check_address((char *)LCD->CR)) {
            /* STM32L1x2 */
            series += 1;
        }
    #endif
#endif

    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    
    if (ST_DEV_ID == STM32L1_DEV_ID_CAT2) {
        snprintf(name, CPU_NAME_MAX_SIZE, "STM32L%03dx%c-A", series, model);
    } else {
        snprintf(name, CPU_NAME_MAX_SIZE, "STM32L%03dx%c", series, model);
    }
    return;
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

    return;
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
    #endif
        
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return;
#elif defined(CPU_FAM_STM32F2)
    #if defined(STM32F205xx)
        series = 205;
    #elif defined(STM32F215xx)
        series = 215;
    #elif defined(STM32F207xx)
        series = 207;
    #elif defined(STM32F217xx)
        series = 217;
    #else
        series = 200;
    #endif
    
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return;
#elif defined(CPU_FAM_STM32F3)
    series = 300;
    #if defined(STM32F301x8)
        series = 301;
    #elif defined(STM32F302x8) || defined(STM32F302xC) || \
          defined(STM32F302xE)
        series = 302;
    #elif defined(STM32F303x8) || defined(STM32F303xC) || \
          defined(STM32F303xE)
        series = 303;
    #elif defined(STM32F373xC)
        series = 373l
    #elif defined(STM32F334x8)
        series = 334;
    #elif defined(STM32F318xx)
        series = 318;
    #elif defined(STM32F328xx)
        series = 328;
    #elif defined(STM32F358xx)
        series = 358;
    #elif defined(STM32F378xx)
        series = 378;
    #elif defined(STM32F398xx)
        series = 398;
    #else
        series = 300;
    #endif
    
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return;
#elif defined(CPU_FAM_STM32F4)
    #if defined(STM32F405xx)
        series = 405;
    #elif defined(STM32F415xx)
        series = 415;
    #elif defined(STM32F407xx)
        series = 407;
    #elif defined(STM32F417xx)
        series = 417;
    #elif defined(STM32F427xx)
        series = 427;
    #elif defined(STM32F437xx)
        series = 437;
    #elif defined(STM32F429xx)
        series = 429;
    #elif defined(STM32F439xx)
        series = 439;
    #elif defined(STM32F401xC) || defined(STM32F401xE)
        series = 401;
    #elif defined(STM32F410Tx) || defined(STM32F410Cx) || \
          defined(STM32F410Rx)
        series = 410;
    #elif defined(STM32F411xE)
        series = 411;
    #elif defined(STM32F446xx)
        series = 446;
    #elif defined(STM32F469xx)
        series = 469;
    #elif defined(STM32F479xx)
        series = 479;
    #elif defined(STM32F412Cx) || defined(STM32F412Zx) || \
          defined(STM32F412Rx) || defined(STM32F412Vx)
        series = 412;
    #elif defined(STM32F413xx)
        series = 413;
    #elif defined(STM32F423xx)
        series = 423;
    #else
        series = 400;
    #endif
    
    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return;
    
#elif defined(CPU_FAM_STM32F7)
    #if defined(STM32F722xx)
        series = 722;
    #elif defined(STM32F723xx)
        series = 723;
    #elif defined(STM32F732xx)
        series = 732;
    #elif defined(STM32F733xx)
        series = 733;
    #elif defined(STM32F756xx)
        series = 756;
    #elif defined(STM32F746xx)
        series = 746;
    #elif defined(STM32F745xx)
        series = 745;
    #elif defined(STM32F765xx)
        series = 765;
    #elif defined(STM32F767xx)
        series = 767;
    #elif defined(STM32F769xx)
        series = 769;
    #elif defined(STM32F777xx)
        series = 777;
    #elif defined(STM32F779xx)
        series = 779;
    #else
        series = 700;
    #endif

    uint32_t memory = get_cpu_flash_size();
    char model = get_cpu_memory_code(memory);
    snprintf(name, CPU_NAME_MAX_SIZE, "STM32F%03dx%c", series, model);

    return;
#else
#error unexpected MCU
#endif
}
#endif /* MODULE_PERIPH_STATUS_EXTENDED */

static void get_cpu_flash(cpu_status_t* status) {
    status->flash.size = get_cpu_flash_size();
    
#if defined(CPU_FAM_STM32L0)
    status->flash.pagesize = 128;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 4;
    return;
#elif defined(CPU_FAM_STM32L1)
    status->flash.pagesize = 256;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 4;
    return;
#elif defined(CPU_FAM_STM32L4)
    status->flash.pagesize = 2048;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 8;
    return;
#elif defined(CPU_FAM_STM32F0)
    switch (ST_DEV_ID) {
        case STM32F0_DEV_ID_CAT3:
        case STM32F0_DEV_ID_CAT4:
        case STM32F0_DEV_ID_CAT5:
            status->flash.pagesize = 1024;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        case STM32F0_DEV_ID_CAT7:
        case STM32F0_DEV_ID_CAT9:
            status->flash.pagesize = 2048;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        default:
            status->flash.pagesize = 0;
            status->flash.pages = 0;
            status->flash.alignment = 4;
            break;
    }
    return;
#elif defined(CPU_FAM_STM32F1)
    switch (ST_DEV_ID) {
        case STM32F1_DEV_ID_LD:
        case STM32F1_DEV_ID_MD:
        case STM32F1_DEV_ID_MDVL:
            status->flash.pagesize = 1024;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        case STM32F1_DEV_ID_HD:
        case STM32F1_DEV_ID_HDVL:
        case STM32F1_DEV_ID_XL:
        case STM32F1_DEV_ID_CD:
            status->flash.pagesize = 2048;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        default:
            status->flash.pagesize = 0;
            status->flash.pages = 0;
            status->flash.alignment = 4;
            break;
    }
    return;
#elif defined(CPU_FAM_STM32F2)
    /* actually, flash partition is complicated here */
    status->flash.pagesize = 16384;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 4;
    return;
#elif defined(CPU_FAM_STM32F3)
    status->flash.pagesize = 2048;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 4;
    return;
#elif defined(CPU_FAM_STM32F4)
    /* actually, flash partition is complicated here */
    switch (ST_DEV_ID) {
        case STM32F4_DEV_ID_CAT2:
        case STM32F4_DEV_ID_CAT9:
            status->flash.pagesize = 16384;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        case STM32F4_DEV_ID_CAT1:
        case STM32F4_DEV_ID_CAT3:
        case STM32F4_DEV_ID_CAT4:
        case STM32F4_DEV_ID_CAT5:
        case STM32F4_DEV_ID_CAT6:
        case STM32F4_DEV_ID_CAT7:
        case STM32F4_DEV_ID_CAT8:
        case STM32F4_DEV_ID_CAT10:
            status->flash.pagesize = 16384;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        default:
            status->flash.pagesize = 0;
            status->flash.pages = 0;
            status->flash.alignment = 4;
            break;
    }
    return;
#elif defined(CPU_FAM_STM32F7)
    /* actually, flash partition is complicated here */
    switch (ST_DEV_ID) {
        case STM32F7_DEV_ID_CAT1:
            status->flash.pagesize = 16384;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        case STM32F7_DEV_ID_CAT2:
        case STM32F7_DEV_ID_CAT3:
        case STM32F7_DEV_ID_CAT4:
            status->flash.pagesize = 32768;
            status->flash.pages = status->flash.size / status->flash.pagesize;
            status->flash.alignment = 4;
            break;
        default:
            status->flash.pagesize = 0;
            status->flash.pages = 0;
            status->flash.alignment = 4;
            break;
    }
    return;
#else
#error unexpected MCU
#endif
}

static void get_cpu_ram(cpu_status_t* status) {
    status->ram.size = get_cpu_ram_size();
    status->ram.alignment = 4;
}

static void get_cpu_eeprom(cpu_status_t* status) {
    status->eeprom.size = get_cpu_eeprom_size();
    status->eeprom.alignment = 4;
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
static void get_cpu_voltage(cpu_status_t* status) {
    int vdd = INT16_MIN;
    
    #if defined ADC_VREF_INDEX
    if (adc_init(ADC_VREF_INDEX) == 0) {
        vdd = adc_sample(ADC_VREF_INDEX, ADC_RES_12BIT);
    }
    #endif

    status->voltage.vdd  = vdd;
    status->voltage.vdda = INT16_MIN;
    status->voltage.vbat = INT16_MIN;
}

static void get_cpu_temp(cpu_status_t* status) {
    int temp = INT16_MIN;
    
    #if defined ADC_TEMPERATURE_INDEX
    if (adc_init(ADC_TEMPERATURE_INDEX) == 0) {
        temp = adc_sample(ADC_TEMPERATURE_INDEX, ADC_RES_12BIT);
    }
    #endif

    status->temp.core_temp = temp;
}
#endif

void cpu_init_status(void) {
    get_cpu_ram(&cpu_status);
    get_cpu_flash(&cpu_status);
    get_cpu_eeprom(&cpu_status);

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    cpu_status.voltage.vdd      = INT16_MIN;
    cpu_status.voltage.vdda     = INT16_MIN;
    cpu_status.voltage.vbat     = INT16_MIN;
    cpu_status.temp.core_temp   = INT16_MIN;
    get_cpu_name(cpu_status.model);
#endif
    
    cpu_status.category = get_cpu_category();
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
void cpu_update_status(void) {
    get_cpu_voltage(&cpu_status);
    get_cpu_temp(&cpu_status);
}
#endif

#endif /* MODULE_PERIPH_STATUS */