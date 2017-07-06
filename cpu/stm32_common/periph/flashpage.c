/*
 * Copyright (C) 2017 Unwired Devices LLC
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
 * @brief       Low-level Flash driver implementation
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */
 
#include <inttypes.h>

#include "cpu.h"
#include "periph/flashpage.h"

static void flash_unlock(void) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

static void flash_lock() {
    FLASH->CR |= FLASH_CR_LOCK;
}

static uint8_t flash_ready(void) {
    return !(FLASH->SR & FLASH_SR_BSY);
}

static void flash_erase_page(uint32_t address) {
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR|= FLASH_CR_STRT;
    while(!flash_ready())
    FLASH->CR&= ~FLASH_CR_PER;
}

void flashpage_write(int page, void *data) {
    FLASH->CR |= FLASH_CR_PG;
    while(!flash_ready()) {}
    
    uint32_t address = (uint32_t)flashpage_addr(page);
    
    uint16_t *p_data = (uint16_t *)data;
    
    int i = 0;
    for (i = 0; i < FLASHPAGE_SIZE/2; i++) {
        *(__IO uint16_t *)address = p_data[i];
        
        while(!flash_ready()) {}
        address += 2;
    }
    
    FLASH->CR &= ~(FLASH_CR_PG);
}

void flashpage_read(int page, void *data) {
    uint32_t address = (uint32_t)flashpage_addr(page);

    uint16_t *p_data = (uint32_t *)data;
    
    int i = 0;
    for (i = 0; i < FLASHPAGE_SIZE/4; i++) {
        p_data[i] = *(__IO uint32_t *)address;
    }
}

int flashpage_verify(int page, void *data) {
    
}

int flashpage_write_and_verify(int page, void *data) {
    flashpage_write(page, data);
    return (flashpage_verify(page, data));
}