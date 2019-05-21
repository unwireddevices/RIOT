/*
 * Copyright (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf52_common
 * @ingroup     drivers_periph_eeprom
 * @{
 *
 * @file
 * @brief       EEPROM emulation driver implementation
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include <assert.h>
#include <string.h>

#include "cpu.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

#include "periph/eeprom.h"
#include "periph/flashpage.h"

#include "periph_conf.h"

static uint32_t eeprom_start_addr = (FLASHPAGE_SIZE * FLASHPAGE_NUMOF) - EEPROM_SIZE;
static uint8_t  eeprom_buf[FLASHPAGE_SIZE];

void eeprom_write_byte(uint32_t pos, uint8_t data)
{
    assert(pos < EEPROM_SIZE);

    uint32_t page = flashpage_page((uint32_t *)(eeprom_start_addr + pos));
    uint8_t *page_start = flashpage_addr(page);
    uint32_t pos_relative = pos % FLASHPAGE_SIZE;
    
    DEBUG("[F-EEPROM] Writing data '%c' to page %lu offset %lu\n", data, page, pos);
    
    /* read-modify-write */
    memcpy(eeprom_buf, page_start, FLASHPAGE_SIZE);
    eeprom_buf[pos_relative] = data;
    flashpage_write(page, eeprom_buf, FLASHPAGE_SIZE);
    
    DEBUG("[F-EEPROM] Data successfully written\n");
}

size_t eeprom_write(uint32_t pos, uint8_t *data, size_t len)
{
    assert((pos + len) <= EEPROM_SIZE);

    uint32_t num_pages = (pos + len) / FLASHPAGE_SIZE;
    if (((pos + len) % FLASHPAGE_SIZE) != 0) {
        num_pages += 1;
    }
    
    for (uint32_t i = 0; i < num_pages; i++) {
        uint32_t page = flashpage_page((uint32_t *)(eeprom_start_addr + pos)) + i;
        uint8_t *page_start = flashpage_addr(page);
        
        uint32_t start, size;
        
        if (i == 0) {
            /* first page */
            start = pos % FLASHPAGE_SIZE;
            
            if (num_pages > 1) {
                /* first but not last */
                size = FLASHPAGE_SIZE - pos;
            } else {
                /* first and last */
                size = len;
            }
        } else {
            start = 0;

            /* last page */
            if (i == num_pages - 1) {
                size = len - FLASHPAGE_SIZE * (num_pages - 2) - (FLASHPAGE_SIZE - pos);
            } else {
                size = FLASHPAGE_SIZE;
            }
        }
        
        DEBUG("[F-EEPROM] Writing %lu bytes offset %lu to page %lu (%lu/%lu) offset %lu\n",
               size, FLASHPAGE_SIZE * i, page, i+1, num_pages, start);
        
        /* read-modify-write */
        memcpy(eeprom_buf, page_start, FLASHPAGE_SIZE);
        memcpy(&eeprom_buf[start], &data[FLASHPAGE_SIZE * i], size);
        flashpage_write(page, eeprom_buf, FLASHPAGE_SIZE);
        
        DEBUG("[F-EEPROM] Data successfully written\n");
    }

    return len;
}

uint8_t eeprom_read_byte(uint32_t pos)
{
    assert(pos < EEPROM_SIZE);

    DEBUG("[F-EEPROM] Reading data from pos %" PRIu32 "\n", pos);
    
    uint32_t page = flashpage_page((uint32_t *)(eeprom_start_addr + pos));
    uint32_t start = pos % FLASHPAGE_SIZE;
    
    flashpage_read(page, eeprom_buf, FLASHPAGE_SIZE);
    
    return eeprom_buf[start];
}

size_t eeprom_read(uint32_t pos, uint8_t *data, size_t len)
{
    uint32_t num_pages = (pos + len) / FLASHPAGE_SIZE;
    if (((pos + len) % FLASHPAGE_SIZE) != 0) {
        num_pages += 1;
    }
    
    for (uint32_t i = 0; i < num_pages; i++) {
        uint32_t page = flashpage_page((uint32_t *)(eeprom_start_addr + pos)) + i;
        
        uint32_t start, size;
        
        if (i == 0) {
            /* first page */
            start = pos % FLASHPAGE_SIZE;
            
            if (num_pages > 1) {
                /* first but not last */
                size = FLASHPAGE_SIZE - pos;
            } else {
                /* first and last */
                size = len;
            }
        } else {
            start = 0;

            /* last page */
            if (i == num_pages - 1) {
                size = len - FLASHPAGE_SIZE * (num_pages - 2) - (FLASHPAGE_SIZE - pos);
            } else {
                size = FLASHPAGE_SIZE;
            }
        }
        
        DEBUG("[F-EEPROM] Reading %lu bytes offset %lu from page %lu (%lu/%lu) offset %lu\n",
               size, FLASHPAGE_SIZE * i, page, i + 1, num_pages, start);
        
        flashpage_read(page, eeprom_buf, FLASHPAGE_SIZE);
        memcpy(&data[FLASHPAGE_SIZE * i], &eeprom_buf[start], size);
        
        DEBUG("[F-EEPROM] Data successfully read\n");
    }
    return len;
}

size_t eeprom_erase(void)
{
    return eeprom_write(0, NULL, EEPROM_SIZE);
}

size_t eeprom_set(uint32_t pos, uint8_t val, size_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        eeprom_write_byte(pos + i, val);
    }
    
    return len;
}

size_t eeprom_clear(uint32_t pos, size_t len)
{
    if (len > EEPROM_SIZE - pos) {
        len = EEPROM_SIZE - pos;
    }
    
    return eeprom_write(pos, NULL, len);
}
