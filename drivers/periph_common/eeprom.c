/*
 * Copyright (C) 2018 Inria
 *           (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers
 * @{
 *
 * @file
 * @brief       Common eeprom functions implementation
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include <string.h>

#include "periph_cpu.h"
#include "assert.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* guard this file, must be done before including periph/eeprom.h */
#if defined(EEPROM_SIZE)

#include "periph/eeprom.h"

uint8_t eeprom_read_byte(uint32_t pos)
{
    uint8_t byte;
    eeprom_read(pos, &byte, 1);
    return byte;
}

void eeprom_write_byte(uint32_t pos, uint8_t byte)
{
    eeprom_write(pos, &byte, 1);
}

size_t eeprom_set(uint32_t pos, uint8_t val, size_t len)
{
    assert(pos + len <= EEPROM_SIZE);

    for (size_t i = 0; i < len; i++) {
        eeprom_write_byte(pos++, val);
    }

    return len;
}

size_t eeprom_clear(uint32_t pos, size_t len)
{
    return eeprom_set(pos, EEPROM_CLEAR_BYTE, len);
}

size_t eeprom_erase(void)
{
    return eeprom_clear(0, EEPROM_SIZE);
}

#else

/* EEPROM emulation using flash */

#include "periph/flashpage.h"
#include "periph_conf.h"

/* EEPROM_SIZE should be defined in periph_conf.h of the specific board */
#if defined(EEPROM_SIZE)

#include "periph/eeprom.h"

static uint32_t eeprom_start_addr = FLASHPAGE_SIZE*FLASHPAGE_NUMOF - EEPROM_SIZE;
static uint8_t ram_buffer[FLASHPAGE_SIZE];

typedef enum {
    FLASH_EEPROM_WRITE,
    FLASH_EEPROM_READ,
    FLASH_EEPROM_SET
} _eeprom_op_type;

static size_t _eeprom_operation(uint32_t pos, uint8_t *data, size_t len, _eeprom_op_type type) {
    assert((pos + len) <= EEPROM_SIZE);
    
    uint32_t num_pages = (pos + len)/FLASHPAGE_SIZE;
    if ((pos + len) % FLASHPAGE_SIZE) {
        num_pages += 1;
    }
    
    uint32_t page = flashpage_page((uint32_t *)(eeprom_start_addr + pos));
    uint32_t start, size, transferred = 0;
    
    for (uint32_t i = 0; i < num_pages; i++) {
        if (i == 0) {
            /* first page */
            start = pos;
            if (num_pages == 1) {
                /* and last at the same time */
                size = len;
            } else {
                size = FLASHPAGE_SIZE - pos;
            }
        } else {
            start = 0;
            if (i == (num_pages - 1)) {
                /* last page */
                size = len - transferred;
            } else {
                size = FLASHPAGE_SIZE;
            }
        }
        
        DEBUG("[EEPROM] Page %lu/%lu (flash page %lu) %lu bytes starting at %lu" \
              " (%lu already done)\n", i + 1, num_pages, page + i, size, start, transferred);
        
        switch (type) {
            case FLASH_EEPROM_WRITE:
                /* read-modify-write */
                flashpage_read(page + i, ram_buffer, FLASHPAGE_SIZE);
                memcpy(&ram_buffer[start], &data[transferred], size);
                flashpage_write(page + i, ram_buffer, FLASHPAGE_SIZE);
                break;
                
            case FLASH_EEPROM_READ:
                flashpage_read(page + i, ram_buffer, FLASHPAGE_SIZE);
                memcpy(&data[transferred], &ram_buffer[start], size);
                break;

            case FLASH_EEPROM_SET:
                /* read-modify-write */
                flashpage_read(page + i, ram_buffer, FLASHPAGE_SIZE);
                memset(&ram_buffer[start], *data, size);
                flashpage_write(page + i, ram_buffer, FLASHPAGE_SIZE);
                break;

            default:
                return 0;
        }

        transferred += size;
    }
    
    return transferred;
}

size_t eeprom_write(uint32_t pos, uint8_t *data, size_t len)
{    
    return _eeprom_operation(pos, data, len, FLASH_EEPROM_WRITE);
}

void eeprom_write_byte(uint32_t pos, uint8_t data)
{
    _eeprom_operation(pos, &data, 1, FLASH_EEPROM_WRITE);
}

size_t eeprom_read(uint32_t pos, uint8_t *data, size_t len)
{
    return _eeprom_operation(pos, data, len, FLASH_EEPROM_READ);
}

uint8_t eeprom_read_byte(uint32_t pos)
{
    uint8_t byte;
    
    _eeprom_operation(pos, &byte, 1, FLASH_EEPROM_READ);
    
    return byte;
}

size_t eeprom_erase(void)
{
    uint8_t val = 0;
    return _eeprom_operation(0, &val, EEPROM_SIZE, FLASH_EEPROM_SET);
}

size_t eeprom_clear(uint32_t pos, size_t len)
{
    uint8_t val = 0;
    return _eeprom_operation(pos, &val, len, FLASH_EEPROM_SET);
}

size_t eeprom_set(uint32_t pos, uint8_t val, size_t len)
{
    return _eeprom_operation(pos, &val, len, FLASH_EEPROM_SET);
}
#endif
#endif


