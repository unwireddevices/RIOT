/*
 * Copyright (C) 2018 Inria
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_eeprom
 * @{
 *
 * @file
 * @brief       Low-level eeprom driver implementation
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */
 
#if defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32L0)

#include <assert.h>
#include <string.h>

#include "cpu.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#include "periph/eeprom.h"

extern void _lock(void);
extern void _unlock(void);
extern void _wait_for_pending_operations(void);

#if defined(CPU_FAM_STM32L0)
    uint32_t eeprom_start_addr = DATA_EEPROM_BASE;
#elif defined(CPU_FAM_STM32L1)
    uint32_t eeprom_start_addr = FLASH_EEPROM_BASE;
#else
    #error "periph/eeprom: *_EEPROM_BASE is not defined"
#endif

static union {
    uint32_t data_word;
    uint8_t  data_bytes[4];
} eeprom_data;

static void _eeprom_write_byte(uint32_t pos, uint8_t byte)
{
    assert(pos < cpu_status.eeprom.size);
    
    _wait_for_pending_operations();
    *(__IO uint8_t *)(eeprom_start_addr + pos) = byte;
}

static void _eeprom_write_word(uint32_t pos, uint32_t word)
{
    assert(pos <= cpu_status.eeprom.size - sizeof(uint32_t));
    
    _wait_for_pending_operations();
    *(__IO uint32_t *)(eeprom_start_addr + pos) = word;
}

static uint8_t _eeprom_read_byte(uint32_t pos)
{
    assert(pos < cpu_status.eeprom.size);
    
    _wait_for_pending_operations();
    return *(__IO uint8_t *)(eeprom_start_addr + pos);
}

static uint32_t _eeprom_read_word(uint32_t pos)
{
    assert(pos <= cpu_status.eeprom.size - sizeof(uint32_t));
    
    _wait_for_pending_operations();
    return *(__IO uint32_t *)(eeprom_start_addr + pos);
}

void eeprom_write_byte(uint32_t pos, uint8_t data)
{
    assert(pos < cpu_status.eeprom.size);

    DEBUG("Writing data '%c' to EEPROM at pos %" PRIu32 "\n", data, pos);
    _unlock();
    _eeprom_write_byte(pos, data);
    _lock();
}

size_t eeprom_write(uint32_t pos, uint8_t *data, size_t len)
{
    assert((pos + len) <= cpu_status.eeprom.size);

    DEBUG("[EEPROM] write %d bytes\n", len);
    _unlock();
    uint32_t i;
    uint32_t bytes = 0;
    
    /* write first bytes in case of unaligned access */
    uint32_t shift = pos & 0x3;
    if (shift > 0) {
        /* read-modify-write */
        eeprom_data.data_word = _eeprom_read_word(pos - shift);
        bytes = 4 - shift;
        if (len < bytes) {
            bytes = len;
        }
        if (data) {
            memcpy(&eeprom_data.data_bytes[shift], data, bytes);
        } else {
            /* if data is NULL just clear EEPROM */
            memset(&eeprom_data.data_bytes[shift], 0, bytes);
        }
        _eeprom_write_word(pos - shift, eeprom_data.data_word);
    }
    
    /* write by words to speed up the process */
    uint32_t words = (len - bytes) / 4;
    uint32_t remnant = (len - bytes) % 4;
    for (i = 0; i < words; i++) {
        if (data) {
            /* memcpy to fix possible unaligned access to data array */
            memcpy((void *)&eeprom_data.data_word, (void *)&data[bytes], 4);
            _eeprom_write_word(pos + bytes, eeprom_data.data_word);
        } else {
            /* if data is NULL just clear EEPROM */
            _eeprom_write_word(pos + bytes, 0);
        }
        bytes += 4;
    }
    
    /* write last bytes in case of unaligned access */
    if (remnant) {
        /* read-modify-write */
        eeprom_data.data_word = _eeprom_read_word(pos + bytes);
        if (data) {
            memcpy(eeprom_data.data_bytes, &data[bytes], remnant);
        } else {
            memset(eeprom_data.data_bytes, 0, remnant);
        }
        _eeprom_write_word(pos + bytes, eeprom_data.data_word);
        bytes += remnant;
    }
    _lock();
    
    return bytes;
}

uint8_t eeprom_read_byte(uint32_t pos)
{
    assert(pos < cpu_status.eeprom.size);

    DEBUG("Reading data from EEPROM at pos %" PRIu32 "\n", pos);
    _unlock();
    uint8_t byte = _eeprom_read_byte(pos);
    _lock();
    
    return byte;
}

size_t eeprom_read(uint32_t pos, uint8_t *data, size_t len)
{
    assert((pos + len) <= cpu_status.eeprom.size);
    
    DEBUG("[EEPROM] read %d bytes\n", len);
    _unlock();
    uint32_t i;
    uint32_t bytes = 0;
    
    /* read first bytes in case of unaligned access */
    uint32_t shift = pos & 0x3; 
    if (shift > 0) {
        /* read a word and discard unneded bytes */
        eeprom_data.data_word = _eeprom_read_word(pos - shift);
        bytes = (4 - shift);
        if (len < bytes) {
            bytes = len;
        }
        memcpy(data, &eeprom_data.data_bytes[shift], bytes);
    }
    
    /* read by words to speed up the process */
    uint32_t words = (len - bytes) / 4;
    uint32_t remnant = (len - bytes) % 4;
    for (i = 0; i < words; i++) {
        eeprom_data.data_word = _eeprom_read_word(pos + bytes);
        /* memcpy to fix possible unaligned access to data array */
        memcpy((void *)&data[bytes], (void *)&eeprom_data.data_word, 4);
        bytes += 4;
    }
    
    /* read last bytes in case of unaligned access */
    if (remnant) {
        /* read a word and discard unneded bytes */
        eeprom_data.data_word = _eeprom_read_word(pos + bytes);
        memcpy((void *)&data[bytes], (void *)&eeprom_data.data_word, remnant);
        bytes += remnant;
    }
    _lock();
    
    return bytes;
}

size_t eeprom_erase(void)
{
    return eeprom_write(0, NULL, cpu_status.eeprom.size);
}

size_t eeprom_clear(uint32_t pos, size_t len)
{
    return eeprom_write(pos, NULL, len);
}
#endif /* defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32L0) */
