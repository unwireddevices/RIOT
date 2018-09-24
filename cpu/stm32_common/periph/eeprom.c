/*
 * Copyright (C) 2018 Inria
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
 *
 * @}
 */

#include <assert.h>
#include <string.h>

#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)

#include "cpu.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#include "periph/eeprom.h"

extern void _lock(void);
extern void _unlock(void);
extern void _wait_for_pending_operations(void);

#ifndef EEPROM_START_ADDR
#error "periph/eeprom: EEPROM_START_ADDR is not defined"
#endif

static void _eeprom_write_byte(uint32_t pos, uint8_t byte)
{
    _wait_for_pending_operations();
    *(__IO uint8_t *)(EEPROM_START_ADDR + pos) = byte;
}

static void _eeprom_write_word(uint32_t pos, uint32_t word)
{
    _wait_for_pending_operations();
    *(__IO uint32_t *)(EEPROM_START_ADDR + pos) = word;
}

uint8_t eeprom_read_byte(uint32_t pos)
{
    _wait_for_pending_operations();
    return *(uint8_t *)(EEPROM_START_ADDR + pos);
}

static uint32_t _eeprom_read_word(uint32_t pos)
{
    _wait_for_pending_operations();
    return *(uint32_t *)(EEPROM_START_ADDR + pos);
}

void eeprom_write_byte(uint32_t pos, uint8_t data)
{
    if (!(pos < get_cpu_eeprom_size())) {
        DEBUG("[EERPOM] overflow");
        assert(false);
    }

    DEBUG("Writing data '%c' to EEPROM at pos %lu\n", data, pos);
    _unlock();
    _eeprom_write_byte(pos, data);
    _lock();
}

size_t eeprom_write(uint32_t pos, const uint8_t *data, size_t len)
{
    if (!(pos + len < get_cpu_eeprom_size())) {
        DEBUG("[EERPOM] overflow");
        assert(false);
    }

    DEBUG("[EEPROM] write %d bytes\n", len);
    _unlock();
    uint32_t i;
    uint32_t bytes = 0;
    /* write first bytes in case of unaligned access */
    uint32_t shift = pos & 0x3;
    if (shift > 0) {
        for (i = shift; i < 4; i++) {
            if (data)
                _eeprom_write_byte(pos + bytes, data[bytes]);
            else
                _eeprom_write_byte(pos + bytes, 0);
            bytes++;
        }
    }
    
    /* write by words to speed up the process */
    uint32_t words = (len - bytes)/4;
    uint32_t remnant = (len - bytes)%4;
    uint32_t data_word;
    for (i = 0; i < words; i++) {
        /* memcpy to fix possible unaligned access to data array */
        if (data) {
            memcpy((void *)&data_word, (void *)&data[bytes], 4);
            _eeprom_write_word(pos + bytes, data_word);
        } else {
            _eeprom_write_word(pos + bytes, 0);
        }
        bytes += 4;
    }
    
    /* write last bytes in case of unaligned access */
    if (remnant) {
        for (i = 0; i < remnant; i++) {
            if (data) {
                _eeprom_write_byte(pos + bytes, data[bytes]);
            } else {
                _eeprom_write_byte(pos + bytes, 0);
            }
            bytes++;
        }
    }
    _lock();
    
    return bytes;
}

size_t eeprom_read(uint32_t pos, uint8_t *data, size_t len)
{
    if (!(pos + len < get_cpu_eeprom_size())) {
        DEBUG("[EERPOM] overflow");
        assert(false);
    }
    
    DEBUG("[EEPROM] read %d bytes\n", len);
    _unlock();
    uint32_t i;
    uint32_t bytes = 0;
    /* read first bytes in case of unaligned access */
    uint32_t shift = pos & 0x3;
    if (shift > 0) {
        for (i = shift; i < 4; i++) {
            data[bytes] = eeprom_read_byte(pos + bytes);
            bytes++;
        }
    }
    
    /* read by words to speed up the process */
    uint32_t words = (len - bytes)/4;
    uint32_t remnant = (len - bytes)%4;
    uint32_t data_word;
    for (i = 0; i < words; i++) {
        data_word = _eeprom_read_word(pos + bytes);
        /* memcpy to fix possible unaligned access to data array */
        memcpy((void *)&data[bytes], (void *)&data_word, 4);
        bytes += 4;
    }
    
    /* read last bytes in case of unaligned access */
    if (remnant) {
        for (i = 0; i < remnant; i++) {
            data[bytes] = eeprom_read_byte(pos + bytes);
            bytes++;
        }
    }
    _lock();
    
    return bytes;
}

size_t eeprom_clear_all(void)
{
    size_t len = get_cpu_eeprom_size();
    _unlock();
    uint32_t i;
    /* we assume EEPROM addresses are aligned by default */
    for (i = 0; i < len; i += 4) {
        /* Erase word */
        _eeprom_write_word(i, 0);
    }
    _lock();
    
    return i + 1;
}

size_t eeprom_clear(uint32_t pos, size_t len)
{
    return eeprom_write(pos, NULL, len);
}

#endif