/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_periph_flashpage
 * @{
 *
 * @file
 * @brief       Common flash page functions
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include "cpu.h"
#include "assert.h"

/* guard this file, must be done before including periph/flashpage.h
 * TODO: remove as soon as periph drivers can be build selectively */
#if defined(FLASHPAGE_SIZE)

#include "periph/flashpage.h"

void flashpage_read(uint32_t page, void *data, uint32_t size)
{
    assert(page < (cpu_status.flash.size / FLASHPAGE_SIZE));
    assert(size < FLASHPAGE_SIZE);

    memcpy(data, flashpage_addr(page), size);
}

int flashpage_verify(uint32_t page, const void *data, uint32_t size)
{
    assert(page < (cpu_status.flash.size / FLASHPAGE_SIZE));

    if (memcmp(flashpage_addr(page), data, size) == 0) {
        return FLASHPAGE_OK;
    }
    else {
        return FLASHPAGE_NOMATCH;
    }
}

int flashpage_write_and_verify(uint32_t page, const void *data, uint32_t size)
{
    flashpage_write(page, data, size);
    return flashpage_verify(page, data, size);
}

#endif
