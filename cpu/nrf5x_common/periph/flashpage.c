/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf5x_common
 * @ingroup     drivers_periph_flashpage
 * @{
 *
 * @file
 * @brief       Low-level flash page driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "assert.h"
#include "periph/flashpage.h"

void flashpage_write(uint32_t page, const void *data, uint32_t data_size)
{
    assert(page < FLASHPAGE_NUMOF);

    uint32_t *page_addr = (uint32_t *)flashpage_addr(page);
    const uint32_t *data_addr = data;

    /* erase given page */
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    NRF_NVMC->ERASEPAGE = (uint32_t)page_addr;
    while (NRF_NVMC->READY == 0) {}
    
    bool erase_is_enough = true;
    for (unsigned i = 0; i < (data_size/4); i++) {
        if (data_addr[i] != 0xFFFFFFFF) {
            erase_is_enough = false;
        }
    }

    /* write data to page */
    if ((data != NULL) && !erase_is_enough) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        for (unsigned i = 0; i < (data_size / 4); i++) {
            *page_addr++ = data_addr[i];
        }
    }

    /* finish up */
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
}

void flashpage_write_raw(void *target_addr, const void *data, size_t len) {
    uint32_t *page_addr = target_addr;
    const uint32_t *data_addr = data;
    
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while (NRF_NVMC->READY == 0) {}
    
    /* write data to page */
    if (data != NULL) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        for (unsigned i = 0; i < (len / 4); i++) {
            *page_addr++ = data_addr[i];
        }
    }
    
    /* finish up */
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
}
