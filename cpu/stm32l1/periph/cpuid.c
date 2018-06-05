/*
 * Copyright (C) 2017 Eistec AB
 * Copyright (C) 2014-2016 Freie Universität Berlin
 * Copyright (C) 2015 James Hollister
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  drivers
 * @{
 *
 * @file
 * @brief       Generic implementation of the CPUID driver interface
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      James Hollister <jhollisterjr@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "periph/cpuid.h"
#include "cpu_conf.h"

void cpuid_get(void *id)
{
    uint32_t cpuid_address;
    
    if ((ST_DEV_ID == STM32L1_DEV_ID_CAT1) || (ST_DEV_ID == STM32L1_DEV_ID_CAT2)) {
        cpuid_address = STM32L1_CPUID_ADDR_CAT12;
    } else {
        cpuid_address = STM32L1_CPUID_ADDR_CAT3456;
    }
    
    memcpy(id, (void *)cpuid_address, CPUID_LEN);
}
