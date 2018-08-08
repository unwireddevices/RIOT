/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of LIS2HH12 accelerometers
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#ifdef MODULE_LIS2HH12

#include "log.h"
#include "assert.h"
#include "saul_reg.h"
#include "lis2hh12.h"
#include "lis2hh12_params.h"

/**
 * @brief   Number of configured sensors
 */
#define LIS2DH12_NUM    (sizeof(lis2hh12_params) / sizeof(lis2hh12_params[0]))


/**
 * @brief   Number of defined SAUL registry info entries
 */
#define LIS2HH12_SAULINFO_NUM   (sizeof(lis2hh12_saul_info) / \
                                 sizeof(lis2hh12_saul_info[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static lis2hh12_t lis2hh12_devs[LIS2HH12_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[LIS2HH12_NUM];

void auto_init_lis2hh12(void)
{
    assert(LIS2HH12_NUM == LIS2HH12_SAULINFO_NUM);

    for (unsigned int i = 0; i < LIS2HH12_NUM; i++) {
        int res;

        LOG_DEBUG("[auto_init_saul] initializing lis2hh12 #%u\n", i);

        res = lis2dh12_init(&lis2hh12_devs[i], &lis2hh12_params[i]);
        if (res < 0) {
            LOG_ERROR("[auto_init_saul] error initializing lis2hh12 #%u\n", i);
            continue;
        }

        saul_entries[i].dev = &(lis2hh12_devs[i]);
        saul_entries[i].name = lis2hh12_saul_info[i].name;
        saul_entries[i].driver = &lis2hh12_saul_driver;
        saul_reg_add(&(saul_entries[i]));
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_LIS2HH12 */