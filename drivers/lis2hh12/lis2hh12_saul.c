/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lis2hh12
 * @{
 *
 * @file
 * @brief       LIS2HH12 accelerometer SAUL mapping
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include "saul.h"
#include "lis2dh12.h"

static int read_accelerometer(const void *dev, phydat_t *res)
{
    if (lis2hh12_read((const lis2hh12_t *)dev, res->val) != LIS2HH12_OK) {
        return 0;
    }
    res->unit = UNIT_G;
    res->scale = -3;
    return 3;
}

const saul_driver_t lis2hh12_saul_driver = {
    .read = read_accelerometer,
    .write = saul_notsup,
    .type = SAUL_SENSE_ACCEL
};
