/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sht3x
 * @brief       SAUL adaption for Sensirion SHT30/SHT31/SHT35 devices
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "phydat.h"
#include "saul.h"
#include "sht3x.h"

static int read(const sht3x_dev_t *dev, int16_t *temp, int16_t *hum)
{
    return sht3x_read((sht3x_dev_t *)dev, temp, hum);
}

static int read_temp(const void *dev, phydat_t *res)
{
   if (read(dev, &res->val[0], NULL) == SHT3X_OK) {
        res->unit = UNIT_TEMP_C;
        res->scale = -2;
        return 1;
    }

    return -ECANCELED;
}

static int read_hum(const void *dev, phydat_t *res)
{
    if (read(dev, NULL, &res->val[0]) == SHT3X_OK) {
        res->unit = UNIT_PERCENT;
        res->scale = -2;
        return 1;
    }

    return -ECANCELED;
}

const saul_driver_t sht3x_saul_driver_temperature = {
    .read = read_temp,
    .write = saul_notsup,
    .type = SAUL_SENSE_TEMP
};

const saul_driver_t sht3x_saul_driver_humidity = {
    .read = read_hum,
    .write = saul_notsup,
    .type = SAUL_SENSE_HUM
};
