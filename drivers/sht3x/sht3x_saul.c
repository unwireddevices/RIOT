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

/**
 * Humidity and temperature sensor values are fetched by separate saul
 * functions. To avoid double waiting for the sensor and readout of the
 * sensor values, we read both sensor values once, if necessary, and
 * store them in local variables to provide them in the separate saul
 * read functions.
 */
static bool _temp_valid = false;
static bool _hum_valid = false;
static int16_t _temp;
static int16_t _hum;

static int read(const sht3x_dev_t *dev)
{
    /* read both sensor values */
    int res = sht3x_read((sht3x_dev_t *)dev, &_temp, &_hum);
    if (res != SHT3X_OK) {
        return res;
    }
    /* mark both sensor values as valid */
    _temp_valid = true;
    _hum_valid = true;
    return SHT3X_OK;
}

static int read_temp(const void *dev, phydat_t *data)
{
    /* either local variable is valid or fetching it was successful */
    if (_temp_valid || read(dev) == SHT3X_OK) {
        /* mark local variable as invalid */
        _temp_valid = false;

        data->val[0] = _temp;
        data->unit = UNIT_TEMP_C;
        data->scale = -2;
        return 1;
    }
    return -ECANCELED;
}

static int read_hum(const void *dev, phydat_t *data)
{
    /* either local variable is valid or fetching it was successful */
    if (_hum_valid || read(dev) == SHT3X_OK) {
        /* mark local variable as invalid */
        _hum_valid = false;

        data->val[0] = _hum;
        data->unit = UNIT_PERCENT;
        data->scale = -2;
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
