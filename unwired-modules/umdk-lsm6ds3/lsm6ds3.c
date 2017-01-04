/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file		lsm6ds3.h
 * @brief       ST's LSM6DS3 6-axis motion sensor driver implementation
 * @author      EP <ep@unwds.com>
 */

#include "../umdk-lsm6ds3/include/lsm6ds3.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../umdk-lsm6ds3/include/lsm6ds3_regs.h"
#include "assert.h"
#include "periph/i2c.h"


#ifdef __cplusplus
extern "C" {
#endif

static bool read_register(lsm6ds3_t *dev, uint8_t *result, uint8_t reg)
{
    i2c_acquire(dev->params.i2c);

    if (i2c_read_reg(dev->params.i2c, dev->params.i2c_addr, reg, (char *) result) < 0) {
        i2c_release(dev->params.i2c);
        return false;
    }

    i2c_release(dev->params.i2c);

    return true;
}

bool read_register_int16(lsm6ds3_t *dev, int16_t *result, uint8_t reg)
{
    i2c_acquire(dev->params.i2c);

    uint8_t buf[2];
    if (i2c_read_regs(dev->params.i2c, dev->params.i2c_addr, reg, (char *) buf, 2) < 0) {
        i2c_release(dev->params.i2c);
        return false;
    }

    /* Swap bytes */
    int16_t out = (int16_t) buf[0] | ((int16_t) buf[1] << 8);

    /* Return output */
    *result = out;

    i2c_release(dev->params.i2c);

    return true;
}

bool write_register(lsm6ds3_t *dev, uint8_t reg, uint8_t data)
{
    i2c_acquire(dev->params.i2c);

    if (i2c_write_reg(dev->params.i2c, dev->params.i2c_addr, reg, data) < 0) {
        i2c_release(dev->params.i2c);
        return false;
    }

    i2c_release(dev->params.i2c);

    return true;
}

bool switch_to_emb_page(lsm6ds3_t *dev)
{
    return write_register(dev, LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80);
}

bool switch_to_base_page(lsm6ds3_t *dev)
{
    return write_register(dev, LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);
}

int lsm6ds3_init(lsm6ds3_t *dev, lsm6ds3_param_t *param)
{
    assert(dev != NULL);
    assert(param != NULL);

    /* Copy settings */
    dev->params = *param;
	
	/* Initialize I2C bus */
	if (i2c_init_master(dev->params.i2c, I2C_SPEED_NORMAL) < 0) {
		i2c_release(dev->params.i2c);
		
		return -1;
	}

    /* Check device ID */
    uint8_t id;
    read_register(dev, &id, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

    if (id != LSM6DS3_DEFAULT_ID) {
        return -3;
    }

    return 0;
}

bool lsm6ds3_configure(lsm6ds3_t *dev, lsm6ds3_param_t *settings)
{
    assert(dev != NULL);
    assert(settings != NULL);

    uint8_t data = 0;

    /* Setup the accelerometer */
    data = 0;
    if (settings->accel_enabled) {
        switch (settings->accel_bandwidth) {
            case 50:
                data |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
                break;
            case 100:
                data |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
                break;
            case 200:
                data |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
                break;
            default:
            case 400:
                data |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
                break;
        }

        switch (settings->accel_range) {
            case 2:
                data |= LSM6DS3_ACC_GYRO_FS_XL_2g;
                break;
            case 4:
                data |= LSM6DS3_ACC_GYRO_FS_XL_4g;
                break;
            case 8:
                data |= LSM6DS3_ACC_GYRO_FS_XL_8g;
                break;
            default:
            case 16:
                data |= LSM6DS3_ACC_GYRO_FS_XL_16g;
                break;
        }

        switch (settings->accel_sample_rate) {
            case 13:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
                break;
            case 26:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
                break;
            case 52:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
                break;
            default:
            case 104:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
                break;
            case 208:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
                break;
            case 416:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
                break;
            case 833:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
                break;
            case 1660:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
                break;
            case 3330:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
                break;
            case 6660:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
                break;
            case 13330:
                data |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
                break;
        }
    }

    /* Write composed register value */
    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL1_XL, data)) {
        return false;
    }

    /* Set ODR bit */
    if (!read_register(dev, &data, LSM6DS3_ACC_GYRO_CTRL4_C)) {
        return false;
    }

    data &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
    if (settings->accel_odr_off == 1) {
        data |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
    }

    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL4_C, data)) {
        return false;
    }

    /* Setup the gyro */
    data = 0;
    if (settings->gyro_enabled) {
        switch (settings->gyro_range) {
            case 125:
                data |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
                break;
            case 245:
                data |= LSM6DS3_ACC_GYRO_FS_G_245dps;
                break;
            case 500:
                data |= LSM6DS3_ACC_GYRO_FS_G_500dps;
                break;
            case 1000:
                data |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
                break;
            default:
            case 2000:
                data |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
                break;
        }

        switch (settings->gyro_sample_rate) {
            case 13:
                data |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
                break;
            case 26:
                data |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
                break;
            case 52:
                data |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
                break;
            default:
            case 104:
                data |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
                break;
            case 208:
                data |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
                break;
            case 416:
                data |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
                break;
            case 833:
                data |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
                break;
            case 1660:
                data |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
                break;
        }
    }

    /* Write down the settings */
    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL2_G, data)) {
        return false;
    }

    return true;
}

static inline float_t convert_acc(lsm6ds3_t *dev, int16_t data)
{
    float_t output = (float_t) data * 0.061 * (dev->params.accel_range >> 1) / 1000;

    return output;
}

static bool read_raw_accel_x(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTX_L_XL);

    return res;
}

static bool read_raw_accel_y(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTY_L_XL);

    return res;
}

static bool read_raw_accel_z(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTZ_L_XL);

    return res;
}

static inline float_t convert_gyr(lsm6ds3_t *dev, int16_t data)
{
    uint8_t gyro_range_factor = dev->params.gyro_range / 125;

    if (dev->params.gyro_range == 245) {
        gyro_range_factor = 2;
    }

    float_t output = (float_t) data * 4.375 * (gyro_range_factor) / 1000;
    return output;
}

static bool read_raw_gyro_x(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTX_L_G);

    return res;
}

static bool read_raw_gyro_y(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTY_L_G);

    return res;
}

static bool read_raw_gyro_z(lsm6ds3_t *dev, int16_t *data)
{
    bool res = read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTZ_L_G);

    return res;
}

bool lsm6ds3_get_raw(lsm6ds3_t *dev, lsm6ds3_data_t *data)
{
    int16_t acc_x, acc_y, acc_z;

    if (!read_raw_accel_x(dev, &acc_x)) {
        return false;
    }

    if (!read_raw_accel_y(dev, &acc_y)) {
        return false;
    }

    if (!read_raw_accel_z(dev, &acc_z)) {
        return false;
    }


    data->acc_x = convert_acc(dev, acc_x);
    data->acc_y = convert_acc(dev, acc_y);
    data->acc_z = convert_acc(dev, acc_z);

    int16_t gyr_x, gyr_y, gyr_z;

    if (!read_raw_gyro_x(dev, &gyr_x)) {
        return false;
    }

    if (!read_raw_gyro_y(dev, &gyr_y)) {
        return false;
    }

    if (!read_raw_gyro_z(dev, &gyr_z)) {
        return false;
    }


    data->gyr_x = convert_gyr(dev, gyr_x);
    data->gyr_y = convert_gyr(dev, gyr_y);
    data->gyr_z = convert_gyr(dev, gyr_z);

    return true;
}

uint16_t lsm6ds3_read_temp_c(lsm6ds3_t *dev)
{
	int16_t out = 0;
	read_register_int16(dev, &out, LSM6DS3_ACC_GYRO_OUT_TEMP_L);
	out += (125*16);
	return (uint16_t)(out & 0x7FFF);
}

#ifdef __cplusplus
}
#endif
