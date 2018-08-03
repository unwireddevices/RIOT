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
 * @file        lsm6ds3.h
 * @brief       ST's LSM6DS3 6-axis motion sensor driver implementation
 * @author      EP <ep@unwds.com>
 */

#include "lsm6ds3.h"
#include "lsm6ds3_regs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

static bool read_register(lsm6ds3_t *dev, uint8_t *result, uint8_t reg)
{
    i2c_acquire(dev->params.i2c);

    if (i2c_read_reg(dev->params.i2c, dev->params.i2c_addr, reg, (char *) result, 0) < 0) {
        i2c_release(dev->params.i2c);
        return false;
    }

    i2c_release(dev->params.i2c);

    return true;
}

bool read_register_int16(lsm6ds3_t *dev, int16_t *result, uint8_t reg)
{
    i2c_acquire(dev->params.i2c);

    if (i2c_read_regs(dev->params.i2c, dev->params.i2c_addr, reg, (void *) result, 2, 0) < 0) {
        i2c_release(dev->params.i2c);
        return false;
    }

    i2c_release(dev->params.i2c);

    return true;
}

bool write_register(lsm6ds3_t *dev, uint8_t reg, uint8_t data)
{
    i2c_acquire(dev->params.i2c);

    if (i2c_write_reg(dev->params.i2c, dev->params.i2c_addr, reg, data, 0) < 0) {
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

static bool lsm6ds3_configure(lsm6ds3_t *dev)
{
    assert(dev != NULL);

    uint8_t data = 0;

    /* Setup the accelerometer */
    data = 0;
    if (dev->params.accel_enabled) {
        data |= dev->params.accel_bandwidth;
        data |= dev->params.accel_range;
        data |= dev->params.accel_sample_rate;
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
    if (dev->params.accel_odr_off == 1) {
        data |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
    }

    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL4_C, data)) {
        return false;
    }
    
    /* change byte order */
    data = 0;
    // data |= LSM6DS3_ACC_GYRO_BLE_MSB;
    data |= LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL3_C, data)) {
        return false;
    }

    /* Setup the gyro */
    data = 0;
    if (dev->params.gyro_enabled) {
        data |= dev->params.gyro_range;
        data |= dev->params.gyro_sample_rate;
    }

    /* Write down the settings */
    if (!write_register(dev, LSM6DS3_ACC_GYRO_CTRL2_G, data)) {
        return false;
    }

    return true;
}

int lsm6ds3_init(lsm6ds3_t *dev)
{
    assert(dev != NULL);
    
    /* Initialize I2C bus */
    i2c_acquire(dev->params.i2c);
    // if (i2c_init_master(dev->params.i2c, I2C_SPEED_NORMAL) < 0) {
    //     i2c_release(dev->params.i2c);
    //     return -1;
    // }
    i2c_init(dev->params.i2c);
    i2c_release(dev->params.i2c);
    
    /* Check device ID */
    uint8_t id;
    read_register(dev, &id, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

    if (id != LSM6DS3_DEFAULT_ID) {
        return -3;
    }
    
    if (!lsm6ds3_configure(dev)) {
        return -4;
    }

    return 0;
}

static bool read_raw_accel_x(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTX_L_XL);
}

static bool read_raw_accel_y(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTY_L_XL);
}

static bool read_raw_accel_z(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTZ_L_XL);
}

static bool read_raw_gyro_x(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTX_L_G);
}

static bool read_raw_gyro_y(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTY_L_G);
}

static bool read_raw_gyro_z(lsm6ds3_t *dev, int16_t *data)
{
    return read_register_int16(dev, data, LSM6DS3_ACC_GYRO_OUTZ_L_G);
}

static inline int convert_acc(lsm6ds3_t *dev, int16_t data)
{
    int range = 0;
    switch(dev->params.accel_range) {
        case (LSM6DS3_ACC_GYRO_FS_XL_2g):
            range = 61;
            break;
        case (LSM6DS3_ACC_GYRO_FS_XL_4g):
            range = 122;
            break;
        case (LSM6DS3_ACC_GYRO_FS_XL_8g):
            range = 244;
            break;
        case (LSM6DS3_ACC_GYRO_FS_XL_16g):
            range = 488;
            break;
    }
    
    /* result in mg */
    int result = ((int)data * range) / 1000;
    return result;
}

static inline int32_t convert_gyr(lsm6ds3_t *dev, int16_t data) {
    int32_t range = 0;
    switch(dev->params.gyro_range) {
        case (LSM6DS3_ACC_GYRO_FS_G_245dps):
            range = 875;
            break;
        case (LSM6DS3_ACC_GYRO_FS_G_500dps):
            range = 1750;
            break;
        case (LSM6DS3_ACC_GYRO_FS_G_1000dps):
            range = 3500;
            break;
        case (LSM6DS3_ACC_GYRO_FS_G_2000dps):
            range = 7000;
            break;
    }

    /* result in mdps */
    return (((int32_t)data * range) / 1000);
}

bool lsm6ds3_read_acc(lsm6ds3_t *dev, lsm6ds3_data_t *data)
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
    
    return true;
}
    
bool lsm6ds3_read_gyro(lsm6ds3_t *dev, lsm6ds3_data_t *data)
{
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

int lsm6ds3_read_temp(lsm6ds3_t *dev)
{
    int16_t out = 0;
    read_register_int16(dev, &out, LSM6DS3_ACC_GYRO_OUT_TEMP_L);
   
    /* millicelsius */
    return (25000 + ((int)out*1000)/16);
}

#ifdef __cplusplus
}
#endif
