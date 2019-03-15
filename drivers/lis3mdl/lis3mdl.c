/*
 * Copyright (C) 2015 HAW Hamburg
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis3mdl
 * @{
 *
 * @file
 * @brief       Device driver implementation for the LIS3MDL 3-axis magnetometer
 *
 * @author      René Herthel <rene-herthel@outlook.de>
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include "lis3mdl.h"
#include "include/lis3mdl-internal.h"

#define ENABLE_DEBUG        (0)

#include "debug.h"

#define MASK_INT16_MSB      (0x8000)
#define MASK_INT16_NMSB     (0x7FFF)

#define TEMP_DIVIDER        (8)
#define TEMP_OFFSET         (25)

#define GAUSS_DIVIDER       (1000)

#define DEV_I2C             (dev->params.i2c)
#define DEV_ADDR            (dev->params.addr)

/**
 * @brief Takes an unsigned value representing a two's complement number
 *        and returns the signed number it represents
 *
 * @param[in] value    value which represents a two's complement number
 *
 * @return             the converted signed number of 'value'
 */
 static inline int16_t _twos_complement(int16_t value)
{
    if (value & MASK_INT16_MSB) {
        value = ~(value & MASK_INT16_NMSB) + 1;
        return ~(value & MASK_INT16_NMSB);
    }
    else {
        return value;
    }
}

int lis3mdl_init(lis3mdl_t *dev, const lis3mdl_params_t *params)
{
    dev->params = *params;

    uint8_t tmp = 0x00;

    i2c_acquire(DEV_I2C);
    i2c_init(DEV_I2C);

    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3DML_WHO_AM_I_REG, &tmp, 0);
    if (tmp != LIS3MDL_CHIP_ID) {
        DEBUG("LIS3MDL: Identification failed, %02X != %02X\n",
              tmp, LIS3MDL_CHIP_ID);
        return -1;
    }

    tmp = ( LIS3MDL_MASK_REG1_TEMP_EN   /* enable temperature sensor */
          | dev->params.xy_mode         /* set x-, y-axis operative mode */
          | dev->params.odr);           /* set output data rate */
    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG1, tmp, 0);

    /* set Full-scale configuration */
    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG2, dev->params.scale, 0);

    /* set continuous-conversion mode */
    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG3, dev->params.op_mode, 0);

    /* set z-axis operative mode */
    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG4, dev->params.z_mode, 0);

    /* enable BDU (block data update) */ 
    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG5, LIS3MDL_MASK_REG5_BDU, 0);
    
    tmp = 0xff;
    i2c_write_reg(DEV_I2C, DEV_ADDR, 0x1E, tmp, 0);

    i2c_release(DEV_I2C);

    return 0;
}

void lis3mdl_read_mag(const lis3mdl_t *dev, lis3mdl_3d_data_t *data)
{
    uint8_t tmp[2] = {0, 0};
    uint8_t status = 0;

    i2c_acquire(DEV_I2C);

    /* wait for data to be ready for all axes */
    do {
        i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_STATUS_REG, &status, 0);
    } while (!(status & LIS3MDL_MASK_STATUS_REG_ZYXDA));


    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_X_L_REG, &tmp[0], 0);
    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_X_H_REG, &tmp[1], 0);

    DEBUG("LIS3MDL: OUT_X %02X %02X\n", tmp[1], tmp[0]);

    int16_t x = ((tmp[1] << 8) | tmp[0]);
    x = _twos_complement(x);

    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_Y_L_REG, &tmp[0], 0);
    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_Y_H_REG, &tmp[1], 0);

    DEBUG("LIS3MDL: OUT_Y %02X %02X\n", tmp[1], tmp[0]);

    int16_t y = ((tmp[1] << 8) | tmp[0]);
    y = _twos_complement(y);

    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_Z_L_REG, &tmp[0], 0);
    i2c_read_reg(DEV_I2C, DEV_ADDR, LIS3MDL_OUT_Z_H_REG, &tmp[1], 0);

    DEBUG("LIS3MDL: OUT_Z %02X %02X\n", tmp[1], tmp[0]);

    int16_t z = ((tmp[1] << 8) | tmp[0]);
    z = _twos_complement(z);

    int32_t scale = 0;
    switch (dev->params.scale) {
        case LIS3MDL_SCALE_4G:
            scale = 6842;
            break;
        case LIS3MDL_SCALE_8G:
            scale = 3421;
            break;
        case LIS3MDL_SCALE_12G:
            scale = 2281;
            break;
        case LIS3MDL_SCALE_16G:
            scale = 1711;
            break;
        default:
            scale = 1;
            break;
    }

    data->x_axis = (1000 * (int32_t)x)/scale;
    data->y_axis = (1000 * (int32_t)y)/scale;
    data->z_axis = (1000 * (int32_t)z)/scale;

    i2c_release(DEV_I2C);
}

void lis3mdl_read_temp(const lis3mdl_t *dev, int16_t *value)
{
    uint8_t tmp[2] = {0, 0};

    i2c_acquire(DEV_I2C);
    i2c_read_regs(DEV_I2C, DEV_ADDR, LIS3MDL_TEMP_OUT_L_REG, (uint8_t*)value, 2, 0);
    i2c_release(DEV_I2C);

    DEBUG("LIS3MDL: TEMP_OUT %02X %02X\n", tmp[1], tmp[0]);

    *value = ((tmp[1] << 8) | tmp[0]);
    DEBUG("LIS3MDL: value %d(%04X)\n", *value, *value);

    *value = _twos_complement(*value);
    DEBUG("LIS3MDL: value %d\n", *value);

    *value = (TEMP_OFFSET + (*value / TEMP_DIVIDER));
    DEBUG("LIS3MDL: value %d\n", *value);
}

void lis3mdl_poweron(const lis3mdl_t *dev)
{
    i2c_acquire(DEV_I2C);

    uint8_t tmp;
    i2c_read_regs(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG3, &tmp, 1, 0);
    tmp &= ~LIS3MDL_MASK_REG3_MODE;
    tmp |= dev->params.op_mode;

    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG3, tmp, 0);

    i2c_release(DEV_I2C);
}

void lis3mdl_poweroff(const lis3mdl_t *dev)
{
    i2c_acquire(DEV_I2C);
    uint8_t tmp;

    i2c_read_regs(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG3, &tmp, 1, 0);
    tmp &= ~LIS3MDL_MASK_REG3_MODE;
    tmp |= LIS3MDL_OP_PDOWN;

    i2c_write_reg(DEV_I2C, DEV_ADDR, LIS3MDL_CTRL_REG3, tmp, 0);
    i2c_release(DEV_I2C);
}
