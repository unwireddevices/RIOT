/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis2dh12
 * @{
 *
 * @file
 * @brief       LIS2DH12 accelerometer driver implementation
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 * @}
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "xtimer.h"

#include "assert.h"

#include "lis2dh12.h"
#include "lis2dh12_internal.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#include "log.h"

#if ENABLE_DEBUG
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        while (len) {
            len--;
            printf("%02Xh ", *buff++);
        }
        printf("\n");
    }
#else
    #define PRINTBUFF(...)
#endif

/**
 * @brief Write a data to registers in the LIS2DH12.
 *
 * @param[in] dev     Device descriptor
 * @param[in] reg     The source register starting address
 * @param[in] data    The values of the source registers will be written here
 * @param[in] length  Number of bytes to write
 * 
 * @return            Error status
 */
static int _write(const lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief Read sequential registers from the LIS2DH12.
 *
 * @param[in]  dev     Device descriptor
 * @param[in]  reg     The source register starting address
 * @param[out] data    The values of the source registers will be written here
 * @param[in]  length  Number of bytes to read
 *
 * @return             Error status
 */
static int _read(const lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief  Read generic device register
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to read
 * @param  data   Pointer to buffer that store the data read
 * @param  len    Number of consecutive register to read
 * 
 * @return        Error status
 */
static int _lis2dh12_read_reg(lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief  Write generic device register
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to write
 * @param  data   Pointer to data to write in register reg
 * @param  len    Number of consecutive register to write
 * 
 * @return        Error status
 */
static int _lis2dh12_write_reg(lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t len);



#define I2C_AUTO_INCREMENT                              (0x80)

static int _read(const lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Read multiple command */
    reg &= ~(I2C_AUTO_INCREMENT);
    if (length > 1) {
        reg |= I2C_AUTO_INCREMENT;
    }

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    status = i2c_read_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

    DEBUG("LIS2DH12 [REG %02X]: <- ", (reg & 0x7F));
    PRINTBUFF(data, length);

    return status;
}

static int _write(const lis2dh12_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Write multiple command */
    reg &= ~(I2C_AUTO_INCREMENT);
    if (length > 1) {
        reg |= I2C_AUTO_INCREMENT;
    }

    DEBUG("LIS2DH12 [REG %02X]: -> ", (reg & 0x7F));
    PRINTBUFF(data, length);

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    status = i2c_write_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);
    return status;
}

static int _lis2dh12_read_reg(lis2dh12_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    int err_code;

    err_code = _read(dev, reg, data, len);

    return err_code;
}

static int _lis2dh12_write_reg(lis2dh12_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    int err_code;

    err_code = _write(dev, reg, data, len);

    return err_code;
}

static int16_t _lis2dh12_calculation_acceleration(lis2dh12_t *dev, const int16_t acc_raw) 
{
    DEBUG("acc raw data: %d[%04X]\n", acc_raw, acc_raw);
    switch (dev->params.res) {
        case LIS2DH12_HR_12BIT:
            switch (dev->params.scale) {
                case LIS2DH12_SCALE_2G:
                    return ((acc_raw >> 4) * 1);
                    break;
                case LIS2DH12_SCALE_4G:
                    return ((acc_raw >> 4) * 2);
                    break;
                case LIS2DH12_SCALE_8G:
                    return ((acc_raw >> 4) * 4);
                    break;
                case LIS2DH12_SCALE_16G:
                    return ((acc_raw >> 4) * 12);
                    break;
            } 
            break;
        case LIS2DH12_NM_10BIT:
            switch (dev->params.scale) {
                case LIS2DH12_SCALE_2G:
                    return ((acc_raw >> 6) * 4);
                    break;
                case LIS2DH12_SCALE_4G:
                    return ((acc_raw >> 6) * 8);
                    break;
                case LIS2DH12_SCALE_8G:
                    return ((acc_raw >> 6) * 16);
                    break;
                case LIS2DH12_SCALE_16G:
                    return ((acc_raw >> 6) * 48);
                    break;
            }
            break;
        case LIS2DH12_LP_8BIT:
            switch (dev->params.scale) {
                case LIS2DH12_SCALE_2G:
                    return ((acc_raw >> 8) * 16);
                    break;
                case LIS2DH12_SCALE_4G:
                    return ((acc_raw >> 8) * 32);
                    break;
                case LIS2DH12_SCALE_8G:
                    return ((acc_raw >> 8) * 64);
                    break;
                case LIS2DH12_SCALE_16G:
                    return ((acc_raw >> 8) * 192);
                    break;
            }
            break;
        default:
            return 0x7FFF;
    }
    return 0x7FFF;
}

static int16_t _lis2dh12_calculation_temperature(lis2dh12_t *dev, const int16_t temp_raw) 
{
    switch (dev->params.res) {
        case LIS2DH12_HR_12BIT:
            return (((temp_raw >> 6) / 4) + 25);
            break;
        case LIS2DH12_NM_10BIT:
            return (((temp_raw >> 6) / 4) + 25);
            break;
        case LIS2DH12_LP_8BIT:
            return (((temp_raw >> 8) * 1) + 25);
            break;
        default:
            return 0x7FFF;
            break;
    }
    return 0x7FFF;
}

int lis2dh12_init(lis2dh12_t *dev, const lis2dh12_params_t *params)
{
    uint8_t dev_id;
    uint8_t reg_val;

    /* Initialization of parameters dependent on the connection interface */
    dev->params = *params;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);

    /* Initialize I2C interface */
    i2c_init(dev->params.i2c_dev);

    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

    /* Sensor availability check */
    DEBUG("Sensor availability check\n");
    if (_lis2dh12_read_reg(dev, LIS2DH12_WHO_AM_I, &dev_id, 1) < 0) {
        LOG_ERROR("[LIS2DH12] Sensor is not available\n");
        return -LIS2DH12_ERROR_I2C;
    }

    /* Checking the value of the "Who am I" register */
    DEBUG("Checking the value of the \"Who am I\" register\n");
    if (dev_id != LIS2DH12_WHO_AM_I_RESPONSE) {
        /* the chip responded incorrectly */
        LOG_ERROR("[LIS2DH12] Error reading chip ID\n");
        return -LIS2DH12_ERROR_NO_DEV;
    }

    /* Explicitly set the LIS2DH12 in power-down mode */
    DEBUG("Explicitly set the LIS2DH12 in power-down mode\n");
    reg_val = LIS2DH12_CTRL_REG1_LPEN_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Clear any interrupts */
    DEBUG("Clear any interrupts\n");
    uint8_t dummy;
    if (_lis2dh12_read_reg(dev, LIS2DH12_REG_INT1_SRC, &dummy, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Disable Pull-Up */
    DEBUG("Disable Pull-Up\n");
    reg_val = (LIS2DH12_CTRL_REG0_SDO_PU_DISC_MASK | LIS2DH12_CTRL_REG0_CORRECT_OPER_MASK);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG0, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Enable all axis */
    DEBUG("Enable all axis\n");
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_XYZEN_MASK);
    reg_val |= LIS2DH12_CTRL_REG1_XYZEN_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1)< 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Enable Block Data Update */
    DEBUG("Enable Block Data Update\n");
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG4_BDU_MASK);
    reg_val |= LIS2DH12_CTRL_REG4_BDU_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set Big endian output value */ 
    DEBUG("Set Big endian output value\n");
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG4_BLE_MASK);
    reg_val |= LIS2DH12_CTRL_REG4_BLE_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set full scale */ 
    DEBUG("Set full scale [%d]\n", dev->params.scale); 
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG4_FS_MASK);
    reg_val |= (uint8_t)(dev->params.scale << LIS2DH12_CTRL_REG4_FS_SHIFT);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Enable temperature sensor */
    DEBUG("Enable temperature sensor\n");
    reg_val = 0x00;
    /* Required in order to use */
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val |= LIS2DH12_CTRL_REG4_BDU_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG4, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val = 0x00;
    reg_val = (LIS2DH12_TEMP_CFG_REG_TEMP_EN_MASK);
    if (_lis2dh12_write_reg(dev, LIS2DH12_TEMP_CFG_REG, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set device operation mode */
    DEBUG("Set device operation mode: ");
    uint8_t ctrl_reg1 = 0x00;
    uint8_t ctrl_reg4 = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &ctrl_reg1, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG4, &ctrl_reg4, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    if (dev->params.res == LIS2DH12_HR_12BIT) {
        DEBUG("LIS2DH12_HR_12BIT\n");
        ctrl_reg1 &= ~(LIS2DH12_CTRL_REG1_LPEN_MASK);
        ctrl_reg4 |= (LIS2DH12_CTRL_REG4_HR_MASK);
    }
    if (dev->params.res == LIS2DH12_NM_10BIT) {
        DEBUG("LIS2DH12_NM_10BIT\n");
        ctrl_reg1 &= ~(LIS2DH12_CTRL_REG1_LPEN_MASK);
        ctrl_reg4 &= ~(LIS2DH12_CTRL_REG4_HR_MASK);
    }
    if (dev->params.res == LIS2DH12_LP_8BIT) {
        DEBUG("LIS2DH12_LP_8BIT\n");
        ctrl_reg1 |= (LIS2DH12_CTRL_REG1_LPEN_MASK);
        ctrl_reg4 &= ~(LIS2DH12_CTRL_REG4_HR_MASK);
    }
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &ctrl_reg1, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG4, &ctrl_reg4, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set Output Data Rate */
    DEBUG("Set Output Data Rate [%d]\n", dev->params.rate);
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_ODR_MASK); 
    reg_val |= (uint8_t)(dev->params.rate << LIS2DH12_CTRL_REG1_ODR_SHIFT);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    return LIS2DH12_OK;
}

int lis2dh12_read_xyz(lis2dh12_t *dev, lis2dh12_acc_t *acceleration) {
    uint8_t axl_data_rdy;
    uint8_t axl_data_ovr;
    uint16_t tick = 0xFFFF;

    /* Read output only if new value available */
    do {
        if (_lis2dh12_read_reg(dev, LIS2DH12_STATUS_REG, &axl_data_rdy, 1) < 0) {
            return -LIS2DH12_ERROR_I2C;
        }
        if ((axl_data_rdy & LIS2DH12_STATUS_REG_ZYXDA_MASK) == LIS2DH12_STATUS_REG_ZYXDA_MASK) {
            DEBUG("[LIS2DH12] Bit ZYXDA STATUS_REG is set\n");
            /* Read raw accelerometer data */
            uint8_t acc_raw[6] = {0x00};
            if (_lis2dh12_read_reg(dev, LIS2DH12_REG_OUT_X_L, acc_raw, 6) < 0) {
                return -LIS2DH12_ERROR_I2C;
            }
            DEBUG("[LIS2DH12] Acceleration Raw Data [mg]: ");
            PRINTBUFF(acc_raw, 6);
            /* Calculation of acceleration values */
            acceleration->axis_x = _lis2dh12_calculation_acceleration(dev, ((((int16_t)acc_raw[0]) << 8) | acc_raw[1]));
            acceleration->axis_y = _lis2dh12_calculation_acceleration(dev, ((((int16_t)acc_raw[2]) << 8) | acc_raw[3]));
            acceleration->axis_z = _lis2dh12_calculation_acceleration(dev, ((((int16_t)acc_raw[4]) << 8) | acc_raw[5]));
            DEBUG("Acceleration [mg]:%d\t%d\t%d\n", acceleration->axis_x, acceleration->axis_y, acceleration->axis_z);
        }
        /* Checking the overflow bit */
        if (_lis2dh12_read_reg(dev, LIS2DH12_STATUS_REG, &axl_data_ovr, 1) < 0) { 
            return -LIS2DH12_ERROR_I2C;
        }
        if ((axl_data_ovr & LIS2DH12_STATUS_REG_ZYXOR_MASK) == LIS2DH12_STATUS_REG_ZYXOR_MASK) {
            DEBUG("[LIS2DH12] Bit ZYXOR STATUS_REG is set\n");
            uint8_t dummy[6] = {0x00};
            if (_lis2dh12_read_reg(dev, LIS2DH12_REG_OUT_X_L, dummy, 6) < 0) {
                return -LIS2DH12_ERROR_I2C;
            }
        }
    } while(((axl_data_rdy & LIS2DH12_STATUS_REG_ZYXDA_MASK) != LIS2DH12_STATUS_REG_ZYXDA_MASK) && --tick);

    if (!tick) {
        LOG_ERROR("[LIS2DH12] Timeout reading acceleration\n");
        return -LIS2DH12_ERROR_NO_NEW_DATA;
    }
    return LIS2DH12_OK;
}

int lis2dh12_read_temp(lis2dh12_t *dev, int16_t *temperature_degC) 
{
    uint8_t temp_data_rdy;
    uint8_t temp_data_ovr;
    uint8_t temp_raw[2] = {0x00};
    uint16_t tick = 0xFFFF;

    /* Read output only if new value available */
    do {
        if (_lis2dh12_read_reg(dev, LIS2DH12_STATUS_REG_AUX, &temp_data_rdy, 1) < 0) {
            return -LIS2DH12_ERROR_I2C;;
        }
        if ((temp_data_rdy & LIS2DH12_STATUS_REG_AUX_TDA_MASK) == LIS2DH12_STATUS_REG_AUX_TDA_MASK) {
            /* Read raw temperature data */
            if (_lis2dh12_read_reg(dev, LIS2DH12_OUT_TEMP_L, temp_raw, 2) < 0) {
                return -LIS2DH12_ERROR_I2C;;
            }
            DEBUG("Temperature: %04Xh\n", ((((int16_t)temp_raw[0]) << 8) | temp_raw[1]));
            *temperature_degC = _lis2dh12_calculation_temperature(dev, ((((int16_t)temp_raw[0]) << 8) | temp_raw[1]));
            DEBUG("Temperature [degC]: %d\n", *temperature_degC);
        }
        /* Checking the overflow bit */
        if (_lis2dh12_read_reg(dev, LIS2DH12_STATUS_REG_AUX, &temp_data_ovr, 1) < 0) { 
            return -LIS2DH12_ERROR_I2C;
        }
        /* Required to clean the overflow bit */
        if ((temp_data_ovr & LIS2DH12_STATUS_REG_AUX_TDO_MASK) == LIS2DH12_STATUS_REG_AUX_TDO_MASK) {
            uint8_t dummy[2] = {0x00};
            DEBUG("Temperature register status overflow\n");
            if (_lis2dh12_read_reg(dev, LIS2DH12_OUT_TEMP_L, dummy, 2) < 0) {
                return -LIS2DH12_ERROR_I2C;;
            }
        }
    } while(((temp_data_rdy & LIS2DH12_STATUS_REG_AUX_TDA_MASK) != LIS2DH12_STATUS_REG_AUX_TDA_MASK) && --tick);

    if (!tick) {
        LOG_ERROR("[LIS2DH12] Timeout reading temperature\n");
         return -LIS2DH12_ERROR_NO_NEW_DATA;
    }
    return LIS2DH12_OK;
}

int lis2dh12_power_on(lis2dh12_t *dev) 
{
    /* Enable all axis */
    uint8_t reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_XYZEN_MASK);
    reg_val |= LIS2DH12_CTRL_REG1_XYZEN_MASK;
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1)< 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set Output Data Rate */
    DEBUG("Set Output Data Rate [%d]\n", dev->params.rate);
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_ODR_MASK); 
    reg_val |= (uint8_t)(dev->params.rate << LIS2DH12_CTRL_REG1_ODR_SHIFT);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    if (dev->params.rate != LIS2DH12_RATE_POWER_DOWN) {
        uint32_t power_on_delay_us;
        
        switch (dev->params.res) {
            case LIS2DH12_HR_12BIT: {
                switch (dev->params.rate) {
                    case LIS2DH12_RATE_10HZ:
                        power_on_delay_us = 7000/10;
                        break;
                    case LIS2DH12_RATE_25HZ:
                        power_on_delay_us = 7000/25;
                        break;
                    case LIS2DH12_RATE_50HZ:
                        power_on_delay_us = 7000/50;
                        break;
                    case LIS2DH12_RATE_100HZ:
                        power_on_delay_us = 7000/100;
                        break;
                    case LIS2DH12_RATE_200HZ:
                        power_on_delay_us = 7000/200;
                        break;
                    case LIS2DH12_RATE_400HZ:
                        power_on_delay_us = 7000/400;
                        break;
                    case LIS2DH12_RATE_5376Hz:
                        power_on_delay_us = 7000/1344;
                        break;
                    default:
                        power_on_delay_us = 7000;
                        break;
                }
            }
            case LIS2DH12_NM_10BIT:
                power_on_delay_us = 1600;
                break;
            case LIS2DH12_LP_8BIT:
                power_on_delay_us = 1000;
                break;
            default:
                power_on_delay_us = 7000;
                break;
        }
        
        xtimer_spin(xtimer_ticks_from_usec(power_on_delay_us));

        /* Discard first measurement after power-on*/
        uint8_t unused[6] = {0x00};
        if (_lis2dh12_read_reg(dev, LIS2DH12_REG_OUT_X_L, unused, 6) < 0) {
            return -LIS2DH12_ERROR_I2C;
        }
    }
    return LIS2DH12_OK;
}

int lis2dh12_power_off(lis2dh12_t *dev)
{
    /* Disable all axis */
    uint8_t reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_XYZEN_MASK);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1)< 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    /* Set Output Data Rate POWER_DOWN*/
    DEBUG("Set Output Data Rate to POWER_DOWN\n");
    reg_val = 0x00;
    if (_lis2dh12_read_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }
    reg_val &= ~(LIS2DH12_CTRL_REG1_ODR_MASK); 
    reg_val |= (uint8_t)(LIS2DH12_RATE_POWER_DOWN << LIS2DH12_CTRL_REG1_ODR_SHIFT);
    if (_lis2dh12_write_reg(dev, LIS2DH12_CTRL_REG1, &reg_val, 1) < 0) {
        return -LIS2DH12_ERROR_I2C;
    }

    return LIS2DH12_OK;
}
