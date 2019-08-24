/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fdc2212
 * @brief       Device Driver for TI FDC2212 digital capacitor sensor
 * @author      Alexander Ugorelov <info@unwds.com>
 * @file
 */

#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "log.h"
#include "lptimer.h"

#include "fdc2212_regs.h"
#include "fdc2212.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


/**
 * Internal macro definitions
 */

#define ASSERT_PARAM(cond) \
    if (!(cond)) { \
        DEBUG("[fdc2212] %s: %s\n", \
              __func__, "parameter condition (" # cond ") not fulfilled"); \
        assert(cond); \
    }

#define DEBUG_DEV(f, d, ...) \
    DEBUG("[fdc2212] %s dev=%d addr=%02x: " f "\n", \
          __func__, d->params.i2c_dev, d->params.i2c_addr, ## __VA_ARGS__)

/**
 * forward declaration of functions for internal use only
 */
static int _reg_read(const fdc2212_t *dev, uint8_t reg, uint8_t *data, uint32_t len);
static int _reg_write(const fdc2212_t *dev, uint8_t reg, uint8_t *data, uint32_t len);
static int _check_error_status(const fdc2212_t *dev);
static int _error_code(const fdc2212_t *dev, uint16_t err_reg);
static int _is_available(const fdc2212_t *dev);

int fdc2212_init(fdc2212_t *dev, const fdc2212_params_t *params)
{
    ASSERT_PARAM(dev != NULL);
    ASSERT_PARAM(params != NULL);

    /* Init sensor data structure */
    dev->params = *params;

    int res = FDC2212_OK;

    if (dev->params.shutdown_pin != GPIO_UNDEF && 
        gpio_init(dev->params.shutdown_pin, GPIO_OUT) == 0) {
        DEBUG_DEV("Shutdown pin configured", dev);
        /* Send a high signal to the shutdown pin */
        gpio_set(dev->params.shutdown_pin);
        lptimer_usleep(10000);
        /* Send a low signal to the shutdown pin */
        gpio_clear(dev->params.shutdown_pin);
        /* Wake-up time from SD high-low transition to I2C readback 2ms */
        lptimer_usleep(2000);
    }

    /* check whether sensor is available including the check of the device id */
    if ((res = _is_available(dev)) != FDC2212_OK) {
        return res;
    }

    /* doing a software reset first */
    uint16_t sw_reset = 0x00;
    if (_reg_reg(dev, FDC2212_REG_RESET_DEV, (uint8_t *)&sw_reset, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_RESET_DEV", dev);
        return -FDC2212_ERROR_I2C;
    }
    sw_reset |= FDC2212_REG_RESET_DEV_MASK;
    if (_reg_write(dev, FDC2212_REG_RESET_DEV, (uint8_t *)&sw_reset, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't write the command of a software reset in register FDC2212_REG_RESET_DEV", dev);
        return -FDC2212_ERROR_I2C;
    }

    /* Entering sleep mode */
    uint16_t sleep = 0x00;
    if (_reg_reg(dev, FDC2212_REG_CONFIG, (uint8_t *)&sleep, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
    sleep |= FDC2212_REG_CONFIG_SLEEP_MODE_EN_MASK;
    if (_reg_write(dev, FDC2212_REG_CONFIG, (uint8_t *)&sleep, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't write the command of a sleeping mode in register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }

    /* Setting the clocking */
    for (uint8_t i = 0; i < FDC2212_CHANNELS; i++) {
        /* Setting Reference Count */
        if (dev->ref_count[i] < 0x0100) {
            DEBUG_DEV("invalid setting for RCOUNT for channel %d", i, dev);
            return -FDC2212_ERROR_SETTING_INV;
        }
        if (_reg_write(dev, FDC2212_REG_CONFIG + i, (uint8_t *)&dev->ref_count[i], 2) != FDC2212_OK) {
            DEBUG_DEV("couldn't write the setting for RCOUNT in register FDC2212_REG_RCOUNT_CH%d", i, dev);
            return -FDC2212_ERROR_I2C;
        }
        /* Setting Settle Count */
        if (_reg_write(dev, FDC2212_REG_SETTLECOUNT_CH0 + i, (uint8_t *)&dev->settle_count[i], 2) != FDC2212_OK) {
            DEBUG_DEV("couldn't write the setting for SETTLECOUNT in register FDC2212_REG_SETTLECOUNT_CH%d", i, dev);
            return -FDC2212_ERROR_I2C;
        }
        /* Setting the frequency divider */
        if (dev->freq_divider[i] == 0x0000) {
            DEBUG_DEV("invalid setting for CH%d_FREF_DIVIDER for channel %d", i, i, dev);
            return -FDC2212_ERROR_SETTING_INV;
        }
        dev->freq_divider[i] &= FDC2212_CLOCK_DIVIDERS_CH1_FREF_DIVIDER_MASK;
        dev->freq_in_sel[i] &= 0x03;
        uint16_t reg_dividers = ((dev->freq_in_sel[i] << FDC2212_CLOCK_DIVIDERS_CH0_FIN_SEL_SHIFT) | 
                                 dev->freq_divider[i] &= FDC2212_CLOCK_DIVIDERS_CH0_FREF_DIVIDER_MASK);
        if (_reg_write(dev, FDC2212_REG_CLOCK_DIVIDERS_CH0 + i, (uint8_t *)&reg_dividers, 2) != FDC2212_OK) {
            DEBUG_DEV("couldn't write the setting for FREF_DIVIDER in register FDC2212_REG_CLOCK_DIVIDERS_CH%d", i, dev);
            return -FDC2212_ERROR_I2C;
        }
    }

    /* Select single channel measurement and configure */
    uint16_t reg_config = 0x0000;
    if (_reg_reg(dev, FDC2212_REG_CONFIG, (uint8_t *)&reg_config, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
    reg_config &= ~(FDC2212_REG_CONFIG_REF_CLK_SRC_MASK | 
                    FDC2212_REG_CONFIG_HIGH_CURRENT_DRV_MASK);
    
    reg_config |= (FDC2212_REG_CONFIG_SLEEP_MODE_EN_MASK | 
                   FDC2212_REG_CONFIG_SENSOR_ACTIVATE_SEL_MASK | 
                   FDC2212_REG_CONFIG_INTB_DIS_MASK);
    /* Set reserve bits to 1 per datasheet */
    reg_config |= (FDC2212_REG_CONFIG_OP_MODE0_MASK |
                   FDC2212_REG_CONFIG_OP_MODE1_MASK |
                   FDC2212_REG_CONFIG_OP_MODE3_MASK);

    if (_reg_write(dev, FDC2212_REG_CONFIG, (uint8_t *)&reg_config, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't write the setting in register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
    /* Select measurement sequence */
    uint16_t reg_mux_config = 0x0000;
    if (_reg_reg(dev, FDC2212_REG_CONFIG, (uint8_t *)&reg_mux_config, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
    reg_config |= (FDC2212_REG_MUX_CONFIG_AUTOSCAN_EN_MASK | 
                   FDC2212_REG_MUX_CONFIG_RR_SEQUENCE_MASK | 
                   (FDC2212_DEGLITCH_10MHZ << FDC2212_REG_MUX_CONFIG_DEGLITCH_SHIFT));
    /* Set reserve bits to 1 per datasheet */
    reg_config |= (FDC2212_REG_MUX_CONFIG_RESERVED_MASK);
    if (_reg_write(dev, FDC2212_REG_MUX_CONFIG, (uint8_t *)&reg_mux_config, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't write the setting in register FDC2212_REG_MUX_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }

    /* Adjust drive current for better sensitivity */
    for (uint8_t i = 0; i < FDC2212_CHANNELS; i++) {
        uint8_t reg_idrive = (dev->idrive[i] << FDC2212_REG_DRIVE_CURRENT_CH0_IDRIVE_SHIFT) & 
                             FDC2212_REG_DRIVE_CURRENT_CH0_IDRIVE_MASK;
        if (_reg_write(dev, FDC2212_REG_DRIVE_CURRENT_CH0 + i, (uint8_t *)&reg_idrive, 2) != FDC2212_OK) {
            DEBUG_DEV("couldn't write the setting in register FDC2212_REG_DRIVE_CURRENT_CH0", dev);
            return -FDC2212_ERROR_I2C;
        }
    }

    /* Wake up */
    /* Only change SLEEP MODE BIT */
    sleep = 0x00;
    if (_reg_reg(dev, FDC2212_REG_CONFIG, (uint8_t *)&sleep, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
    sleep &= ~(FDC2212_REG_CONFIG_SLEEP_MODE_EN_MASK);
    if (_reg_write(dev, FDC2212_REG_CONFIG, (uint8_t *)&sleep, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't write the command of a sleeping mode in register FDC2212_REG_CONFIG", dev);
        return -FDC2212_ERROR_I2C;
    }
}

int fdc2212_data_ready(const fdc2212_t *dev)
{
    ASSERT_PARAM(dev != NULL);

    uint8_t status;

    /* check status register */
    if (_reg_read(dev, FDC2212_REG_STATUS, &status, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_STATUS", dev);
        return -FDC2212_ERROR_I2C;
    }

    if ((status & FDC2212_REG_STATUS_DATA_RDY_MASK)) {
        /* new data available */
        return FDC2212_OK;
    }

    return -FDC2212_ERROR_NO_NEW_DATA;
}

int fdc2212_read_raw_data(const fdc2212_t *dev, uint8_t channel, uint32_t *raw_data)
{
    ASSERT_PARAM(dev != NULL);
    ASSERT_PARAM(raw_data != NULL);

    uint16_t reg_val = 0x0000;
    uint16_t raw_data_msb = 0x0000;
    uint16_t raw_data_lsb = 0x0000;

    //CHx measurement
    if (_reg_read(dev, FDC2212_REG_DATA_MSB_CH0 + channel, (uint8_t *)&reg_val, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_DATA_MSB_CH%d", channel, dev);
        return -FDC2212_ERROR_I2C;
    }
    /* Exclude the upper 4 bits  */
    raw_data_msb = reg_val & FDC2212_REG_DATA_MSB_CH0_MASK;

    reg_val = 0x0000;
    if (_reg_read(dev, FDC2212_REG_DATA_LSB_CH0 + channel, (uint8_t *)&reg_val, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_DATA_LSB_CH%d", channel, dev);
        return -FDC2212_ERROR_I2C;
    }
    raw_data_lsb = reg_val;

    *raw_data = (((((uint32_t)raw_data_msb) << 16) | raw_data_lsb) & 0x0FFFFFFF);

    _clear_data_rdy(dev);
    return FDC2212_OK;
}

/**
 * function for internal use only
 */
static int _reg_read(const fdc2212_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    DEBUG_DEV("read %"PRIu32" bytes from sensor registers starting at addr %02x",
              dev, len, reg);

    int res = FDC2212_OK;

    if (i2c_acquire(dev->params.i2c_dev) != FDC2212_OK) {
        DEBUG_DEV("could not aquire I2C bus", dev);
        return -FDC2212_ERROR_I2C;
    }

    res = i2c_read_regs(dev->params.i2c_dev, dev->params.i2c_addr, reg, data, len, 0);
    i2c_release(dev->params.i2c_dev);

    if (res == FDC2212_OK) {
        if (ENABLE_DEBUG) {
            printf("[fdc2212] %s dev=%d addr=%02x: read following bytes: ",
                   __func__, dev->params.i2c_dev, dev->params.i2c_addr);
            for (unsigned i = 0; i < len; i++) {
                    printf("%02x ", data[i]);
            }
            printf("\n");
        }
    }
    else {
        DEBUG_DEV("could not read %"PRIu32" bytes from sensor registers "
                  "starting at addr %02x, reason %i", dev, len, reg, res);
        return -FDC2212_ERROR_I2C;
    }

    return FDC2212_OK;
}

static int _reg_write(const fdc2212_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    DEBUG_DEV("write %"PRIu32" bytes to sensor registers starting at addr %02x",
              dev, len, reg);

    int res = FDC2212_OK;

    if (ENABLE_DEBUG && data && len) {
        printf("[fdc2212] %s dev=%d addr=%02x: write following bytes: ",
               __func__, dev->params.i2c_dev, dev->params.i2c_addr);
        for (unsigned i = 0; i < len; i++) {
            printf("%02x ", data[i]);
        }
        printf("\n");
    }

    if (i2c_acquire(dev->params.i2c_dev)) {
        DEBUG_DEV("could not aquire I2C bus", dev);
        return -FDC2212_ERROR_I2C;
    }

    res = i2c_write_regs(dev->params.i2c_dev, dev->params.i2c_addr, reg, data, len, 0);
    i2c_release(dev->params.i2c_dev);

    if (res != FDC2212_OK) {
        DEBUG_DEV("could not write %"PRIu32" bytes to sensor registers "
                  "starting at addr %02x, reason %i", dev, len, reg, res);
        return -FDC2212_ERROR_I2C;
    }

    return FDC2212_OK;
}

static int _error_code(const fdc2212_t *dev, uint16_t err_reg)
{
    if (((err_reg & FDC2212_REG_STATUS_ERR_CHAN_MASK) >> FDC2212_REG_STATUS_ERR_CHAN_SHIFT) == 0x00) {
        DEBUG_DEV("Channel 0 is source of flag or error", dev);
    } else if (((err_reg & FDC2212_REG_STATUS_ERR_CHAN_MASK) >> FDC2212_REG_STATUS_ERR_CHAN_SHIFT) == 0x01) {
        DEBUG_DEV("Channel 1 is source of flag or error", dev);
    }

    if (err_reg & FDC2212_REG_STATUS_ERR_WD_MASK) {
        DEBUG_DEV("Watchdog Timeout Error", dev);
    }

    if (err_reg & FDC2212_REG_STATUS_ERR_AHW_MASK) {
        DEBUG_DEV("Amplitude High Warning", dev);
    }

    if (err_reg & FDC2212_REG_STATUS_ERR_ALW_MASK) {
        DEBUG_DEV("Amplitude Low Warning", dev);
    }

    return FDC2212_OK;
}

static int _check_error_status(const fdc2212_t *dev)
{
    uint16_t status;

    /* check status register */
    if (_reg_read(dev, FDC2212_REG_STATUS, (uint8_t *)&status, 2) != FDC2212_OK) {
        DEBUG_DEV("could not read CCS811_REG_STATUS", dev);
        return -FDC2212_ERROR_I2C;
    }

    if (!(status & FDC2212_REG_STATUS_ERROR)) {
        /* everything is OK */
        return FDC2212_OK;
    }

    if (err_reg != 0) {
        return _error_code(dev, status);
    }

    return FDC2212_OK;
}

static int _is_available(const fdc2212_t *dev)
{
    uint16_t reg_man_id = 0x0000;
    uint16_t reg_dev_id = 0x0000;


    if (_reg_read(dev, FDC2212_REG_MANUFACTURER_ID, (uint8_t *)&reg_man_id, 2) != FDC2212_OK) {
        DEBUG_DEV("could not read FDC2212_REG_MANUFACTURER_ID", dev);
        return -FDC2212_ERROR_I2C;
    }

    if (reg_man_id != FDC2212_MANUFACTURER_ID) {
        DEBUG_DEV("wrong manafacture ID %04x, should be %04x",
                  dev, ((reg_man_id[0] << 8) | reg_man_id[1]), FDC2212_MANUFACTURER_ID);
        return -FDC2212_ERROR_NO_DEV;
    }

    if (_reg_read(dev, FDC2212_REG_DEVICE_ID, (uint8_t *)&reg_dev_id, 2) != FDC2212_OK) {
        DEBUG_DEV("could not read FDC2212_REG_DEVICE_ID", dev);
        return -FDC2212_ERROR_I2C;
    }

    if (reg_dev_id != FDC2212_DEVICE_ID) {
        DEBUG_DEV("wrong manafacture ID %04x, should be %04x",
                  dev, ((reg_dev_id[0] << 8) | reg_dev_id[1]), FDC2212_DEVICE_ID);
        return -FDC2212_ERROR_NO_DEV;
    }

    DEBUG_DEV("manafactured ID:      %04x", dev, reg_man_id);
    DEBUG_DEV("device ID:            %04x", dev, reg_dev_id);

    return _check_error_status(dev);
}

static int _clear_data_rdy(const fdc2212_t *dev)
{
    uint16_t reg_status = 0x0000;

    /* Clearing the DRDY bit by reading STATUS reg */
    if (_reg_read(dev, FDC2212_REG_STATUS, (uint8_t *)&reg_status, 2) != FDC2212_OK) {
        DEBUG_DEV("couldn't read the register FDC2212_REG_STATUS", dev);
        return -FDC2212_ERROR_I2C;
    }

    return FDC2212_OK;
}