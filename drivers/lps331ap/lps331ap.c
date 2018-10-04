/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lps331ap
 * @{
 *
 * @file
 * @brief       Device driver implementation for the LPS331AP pressure sensor
 *
 * @note The current driver implementation is very basic and allows only for polling the
 *       devices temperature and pressure values. Threshold values and interrupts are not
 *       used.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 *
 * @}
 */

#include <stdint.h>

#include "periph/i2c.h"
#include "byteorder.h"
#include "lps331ap.h"
#include "lps331ap-internal.h"

/**
 * @brief default I2C bus speed for this sensor
 */
#define BUS_SPEED           I2C_SPEED_NORMAL

/**
 * @brief pressure divider for norming pressure output
 */
#define PRES_DIVIDER        (4096U)

/**
 * @brief temperature base value and divider for norming temperature output
 */
#define TEMP_BASE           (42500L)
#define TEMP_DIVIDER        (480L)


int lps331ap_init(lps331ap_t *dev, i2c_t i2c, uint8_t address, lps331ap_rate_t rate)
{
    uint8_t tmp;

    /* save device specifics */
    dev->i2c = i2c;
    dev->address = address;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->i2c);
    
    /* Initialize I2C bus */
    i2c_init(dev->i2c);

    /* configure device, for simple operation only CTRL_REG1 needs to be touched */
    tmp = LPS331AP_CTRL_REG1_DBDU | LPS331AP_CTRL_REG1_PD |
          (rate << LPS331AP_CTRL_REG1_ODR_POS);
    if (i2c_write_reg(dev->i2c, dev->address, LPS331AP_REG_CTRL_REG1, tmp, 0) < 0) {
        i2c_release(dev->i2c);
        return -1;
    }
    i2c_release(dev->i2c);

    return 0;
}

int lps331ap_read_temp(const lps331ap_t *dev)
{
    int16_t val = 0;

    i2c_acquire(dev->i2c);
    i2c_read_regs(dev->i2c, dev->address, LPS331AP_REG_TEMP_OUT_H, &val, 2, 0);
    i2c_release(dev->i2c);
    
    byteorder_swap((void *)&val, sizeof(val));

    /* return temperature in mC */
    return TEMP_BASE + (1000*(int)val)/TEMP_DIVIDER;;
}

int lps331ap_read_pres(const lps331ap_t *dev)
{
    uint8_t tmp;
    int32_t val = 0;

    i2c_acquire(dev->i2c);
    i2c_read_reg(dev->i2c, dev->address, LPS331AP_REG_PRESS_OUT_XL, &tmp, 0);
    val |= tmp;
    i2c_read_reg(dev->i2c, dev->address, LPS331AP_REG_PRESS_OUT_L, &tmp, 0);
    val |= ((uint32_t)tmp << 8);
    i2c_read_reg(dev->i2c, dev->address, LPS331AP_REG_PRESS_OUT_H, &tmp, 0);
    i2c_release(dev->i2c);
    val |= ((uint32_t)tmp << 16);

    return val / PRES_DIVIDER;
}


int lps331ap_enable(const lps331ap_t *dev)
{
    uint8_t tmp;
    int status;

    i2c_acquire(dev->i2c);
    if (i2c_read_reg(dev->i2c, dev->address, LPS331AP_REG_CTRL_REG1, &tmp, 0) != 1) {
        i2c_release(dev->i2c);
        return -1;
    }
    tmp |= (LPS331AP_CTRL_REG1_PD);
    status = i2c_write_reg(dev->i2c, dev->address, LPS331AP_REG_CTRL_REG1, tmp, 0);
    i2c_release(dev->i2c);

    return status;
}

int lps331ap_disable(const lps331ap_t *dev)
{
    uint8_t tmp;
    int status;

    i2c_acquire(dev->i2c);
    if (i2c_read_reg(dev->i2c, dev->address, LPS331AP_REG_CTRL_REG1, &tmp, 0) != 1) {
        i2c_release(dev->i2c);
        return -1;
    }
    tmp &= ~(LPS331AP_CTRL_REG1_PD);
    status = i2c_write_reg(dev->i2c, dev->address, LPS331AP_REG_CTRL_REG1, tmp, 0);
    i2c_release(dev->i2c);

    return status;
}
