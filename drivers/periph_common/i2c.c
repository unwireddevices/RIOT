/*
 * Copyright (C) 2018 Mesotic SAS <dylan.laduranty@mesotic.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       common I2C function fallback implementations
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 *
 * @}
 */
#include <errno.h>

#include "board.h"
#include "cpu.h"
#include "xtimer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef I2C_NUMOF

int __attribute__((weak)) i2c_read_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                 void *data, uint8_t flags)
{
    return i2c_read_regs(dev, addr, reg, data, 1, flags);
}

int __attribute__((weak)) i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                  void *data, size_t len, uint8_t flags)
{
    if (flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }
    /* First set ADDR and register with no stop */
    int ret = i2c_write_bytes(dev, addr, &reg, (flags & I2C_REG16) ? 2 : 1,
                              flags | I2C_NOSTOP);
    if (ret < 0) {
        return ret;
    }
    /* Then get the data from device */
    return i2c_read_bytes(dev, addr, data, len, flags);
}

int i2c_read_byte(i2c_t dev, uint16_t addr, void *data, uint8_t flags)
{
    return i2c_read_bytes(dev, addr, data, 1, flags);
}

int i2c_write_byte(i2c_t dev, uint16_t addr, uint8_t data, uint8_t flags)
{
    return i2c_write_bytes(dev, addr, &data, 1, flags);
}

int __attribute__((weak)) i2c_write_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                  uint8_t data, uint8_t flags)
{
    return i2c_write_regs(dev, addr, reg, &data, 1, flags);
}

int __attribute__((weak)) i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                   const void *data, size_t len, uint8_t flags)
{
    if (flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }
    /* First set ADDR and register with no stop */
    int ret = i2c_write_bytes(dev, addr, &reg, (flags & I2C_REG16) ? 2 : 1,
                              flags | I2C_NOSTOP);
    if (ret < 0) {
        return ret;
    }
    /* Then write data to the device */
    return i2c_write_bytes(dev, addr, data, len, flags | I2C_NOSTART);
}

void i2c_unstuck_sda(i2c_t dev) {
    int count = 0;

    if (gpio_read(i2c_config[dev].sda == 0)) {
        do {
            gpio_clear(i2c_config[dev].scl);
            xtimer_spin(xtimer_ticks_from_usec(5));
            gpio_set(i2c_config[dev].scl);
            xtimer_spin(xtimer_ticks_from_usec(5));
            count++;
        } while (gpio_read(i2c_config[dev].sda == 0) && (count < 100));
    }
}

#endif /* I2C_NUMOF */
