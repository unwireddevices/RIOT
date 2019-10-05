/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "periph/gpio.h"
#include "periph/i2c.h"
#include <mlx90614.h>
#include "xtimer.h"

int mlx90614_init(mlx90614_t *dev)
{
    assert(dev != NULL);
    
    i2c_init(dev->params.i2c);
    
	i2c_acquire(dev->params.i2c);
    
    /* switching to I2C mode by pulling SCL low for 2 ms */
    gpio_init(i2c_config[dev->params.i2c].scl, GPIO_OD);
    gpio_clear(i2c_config[dev->params.i2c].scl);
    xtimer_spin(xtimer_ticks_from_usec(2000));
    gpio_set(i2c_config[dev->params.i2c].scl);
    xtimer_spin(xtimer_ticks_from_usec(2000));
    
    i2c_init(dev->params.i2c);
    
    uint8_t config = 0;
    if (i2c_read_regs(dev->params.i2c, MLX90614_I2C_ADDRESS, MLX90614_REG_ADDR_CONF, &config, 1, 0) != 0) {
        i2c_release(dev->params.i2c);
        return -1;
    }
    
    i2c_release(dev->params.i2c);
    
	return 0;
}

int16_t mlx90614_get_ambient_temperature(mlx90614_t *dev) {
    uint8_t data[3];
    
    i2c_acquire(dev->params.i2c);
    i2c_write_byte(dev->params.i2c, MLX90614_I2C_ADDRESS, MLX90614_RAM_ADDR_AMB, I2C_NOSTOP);
    i2c_read_bytes(dev->params.i2c, MLX90614_I2C_ADDRESS, &data, 3, 0);
    i2c_release(dev->params.i2c);
    
    return (2*(data[0] | data[1] << 8) - 27315);
}

int16_t mlx90614_get_object_temperature(mlx90614_t *dev) {
    uint8_t data[3];
    
    i2c_acquire(dev->params.i2c);
    i2c_write_byte(dev->params.i2c, MLX90614_I2C_ADDRESS, MLX90614_RAM_ADDR_OBJ1, I2C_NOSTOP);
    i2c_read_bytes(dev->params.i2c, MLX90614_I2C_ADDRESS, &data, 3, 0);
    i2c_release(dev->params.i2c);
    
    return (2*(data[0] | data[1] << 8) - 27315);
}

int16_t mlx90614_get_object2_temperature(mlx90614_t *dev) {
    uint8_t data[3];
    
    i2c_acquire(dev->params.i2c);
    i2c_write_byte(dev->params.i2c, MLX90614_I2C_ADDRESS, MLX90614_RAM_ADDR_OBJ2, I2C_NOSTOP);
    i2c_read_bytes(dev->params.i2c, MLX90614_I2C_ADDRESS, &data, 3, 0);
    i2c_release(dev->params.i2c);
    
    return (2*(data[0] | data[1] << 8) - 27315);
}