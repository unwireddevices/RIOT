/* Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file        sht21.c
 * @brief       driver for SHT21 sensor
 * @author      Eugene P. [ep@unwds.com]
 */


#include "sht21.h"
#include "periph/i2c.h"

#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SHT21 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param SHT21 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int sht21_init(sht21_t *dev)
{
    uint8_t config;

    assert(dev != NULL);

    /* Acquire I2C bus */
    i2c_acquire(dev->i2c);

    if (i2c_init_master(dev->i2c, I2C_SPEED_NORMAL) < 0) {
        i2c_release(dev->i2c);
        puts("[sht21 driver] Error initializing I2C bus");
        
        return -1;
    }
    
    if (i2c_read_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_READ, (char *)&config) != 1) {
        puts("[sht21 driver] Error: sensor not found");
        i2c_release(dev->i2c);

        return -1;
    }

    /* Clean all configuration bits except those reserved */
    config &= SHT21_USER_REG_RESERVED_BITS;

    /* Set configuration bits without changing those reserved */
    config |= SHT21_USER_CONFIG;

    puts("[sht21 driver] Setting configuration register");
    i2c_write_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_WRITE, config);
    i2c_release(dev->i2c);
    
    return 0;
}

static int sht21_check_crc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
    int crc = 0;
    int byteCtr;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) {
        crc ^= (data[byteCtr]);
        for (int bit = 8; bit > 0; --bit) {
            crc = (crc << 1);
            if (crc & 0x80) {
                crc ^= 0x131;
            }
        }
    }
    
    if (crc != checksum) {
        return -1;
    }
    
    return 0;
}

static inline int ticks_to_millicelsius(int ticks)
{
    ticks &= ~0x0003; /* clear status bits */
    /*
     * Formula T = -46.85 + 175.72 * ST / 2^16 from data sheet 6.2,
     * optimized for integer fixed point (3 digits) arithmetic
     */
    return ((21965 * ticks) >> 13) - 46850;
}

static inline int ticks_to_per_cent_mille(int ticks)
{
    ticks &= ~0x0003; /* clear status bits */
    /*
     * Formula RH = -6 + 125 * SRH / 2^16 from data sheet 6.1,
     * optimized for integer fixed point (3 digits) arithmetic
     */
    return ((15625 * ticks) >> 13) - 6000;
}

static int read_sensor_hold(sht21_t *dev, bool need_rh, int *result) {
    uint8_t config;
    /* Write command somehow hangs I2C if there was NACK before */
    /* So lets do read first */
    i2c_read_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_READ, (char *)&config);
    
    /* Choose measurement command: humidity or temperature */
    uint8_t cmd = (need_rh) ? SHT21_REG_RH_HOLD : SHT21_REG_T_HOLD;

    i2c_write_byte(dev->i2c, SHT21_ADDRESS, cmd);

    /* Read back measurements: MSB, LSB and checksum byte */
    
    uint8_t bytes[3];
    i2c_read_bytes(dev->i2c, SHT21_ADDRESS, (char *) bytes, 3);
    
    if (sht21_check_crc(bytes, 2, bytes[2])) {
        return -1;
    }
    
    /* Compose 16 bit integer */
    int data = bytes[0] << 8 | bytes[1];
    *result = (need_rh) ? ticks_to_per_cent_mille(data) : ticks_to_millicelsius(data);
    
    return 0;
}

/**
 * @brief Gets SHT21 measure.
 *
 * @param[in] dev pointer to the initialized SHT21 device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * sht21_measure_t measure;
 *
 * sht21_init(sht21);
 * sht21_measure(sht21, &measure)
 */
uint32_t sht21_measure(sht21_t *dev, sht21_measure_t *measure)
{
    assert(dev != NULL);

    //soft_reset(dev);
    
    int temperature = 0;
    int humidity =  0;
    int result = 0;

    i2c_acquire(dev->i2c);

    result = read_sensor_hold(dev, false, &temperature);
    result += read_sensor_hold(dev, true, &humidity);

    i2c_release(dev->i2c);

    if (result) {
        return -1;
    }
    
    measure->temperature = temperature;
    measure->humidity = humidity;

    return 0;
}


#ifdef __cplusplus
}
#endif
