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
 * @file		sht21.c
 * @brief       driver for SHT21 sensor
 * @author      SmallSharky
 */


#include "sht21.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
	uint16_t a;
	uint8_t b1, b2;
} endian_ut;

static uint16_t endian_conversion(uint16_t input){
	uint8_t * adr = (uint8_t *)&input;
	uint8_t tmp = adr[0];
	adr[0] = adr[1];
	adr[1] = tmp;
	return *(uint16_t *)adr;
}


/**
 * @brief SHT21 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param SHT21 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int sht21_init(sht21_t * dev)
{

    assert(dev != NULL);



    return 0;
}

/**
 * @brief Get SHT21 measure
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
uint32_t sht21_measure(sht21_t * dev, sht21_measure_t *measure)
{
    assert(dev != NULL);
    /*init i2c*/
    i2c_init_master(dev->i2c, I2C_SPEED_NORMAL);

    /* Acquire the I2C bus */
    i2c_acquire(dev->i2c);
    uint16_t temp_int;
    uint16_t humi_int;
    i2c_read_regs(dev->i2c, 0x40, SHT21_REG_T_HOLD, (char *)&temp_int, 2);
    i2c_read_regs(dev->i2c, 0x40, SHT21_REG_RH_HOLD, (char *)&humi_int, 2);

    uint32_t temp_tmp = (uint32_t)endian_conversion(temp_int & 0xFCFF),
    humi_tmp = (uint32_t)endian_conversion(humi_int & 0xF0FF);
    temp_tmp = (temp_tmp * 100)/2331 + 850;
    humi_tmp = (humi_tmp * 100)/26214 + 12;


    measure->humidity = (uint16_t)humi_tmp;
    measure->temperature = (uint16_t)temp_tmp;

    i2c_release(dev->i2c);

    return 0;
}


#ifdef __cplusplus
}
#endif
