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
 * @authoh		Eugene P. [ep@unwds.com]
 */


#include "sht21.h"
#include "periph/i2c.h"

#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/
/**
 *  * \name SHT21 register addresses and values
 *   * @{
 *    */
#define SHT21_USER_REG_READ             (0xE7)
#define SHT21_USER_REG_WRITE            (0xE6)
#define SHT21_USER_REG_RESERVED_BITS    (0x38)

#define SHT21_TEMPERATURE_HM_CMD        (0xE3)
#define SHT21_HUMIDITY_HM_CMD           (0xE5)
#define SHT21_TEMPERATURE_NHM_CMD       (0xF3)
#define SHT21_HUMIDITY_NHM_CMD          (0xF5)
#define SHT21_RESET_CMD                 (0xFE)

#define SHT21_STATUS_MASK               (0xFC)

#define SHT21_RESOLUTION_12b_14b        ((0 << 7) | (0 << 0))
#define SHT21_RESOLUTION_8b_12b         ((0 << 7) | (1 << 0))
#define SHT21_RESOLUTION_10b_13b        ((1 << 7) | (0 << 0))
#define SHT21_RESOLUTION_11b_11b        ((1 << 7) | (1 << 0))
#define SHT21_BATTERY_ABOVE_2V25        (0 << 6)
#define SHT21_BATTERY_BELOW_2V25        (1 << 6)
#define SHT21_ONCHIP_HEATER_ENABLE      (1 << 2)
#define SHT21_ONCHIP_HEATER_DISABLE     (0 << 2)
#define SHT21_OTP_RELOAD_ENABLE         (0 << 1)
#define SHT21_OTP_RELOAD_DISABLE        (1 << 1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 *  * \name SHT21 configuration values
 *   * @{
 *    */
#define SHT21_DEFAULT_CONFIG            (SHT21_RESOLUTION_12b_14b | \
                                             SHT21_ONCHIP_HEATER_DISABLE | \
                                             SHT21_BATTERY_ABOVE_2V25 | \
                                             SHT21_OTP_RELOAD_DISABLE)

#define SHT21_USER_CONFIG               (SHT21_RESOLUTION_8b_12b | \
                                             SHT21_ONCHIP_HEATER_DISABLE | \
                                             SHT21_BATTERY_ABOVE_2V25 | \
                                             SHT21_OTP_RELOAD_DISABLE)
/** @} */
/*---------------------------------------------------------------------------*/

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
int sht21_init(sht21_t * dev)
{
    uint8_t config;

    assert(dev != NULL);

    /* Acquire I2C bus */
    i2c_acquire(dev->i2c);

    i2c_init_master(dev->i2c, I2C_SPEED_NORMAL);

    if (i2c_read_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_READ, (char *)&config) != 1) {
      puts("[sht21 driver] Error: sensor not found.\n");
      i2c_release(dev->i2c);

      return -1;
    }

    /* Clean all configuration bits except those reserved */
    config &= SHT21_USER_REG_RESERVED_BITS;

    /* Set configuration bits without changing those reserved */
    config |= SHT21_USER_CONFIG;

    puts("[sht21 driver] Setting configuration register.\n");
    i2c_write_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_WRITE, config);
    i2c_release(dev->i2c);

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

static uint16_t read_sensor_no_hold(sht21_t *dev, bool need_rh) {
	/* Choose measurement command: humidity or temperature */
	uint8_t cmd = (need_rh) ? SHT21_REG_RH_NACK : SHT21_REG_T_NACK;

	/* Send T/RH measurement command without SCK holding */
	volatile long i;
	for(i = 0; i < 100000; i++);

	i2c_write_byte(dev->i2c, SHT21_ADDRESS, cmd);

	/* Sleep long enough (250 ms) */
	/* TODO: polling with tracking of NACK's */
	for(i = 0; i < 100000; i++);

	/* Read back measurements: MSB, LSB and checksum byte */
	uint8_t bytes[3];
	i2c_read_bytes(dev->i2c, SHT21_ADDRESS, (char *) bytes, 3);

	/* Compose 16 bit integer */
	return bytes[0] << 8 | bytes[1];
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

    i2c_acquire(dev->i2c);

    int temperature = ticks_to_millicelsius(read_sensor_no_hold(dev, false));
    int humidity = ticks_to_per_cent_mille(read_sensor_no_hold(dev, true));

    i2c_release(dev->i2c);

    measure->temperature = temperature;
    measure->humidity = humidity;

    return 0;
}


#ifdef __cplusplus
}
#endif
