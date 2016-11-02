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
//#include "i2c.h"

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
 * @return <0 in case of error
 */
int sht21_init(sht21_t * dev)
{
    uint8_t config;

    assert(dev != NULL);

    /* Acquire the I2C bus */
    i2c_acquire(dev->i2c);

    if (i2c_read_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_READ, (char *)&config) != 1) {
      puts("[sht21 driver] The sensor not found. Error.\n");
      return 1;
    }

    /* Clean all the configuration bits except those reserved */
    config &= SHT21_USER_REG_RESERVED_BITS;

    /* Set the configuration bits without changing those reserved */
    config |= SHT21_USER_CONFIG;

    puts("[sht21 driver] Send proper config.\n");
    i2c_write_reg(dev->i2c, SHT21_ADDRESS, SHT21_USER_REG_WRITE, config);

    i2c_release(dev->i2c);


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

    /* Acquire the I2C bus */
    i2c_acquire(dev->i2c);
    uint8_t temp_int[2];
    uint16_t temperature;
    double res_temp = -46.85;
    double res_humid = -6.0;
    uint16_t humidity = 0;
    uint8_t humi_int[2];
    i2c_read_regs(dev->i2c, 0x40, SHT21_REG_T_HOLD, (char *)temp_int, 2);
    temperature = (temp_int[0] << 8) | (temp_int[1] & SHT21_STATUS_MASK);
    res_temp += (175.72 * (double)temperature) / 65536.0;
    temperature = (uint16_t)((res_temp * 100)/2331 + 850);
    //measure->temperature = temperature;
    measure->temperature = temperature;

    i2c_read_regs(dev->i2c, 0x40, SHT21_REG_RH_HOLD, (char *)humi_int, 2);
    humidity = (humi_int[0] << 8) | (humi_int[1] & SHT21_STATUS_MASK);
    res_humid += 125.0 * (double)humidity / 65536.0;
    //humidity = (res_humid * 100)/26214 + 12;
    //measure->humidity = humidity;
    measure->humidity = res_humid;

    i2c_release(dev->i2c);

    return 0;
}


#ifdef __cplusplus
}
#endif
