/*
 * Copyright (C) 2015 Eistec AB
 *               2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis3dh
 * @{
 *
 * @file
 * @brief       Implementation of LIS3DH SPI driver
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#include <stddef.h>
#include <stdint.h>
#include "periph/gpio.h"
#include "lis3dh.h"

#define ENABLE_DEBUG (1)
#include "debug.h"


#if defined (MODULE_LIS3DH_SPI)
#include "periph/spi.h"

#define SPI_MODE        SPI_MODE_3

#define DEV_SPI         (dev->params.spi)
#define DEV_CS          (dev->params.cs)
#define DEV_CLK         (dev->params.clk)
#define DEV_SCALE       (dev->params.scale)
#elif defined (MODULE_LIS3DH_I2C)
#include "periph/i2c.h"
#define DEV_I2C         (dev->params.i2c)
#define DEV_ADDR        (dev->params.addr)
#define DEV_SCALE       (dev->params.scale)
#endif



static inline int lis3dh_write_bits(const lis3dh_t *dev, const uint8_t reg, const uint8_t mask,  const uint8_t values);
static int lis3dh_write_reg(const lis3dh_t *dev, const uint8_t reg, const uint8_t value);
static int lis3dh_read_regs(const lis3dh_t *dev, const uint8_t reg, uint8_t *buf,  const uint8_t len);


#if defined (MODULE_LIS3DH_SPI)
/**
 * @brief Read sequential registers from the LIS3DH.
 *
 * @param[in]  dev          Device descriptor
 * @param[in]  reg          The source register starting address
 * @param[in]  len          Number of bytes to read
 * @param[out] buf          The values of the source registers will be written
 *                          here
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
static int lis3dh_read_regs(const lis3dh_t *dev, const uint8_t reg, uint8_t *buf, const uint8_t len)
{
    /* Set READ MULTIPLE mode */
    uint8_t addr = (reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK |
                    LIS3DH_SPI_MULTI_MASK;

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr, NULL, buf, (size_t)len);
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    return 0;
}

/**
 * @brief Write a value to an 8 bit register in the LIS3DH.
 *
 * @param[in]  reg          The target register.
 * @param[in]  value        The value to write.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */

static int lis3dh_write_reg(const lis3dh_t *dev, const uint8_t reg, const uint8_t value)
{
    /* Set WRITE SINGLE mode */
    uint8_t addr = ((reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_WRITE_MASK |
                    LIS3DH_SPI_SINGLE_MASK);

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_reg(DEV_SPI, DEV_CS, addr, value);
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    return 0;
}

/**
 * @brief Write (both set and clear) bits of an 8-bit register on the LIS3DH.
 *
 * @param[in]  addr         Register address on the LIS3DH.
 * @param[in]  mask         Bitmask for the bits to modify.
 * @param[in]  values       The values to write to the masked bits.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
static inline int lis3dh_write_bits(const lis3dh_t *dev, const uint8_t reg, const uint8_t mask, const uint8_t values)
{
    uint8_t tmp;

    if (lis3dh_read_regs(dev, reg, 1, &tmp) < 0) {
        /* Communication error */
        return -1;
    }

    tmp &= ~mask;
    tmp |= (values & mask);

    if (lis3dh_write_reg(dev, reg, tmp) < 0) {
        /* Communication error */
        return -1;
    }

    return 0;
}

int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params)
{
    dev->params = *params;

    uint8_t test;

    /* initialize the chip select line */
    if (spi_init_cs(DEV_SPI, DEV_CS) != SPI_OK) {
        DEBUG("[lis3dh] error while initializing CS pin\n");
        return -1;
    }

    /* test connection to the device */
    lis3dh_read_regs(dev, LIS3DH_REG_WHO_AM_I, 1, &test);
    if (test != LIS3DH_WHO_AM_I_RESPONSE) {
        /* chip is not responding correctly */
        DEBUG("[lis3dh] error reading the who am i reg [0x%02x]\n", (int)test);
        return -1;
    }

    /* Clear all settings */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, LIS3DH_CTRL_REG1_XYZEN_MASK);
    /* Disable HP filter */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, 0);
    /* Disable INT1 interrupt sources */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG3, 0);
    /* Set block data update and little endian, set Normal mode (LP=0, HR=1) */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4,
                     (LIS3DH_CTRL_REG4_BDU_ENABLE |
                      LIS3DH_CTRL_REG4_BLE_LITTLE_ENDIAN |
                      LIS3DH_CTRL_REG4_HR_MASK));
    /* Disable FIFO */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, 0);
    /* Reset INT2 settings */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG6, 0);

    /* Configure scale */
    lis3dh_set_scale(dev, DEV_SCALE);

    return 0;
}

int lis3dh_read_xyz(const lis3dh_t *dev, lis3dh_data_t *acc_data)
{
    int i;
    /* Set READ MULTIPLE mode */
    static const uint8_t addr = (LIS3DH_REG_OUT_X_L | LIS3DH_SPI_READ_MASK |
                                 LIS3DH_SPI_MULTI_MASK);
                                 
    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr,
                      NULL, acc_data, sizeof(lis3dh_data_t));
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    /* Scale to milli-G */
    for (i = 0; i < 3; ++i) {
        int32_t tmp = (int32_t)(((int16_t *)acc_data)[i]);
        tmp *= dev->scale;
        tmp /= 32768;
        (((int16_t *)acc_data)[i]) = (int16_t)tmp;
    }

    return 0;
}
#elif defined (MODULE_LIS3DH_I2C)
/**
 * @brief Read sequential registers from the LIS3DH.
 *
 * @param[in]  dev          Device descriptor
 * @param[in]  reg          The source register starting address
 * @param[in]  len          Number of bytes to read
 * @param[out] buf          The values of the source registers will be written
 *                          here
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
static int _read(const lis3dh_t *dev, const uint8_t reg, uint8_t *data, const uint16_t length)
{
    int status = 0x00;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c);
    /* Perform the transaction */
    status = i2c_read_regs(dev->params.i2c, dev->params.addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c);

    return status;
}

/**
 * @brief Write a data to registers in the LIS3DH.
 *
 * @param[in]  reg          The target register.
 * @param[in]  value        The value to write.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
static int _write(const lis3dh_t *dev, const uint8_t reg, const uint8_t *data, uint16_t length)
{
    int ststatus = 0x00;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c);
    /* Perform the transaction */
    status = i2c_write_regs(dev->params.i2c, dev->params.addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c);
    return status;
}
/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param dev [description]
 * @param params [description]
 * 
 * @return [description]
 */

int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params)
{
    dev->params = *params;

    uint8_t dev_id;

    i2c_acquire(dev->params.i2c);

    /* initialize the chip select line */
    i2c_init(dev->params.i2c);


    /* test connection to the device */
    lis3dh_device_id_get(dev, &dev_id);
    if (dev_id != LIS3DH_WHO_AM_I_RESPONSE) {
        /* chip is not responding correctly */
        DEBUG("[lis3dh] error reading the who am i reg [0x%02x]\n", (int)test);
        return -1;
    }

    /* Clear all settings */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, LIS3DH_CTRL_REG1_XYZEN_MASK);
    /* Disable HP filter */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, 0);
    /* Disable INT1 interrupt sources */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG3, 0);
    /* Set block data update and little endian, set Normal mode (LP=0, HR=1) */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4,
                     (LIS3DH_CTRL_REG4_BDU_ENABLE |
                      LIS3DH_CTRL_REG4_BLE_LITTLE_ENDIAN |
                      LIS3DH_CTRL_REG4_HR_MASK));
    /* Disable FIFO */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, 0);
    /* Reset INT2 settings */
    lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG6, 0);

    /* Configure scale */
    lis3dh_set_scale(dev, DEV_SCALE);

    return 0;
}

int lis3dh_read_xyz(const lis3dh_t *dev, lis3dh_data_t *acc_data)
{
    int i;
    /* Set READ MULTIPLE mode */
    static const uint8_t addr = (LIS3DH_REG_OUT_X_L|0x80);
                                 
    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c);
    /* Perform the transaction */

    i2c_read_regs(dev->params.i2c, dev->params.addr, (uint16_t)addr, acc_data, sizeof(lis3dh_data_t), 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c);

    /* Scale to milli-G */
    for (i = 0; i < 3; ++i) {
        int32_t tmp = (int32_t)(((int16_t *)acc_data)[i]);
        tmp *= dev->scale;
        tmp /= 32768;
        (((int16_t *)acc_data)[i]) = (int16_t)tmp;
    }

    return 0;
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @defgroup  LIS3DH_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_read_reg(lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int32_t ret;
    ret = _read(dev, reg, data, len);
    return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_write_reg(lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int32_t ret;
    ret = _write(dev, reg, data, len);
    return ret;
}

/**
  * @}
  *
  */


/**
  * @defgroup  LIS3DH_Data_generation
  * @brief     This section group all the functions concerning data generation.
  * @{
  *
  */

/**
  * @brief  Temperature status register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_status_reg_get(lis3dh_ctx_t *ctx, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(ctx, LIS3DH_REG_STATUS_AUX, buff, 1);
    return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tda in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_data_ready_get(lis3dh_t *dev, uint8_t *val)
{
    uint8_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, &status_reg_aux, 1);
    *val = (status_reg_aux & 0x04); //FIXME: Insert Defines in lis3dh-internal

    return ret;
}
/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tor in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_data_ovr_get(lis3dh_t *dev, uint8_t *val)
{
    uint8_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, (uint8_t*)&status_reg_aux, 1);
    *val =  (status_reg_aux & 0x40); //FIXME: Insert Defines in lis3dh-internalstatus_reg_aux._3or;

    return ret;
}
/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temperature_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_AUX_ADC3_L, buff, 2);
    return ret;
}

/**
  * @brief  ADC output value.[get]
  *         Sample frequency: the same as the ODR CTRL_REG1
  *         The resolution:
  *                    10bit if LPen bit in CTRL_REG1 (20h) is clear
  *                     8bit if LPen bit in CTRL_REG1 (20h) is set
  *         Data Format:
  *                     Outputs are Left Justified in 2’ complements
  *                     range 800mV
  *                     code zero means an analogue value of about 1.2V
  *                     Voltage values smaller than centre values are positive
  *                           (Example:  800mV = 7Fh / 127 dec)
  *                     Voltage values bigger than centre values are negative
  *                           (Example: 1600mV = 80h / -128 dec)
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_adc_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_OUT_ADC1_L, buff, 6);
    return ret;
}

/**
  * @brief  Auxiliary ADC.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      configure the auxiliary ADC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_aux_adc_set(lis3dh_t *dev, uint8_t value)
{
    uint8_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TEMP_CFG_REG, &temp_cfg_reg, 1);
    if (ret == 0) {
        if (value != LIS3DH_AUX_DISABLE) {
            /* Required in order to use auxiliary adc */
            ret = lis3dh_block_data_update_set(dev, PROPERTY_ENABLE);
        }
    } 
    if (ret == 0) {
        temp_cfg_reg &= ~(LIS3DH_TEMP_CFG_REG_ADC_PD_MASK | LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK);
        if (val & LIS3DH_AUX_ON_TEMPERATURE) {
            temp_cfg_reg |= (LIS3DH_TEMP_CFG_REG_ADC_PD_MASK | LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK);
        } else (val & LIS3DH_AUX_ON_PADS) {
            temp_cfg_reg |= LIS3DH_TEMP_CFG_REG_ADC_PD_MASK;
        }
        ret = lis3dh_write_reg(ctx, LIS3DH_REG_TEMP_CFG_REG, &temp_cfg_reg, 1);
    }
    return ret;
}

/**
  * @brief  Auxiliary ADC.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      configure the auxiliary ADC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_aux_adc_get(lis3dh_t *dev, uint8_t *val)
{
    uint8_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_TEMP_CFG_REG, &temp_cfg_reg, 1);
    if ((temp_cfg_reg & LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK) && (temp_cfg_reg & LIS3DH_TEMP_CFG_REG_ADC_PD_MASK)) {
        *val = LIS3DH_AUX_ON_TEMPERATURE;
    } 
    if (!(temp_cfg_reg & LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK) && (temp_cfg_reg & LIS3DH_TEMP_CFG_REG_ADC_PD_MASK)) {
        *val = LIS3DH_AUX_ON_PADS;
    } else {
        *val = LIS3DH_AUX_DISABLE;
    }
    return ret;
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  *                  and HR in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_set(lis3dh_ctx_t *ctx, lis3dh_op_md_t val)
{
  uint8_t ctrl_reg1;
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG1,
                          (uint8_t*)&ctrl_reg1, 1);
  if (ret == 0) {
    ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4,
                            (uint8_t*)&ctrl_reg4, 1);
  } 
  if (ret == 0) {
    if ( val == LIS3DH_HR_12bit ) {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 1;
    }
    if (val == LIS3DH_NM_10bit) {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 0;
    }
    if (val == LIS3DH_LP_8bit) {
      ctrl_reg1.lpen = 1;
      ctrl_reg4.hr   = 0;
    }
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  } 
  if (ret == 0) {
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_get(lis3dh_ctx_t *ctx, lis3dh_op_md_t *val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if (ret == 0) {
    ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if ( ctrl_reg1.lpen == PROPERTY_ENABLE ) {
      *val = LIS3DH_LP_8bit;
    } 
    if (ctrl_reg4.hr == PROPERTY_ENABLE ) {
      *val = LIS3DH_HR_12bit;
    } else {
      *val = LIS3DH_NM_10bit;
    }
  }
  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_set(lis3dh_ctx_t *ctx, lis3dh_odr_t val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if (ret == 0) {
    ctrl_reg1.odr = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_get(lis3dh_ctx_t *ctx, lis3dh_odr_t *val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.odr) {
    case LIS3DH_POWER_DOWN:
      *val = LIS3DH_POWER_DOWN;
      break;
    case LIS3DH_ODR_1Hz:
      *val = LIS3DH_ODR_1Hz;
      break;
    case LIS3DH_ODR_10Hz:
      *val = LIS3DH_ODR_10Hz;
      break;
    case LIS3DH_ODR_25Hz:
      *val = LIS3DH_ODR_25Hz;
      break;
    case LIS3DH_ODR_50Hz:
      *val = LIS3DH_ODR_50Hz;
      break;
    case LIS3DH_ODR_100Hz:
      *val = LIS3DH_ODR_100Hz;
      break;
    case LIS3DH_ODR_200Hz:
      *val = LIS3DH_ODR_200Hz;
      break;
    case LIS3DH_ODR_400Hz:
      *val = LIS3DH_ODR_400Hz;
      break;
    case LIS3DH_ODR_1kHz620_LP:
      *val = LIS3DH_ODR_1kHz620_LP;
      break;
    case LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP:
      *val = LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP;
      break;
    default:
      *val = LIS3DH_POWER_DOWN;
      break;
  }
  return ret;
}

/**
  * @brief   High pass data from internal filter sent to output register
  *          and FIFO.
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_on_outputs_set(lis3dh_ctx_t *ctx, uint8_t val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if (ret == 0) {
    ctrl_reg2.fds = val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief   High pass data from internal filter sent to output register
  *          and FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_on_outputs_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = (uint8_t)ctrl_reg2.fds;

  return ret;
}

/**
  * @brief   High-pass filter cutoff frequency selection.[set]
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_bandwidth_set(lis3dh_ctx_t *ctx,
                                         lis3dh_hpcf_t val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if (ret == 0) {
    ctrl_reg2.hpcf = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief   High-pass filter cutoff frequency selection.[get]
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_bandwidth_get(lis3dh_ctx_t *ctx,
                                         lis3dh_hpcf_t *val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  switch (ctrl_reg2.hpcf) {
    case LIS3DH_AGGRESSIVE:
      *val = LIS3DH_AGGRESSIVE;
      break;
    case LIS3DH_STRONG:
      *val = LIS3DH_STRONG;
      break;
    case LIS3DH_MEDIUM:
      *val = LIS3DH_MEDIUM;
      break;
    case LIS3DH_LIGHT:
      *val = LIS3DH_LIGHT;
      break;
    default:
      *val = LIS3DH_LIGHT;
      break;
  }
  return ret;
}

/**
  * @brief  High-pass filter mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_mode_set(lis3dh_ctx_t *ctx, lis3dh_hpm_t val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if (ret == 0) {
    ctrl_reg2.hpm = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  High-pass filter mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_mode_get(lis3dh_ctx_t *ctx, lis3dh_hpm_t *val)
{
  lis3dh_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  switch (ctrl_reg2.hpm) {
    case LIS3DH_NORMAL_WITH_RST:
      *val = LIS3DH_NORMAL_WITH_RST;
      break;
    case LIS3DH_REFERENCE_MODE:
      *val = LIS3DH_REFERENCE_MODE;
      break;
    case LIS3DH_NORMAL:
      *val = LIS3DH_NORMAL;
      break;
    case LIS3DH_AUTORST_ON_INT:
      *val = LIS3DH_AUTORST_ON_INT;
      break;
    default:
      *val = LIS3DH_NORMAL_WITH_RST;
      break;
  }
  return ret;
}

/**
  * @brief  Full-scale configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_set(lis3dh_ctx_t *ctx, lis3dh_fs_t val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.fs = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Full-scale configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_get(lis3dh_ctx_t *ctx, lis3dh_fs_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  switch (ctrl_reg4.fs) {
    case LIS3DH_2g:
      *val = LIS3DH_2g;
      break;
    case LIS3DH_4g:
      *val = LIS3DH_4g;
      break;
    case LIS3DH_8g:
      *val = LIS3DH_8g;
      break;
    case LIS3DH_16g:
      *val = LIS3DH_16g;
      break;
    default:
      *val = LIS3DH_2g;
      break;
  }
  return ret;
}

/**
  * @brief  Block Data Update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_set(lis3dh_ctx_t *ctx, uint8_t val)
{
    uint8_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(ctx, LIS3DH_REG_CTRL_REG4, &ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4 &= ~LIS3DH_CTRL_REG4_BDU_MASK;
        ctrl_reg4 |= (val << LIS3DH_CTRL_REG4_BDU_SHIFT);
        ret = lis3dh_write_reg(ctx, LIS3DH_REG_CTRL_REG4, &ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Block Data Update.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  *val = (uint8_t)ctrl_reg4.bdu;

  return ret;
}

/**
  * @brief  Reference value for interrupt generation.[set]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_filter_reference_set(lis3dh_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis3dh_write_reg(ctx, LIS3DH_REFERENCE, buff, 1);
  return ret;
}

/**
  * @brief  Reference value for interrupt generation.[get]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_filter_reference_get(lis3dh_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_REFERENCE, buff, 1);
  return ret;
}
/**
  * @brief  Acceleration set of data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxda in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ready_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_status_reg_t status_reg;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_STATUS_REG, (uint8_t*)&status_reg, 1);
  *val = status_reg.zyxda;

  return ret;
}
/**
  * @brief  Acceleration set of data overrun.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxor in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ovr_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_status_reg_t status_reg;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_STATUS_REG, (uint8_t*)&status_reg, 1);
  *val = status_reg.zyxor;

  return ret;
}
/**
  * @brief  Acceleration output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_acceleration_raw_get(lis3dh_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_OUT_X_L, buff, 6);
  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Common
  * @brief     This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI .[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_device_id_get(lis3dh_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_WHO_AM_I, buff, 1);
  return ret;
}
/**
  * @brief  Self Test.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_self_test_set(lis3dh_ctx_t *ctx, lis3dh_st_t val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.st = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Self Test.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_self_test_get(lis3dh_ctx_t *ctx, lis3dh_st_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  switch (ctrl_reg4.st) {
    case LIS3DH_ST_DISABLE:
      *val = LIS3DH_ST_DISABLE;
      break;
    case LIS3DH_ST_POSITIVE:
      *val = LIS3DH_ST_POSITIVE;
      break;
    case LIS3DH_ST_NEGATIVE:
      *val = LIS3DH_ST_NEGATIVE;
      break;
    default:
      *val = LIS3DH_ST_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_format_set(lis3dh_ctx_t *ctx, lis3dh_ble_t val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.ble = (uint8_t)val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_format_get(lis3dh_ctx_t *ctx, lis3dh_ble_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  switch (ctrl_reg4.ble) {
    case LIS3DH_LSB_AT_LOW_ADD:
      *val = LIS3DH_LSB_AT_LOW_ADD;
      break;
    case LIS3DH_MSB_AT_LOW_ADD:
      *val = LIS3DH_MSB_AT_LOW_ADD;
      break;
    default:
      *val = LIS3DH_LSB_AT_LOW_ADD;
      break;
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_boot_set(lis3dh_ctx_t *ctx, uint8_t val)
{
  lis3dh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  if (ret == 0) {
    ctrl_reg5.boot = val;
    ret = lis3dh_write_reg(ctx, LIS3DH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_boot_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  *val = (uint8_t)ctrl_reg5.boot;

  return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_status_get(lis3dh_ctx_t *ctx, lis3dh_status_reg_t *val)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_STATUS_REG, (uint8_t*) val, 1);
  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup   LIS3DH_Interrupts_generator_1
  * @brief      This section group all the functions that manage the first
  *             interrupts generator
  * @{
  *
  */

/**
  * @brief  Interrupt generator 1 configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_set(lis3dh_ctx_t *ctx,
                                   lis3dh_int1_cfg_t *val)
{
  int32_t ret;
  ret = lis3dh_write_reg(ctx, LIS3DH_INT1_CFG, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Interrupt generator 1 configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_get(lis3dh_ctx_t *ctx,
                                   lis3dh_int1_cfg_t *val)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_CFG, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Interrupt generator 1 source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Registers INT1_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_source_get(lis3dh_ctx_t *ctx,
                                     lis3dh_int1_src_t *val)
{
  int32_t ret;
  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_SRC, (uint8_t*) val, 1);
  return ret;
}
/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[set]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_set(lis3dh_ctx_t *ctx, uint8_t val)
{
  lis3dh_int1_ths_t int1_ths;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_THS, (uint8_t*)&int1_ths, 1);
  if (ret == 0) {
    int1_ths.ths = val;
    ret = lis3dh_write_reg(ctx, LIS3DH_INT1_THS, (uint8_t*)&int1_ths, 1);
  }
  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_int1_ths_t int1_ths;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_THS, (uint8_t*)&int1_ths, 1);
  *val = (uint8_t)int1_ths.ths;

  return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_set(lis3dh_ctx_t *ctx, uint8_t val)
{
  lis3dh_int1_duration_t int1_duration;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_DURATION, (uint8_t*)&int1_duration, 1);
  if (ret == 0) {
    int1_duration.d = val;
    ret = lis3dh_write_reg(ctx, LIS3DH_INT1_DURATION, (uint8_t*)&int1_duration, 1);
  }
  return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_get(lis3dh_ctx_t *ctx, uint8_t *val)
{
  lis3dh_int1_duration_t int1_duration;
  int32_t ret;

  ret = lis3dh_read_reg(ctx, LIS3DH_INT1_DURATION, (uint8_t*)&int1_duration, 1);
  *val = (uint8_t)int1_duration.d;

  return ret;
}

/**
  * @}
  *
  */





/**
  * @brief  DeviceWhoamI .[get]
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_device_id_get(const lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_regs(dev, LIS3DH_REG_WHO_AM_I, buff, 1);
    return ret;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int lis3dh_read_aux_adc1(const lis3dh_t *dev, int16_t *out)
{
    return lis3dh_read_regs(dev, LIS3DH_REG_OUT_AUX_ADC1_L,
                            LIS3DH_ADC_DATA_SIZE, (uint8_t *)out);
}

int lis3dh_read_aux_adc2(const lis3dh_t *dev, int16_t *out)
{
    return lis3dh_read_regs(dev, LIS3DH_REG_OUT_AUX_ADC2_L,
                            LIS3DH_ADC_DATA_SIZE, (uint8_t *)out);
}

int lis3dh_read_aux_adc3(const lis3dh_t *dev, int16_t *out)
{
    return lis3dh_read_regs(dev, LIS3DH_REG_OUT_AUX_ADC3_L,
                            LIS3DH_ADC_DATA_SIZE, (uint8_t *)out);
}

int lis3dh_set_aux_adc(const lis3dh_t *dev, const uint8_t enable,
                       const uint8_t temperature)
{
    return lis3dh_write_bits(dev, LIS3DH_REG_TEMP_CFG_REG,
                             LIS3DH_TEMP_CFG_REG_ADC_PD_MASK,
                             (enable ? LIS3DH_TEMP_CFG_REG_ADC_PD_MASK : 0) |
                             (temperature ? LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK : 0));
}

int lis3dh_set_axes(const lis3dh_t *dev, const uint8_t axes)
{
    return lis3dh_write_bits(dev, LIS3DH_REG_CTRL_REG1,
                             LIS3DH_CTRL_REG1_XYZEN_MASK, axes);
}

int lis3dh_set_fifo(const lis3dh_t *dev, const uint8_t mode, const uint8_t watermark)
{
    int status;
    uint8_t reg;
    reg = (watermark << LIS3DH_FIFO_CTRL_REG_FTH_SHIFT)
            & LIS3DH_FIFO_CTRL_REG_FTH_MASK;
    reg |= mode;
    status = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, reg);
    if (status < 0) {
        /* communication error */
        return status;
    }
    if (mode != 0x00) {
        status = lis3dh_write_bits(dev, LIS3DH_REG_CTRL_REG5,
            LIS3DH_CTRL_REG5_FIFO_EN_MASK, LIS3DH_CTRL_REG5_FIFO_EN_MASK);
    } else {
        status = lis3dh_write_bits(dev, LIS3DH_REG_CTRL_REG5,
            LIS3DH_CTRL_REG5_FIFO_EN_MASK, 0);
    }
    return status;
}

int lis3dh_set_odr(const lis3dh_t *dev, const uint8_t odr)
{
    return lis3dh_write_bits(dev, LIS3DH_REG_CTRL_REG1,
        LIS3DH_CTRL_REG1_ODR_MASK, odr);
}

int lis3dh_set_scale(lis3dh_t *dev, const uint8_t scale)
{
    uint8_t scale_reg;
    /* Sensor full range is -32768 -- +32767 (measurements are left adjusted) */
    /*  => Scale factor is scale/32768 */
    switch (scale)
    {
        case 2:
            dev->scale = 2000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_2G;
            break;
        case 4:
            dev->scale = 4000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_4G;
            break;
        case 8:
            dev->scale = 8000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_8G;
            break;
        case 16:
            dev->scale = 16000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_16G;
            break;
        default:
            return -1;
    }
    return lis3dh_write_bits(dev, LIS3DH_REG_CTRL_REG4,
                             LIS3DH_CTRL_REG4_FS_MASK, scale_reg);
}

int lis3dh_set_int1(const lis3dh_t *dev, const uint8_t mode)
{
    return lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG3, mode);
}

int lis3dh_get_fifo_level(const lis3dh_t *dev)
{
    uint8_t reg;
    int level;

    if (lis3dh_read_regs(dev, LIS3DH_REG_FIFO_SRC_REG, 1, &reg) != 0) {
        return -1;
    }
    level = (reg & LIS3DH_FIFO_SRC_REG_FSS_MASK) >> LIS3DH_FIFO_SRC_REG_FSS_SHIFT;
    return level;
}

/** @} */
