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
#include <string.h>

#include "periph/gpio.h"

#include "lis3dh.h"
#include "include/lis3dh_internal.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

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

/**
 * @brief Write a data to registers in the LIS3DH.
 *
 * @param[in] dev     Device descriptor
 * @param[in] reg     The source register starting address
 * @param[in] data    The values of the source registers will be written here
 * @param[in] length  Number of bytes to write
 * 
 * @return            0 on success
 * @return           -1 on error
 */
static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief Read sequential registers from the LIS3DH.
 *
 * @param[in]  dev     Device descriptor
 * @param[in]  reg     The source register starting address
 * @param[out] data    The values of the source registers will be written here
 * @param[in]  length  Number of bytes to read
 *
 * @return             0 on success
 * @return            -1 on error
 */
static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length);


static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params);


int32_t lis3dh_temp_status_reg_get(lis3dh_t *dev, uint8_t *buff);
int32_t lis3dh_temp_data_ready_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_temp_data_ovr_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_temperature_raw_get(lis3dh_t *dev, uint8_t *buff);

int32_t lis3dh_adc_raw_get(lis3dh_t *dev, uint8_t *buff);

int32_t lis3dh_aux_adc_set(lis3dh_t *dev, lis3dh_temp_en_t val);
int32_t lis3dh_aux_adc_get(lis3dh_t *dev, lis3dh_temp_en_t *val);

int32_t lis3dh_operating_mode_set(lis3dh_t *dev, lis3dh_op_md_t val);
int32_t lis3dh_operating_mode_get(lis3dh_t *dev, lis3dh_op_md_t *val);

int32_t lis3dh_data_rate_set(lis3dh_t *dev, lis3dh_odr_t val);
int32_t lis3dh_data_rate_get(lis3dh_t *dev, lis3dh_odr_t *val);

int32_t lis3dh_high_pass_on_outputs_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_high_pass_on_outputs_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_high_pass_bandwidth_set(lis3dh_t *dev, lis3dh_hpcf_t val);
int32_t lis3dh_high_pass_bandwidth_get(lis3dh_t *dev, lis3dh_hpcf_t *val);

int32_t lis3dh_high_pass_mode_set(lis3dh_t *dev, lis3dh_hpm_t val);
int32_t lis3dh_high_pass_mode_get(lis3dh_t *dev, lis3dh_hpm_t *val);

int32_t lis3dh_full_scale_set(lis3dh_t *dev, lis3dh_fs_t val);
int32_t lis3dh_full_scale_get(lis3dh_t *dev, lis3dh_fs_t *val);

int32_t lis3dh_block_data_update_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_block_data_update_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_filter_reference_set(lis3dh_t *dev, uint8_t *buff);
int32_t lis3dh_filter_reference_get(lis3dh_t *dev, uint8_t *buff);

int32_t lis3dh_xl_data_ready_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_xl_data_ovr_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_acceleration_raw_get(lis3dh_t *dev, uint8_t *buff);

int32_t lis3dh_device_id_get(lis3dh_t *dev, uint8_t *buff);

int32_t lis3dh_self_test_set(lis3dh_t *dev, lis3dh_st_t val);
int32_t lis3dh_self_test_get(lis3dh_t *dev, lis3dh_st_t *val);

int32_t lis3dh_data_format_set(lis3dh_t *dev, lis3dh_ble_t val);
int32_t lis3dh_data_format_get(lis3dh_t *dev, lis3dh_ble_t *val);

int32_t lis3dh_boot_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_boot_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_status_get(lis3dh_t *dev, lis3dh_status_reg_t *val);

int32_t lis3dh_int1_gen_conf_set(lis3dh_t *dev, lis3dh_int1_cfg_t *val);
int32_t lis3dh_int1_gen_conf_get(lis3dh_t *dev, lis3dh_int1_cfg_t *val);

int32_t lis3dh_int1_gen_source_get(lis3dh_t *dev, lis3dh_int1_src_t *val);

int32_t lis3dh_int1_gen_threshold_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int1_gen_threshold_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_int1_gen_duration_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int1_gen_duration_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_int2_gen_conf_set(lis3dh_t *dev, lis3dh_int2_cfg_t *val);
int32_t lis3dh_int2_gen_conf_get(lis3dh_t *dev, lis3dh_int2_cfg_t *val);

int32_t lis3dh_int2_gen_source_get(lis3dh_t *dev, lis3dh_int2_src_t *val);

int32_t lis3dh_int2_gen_threshold_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int2_gen_threshold_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_int2_gen_duration_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int2_gen_duration_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_high_pass_int_conf_set(lis3dh_t *dev, lis3dh_hp_t val);
int32_t lis3dh_high_pass_int_conf_get(lis3dh_t *dev, lis3dh_hp_t *val);

int32_t lis3dh_pin_int1_config_set(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val);
int32_t lis3dh_pin_int1_config_get(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val);

int32_t lis3dh_int2_pin_detect_4d_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int2_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_int2_pin_notification_mode_set(lis3dh_t *dev, lis3dh_lir_int2_t val);
int32_t lis3dh_int2_pin_notification_mode_get(lis3dh_t *dev, lis3dh_lir_int2_t *val);

int32_t lis3dh_int1_pin_detect_4d_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_int1_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_int1_pin_notification_mode_set(lis3dh_t *dev, lis3dh_lir_int1_t val);
int32_t lis3dh_int1_pin_notification_mode_get(lis3dh_t *dev, lis3dh_lir_int1_t *val);

int32_t lis3dh_pin_int2_config_set(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val);
int32_t lis3dh_pin_int2_config_get(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val);

int32_t lis3dh_fifo_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_fifo_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_fifo_watermark_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_fifo_watermark_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_fifo_trigger_event_set(lis3dh_t *dev, lis3dh_tr_t val);
int32_t lis3dh_fifo_trigger_event_get(lis3dh_t *dev, lis3dh_tr_t *val);

int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val);
int32_t lis3dh_fifo_mode_get(lis3dh_t *dev, lis3dh_fm_t *val);

int32_t lis3dh_fifo_status_get(lis3dh_t *dev, lis3dh_fifo_src_reg_t *val);

int32_t lis3dh_fifo_data_level_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_fifo_empty_flag_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_fifo_ovr_flag_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_fifo_fth_flag_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_tap_conf_set(lis3dh_t *dev, lis3dh_click_cfg_t *val);
int32_t lis3dh_tap_conf_get(lis3dh_t *dev, lis3dh_click_cfg_t *val);

int32_t lis3dh_tap_source_get(lis3dh_t *dev, lis3dh_click_src_t *val);

int32_t lis3dh_tap_threshold_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_tap_threshold_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_tap_notification_mode_set(lis3dh_t *dev, lis3dh_lir_click_t val);
int32_t lis3dh_tap_notification_mode_get(lis3dh_t *dev, lis3dh_lir_click_t *val);

int32_t lis3dh_shock_dur_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_shock_dur_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_quiet_dur_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_quiet_dur_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_double_tap_timeout_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_double_tap_timeout_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_act_threshold_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_act_threshold_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_act_timeout_set(lis3dh_t *dev, uint8_t val);
int32_t lis3dh_act_timeout_get(lis3dh_t *dev, uint8_t *val);

int32_t lis3dh_pin_sdo_sa0_mode_set(lis3dh_t *dev, lis3dh_sdo_pu_disc_t val);
int32_t lis3dh_pin_sdo_sa0_mode_get(lis3dh_t *dev, lis3dh_sdo_pu_disc_t *val);

int32_t lis3dh_spi_mode_set(lis3dh_t *dev, lis3dh_sim_t val);
int32_t lis3dh_spi_mode_get(lis3dh_t *dev, lis3dh_sim_t *val);

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
static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    /* Set READ MULTIPLE mode */
    uint8_t addr = (reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK |
                   LIS3DH_SPI_MULTI_MASK;

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr, NULL, data, (size_t)length);
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

static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    /* Set WRITE SINGLE mode */
    uint8_t addr = ((reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_WRITE_MASK |
                    LIS3DH_SPI_SINGLE_MASK);

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr, data, NULL, (size_t)length);
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    return 0;
}

static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params)
{
    dev->params = *params;

    uint8_t test;

    /* initialize the chip select line */
    if (spi_init_cs(DEV_SPI, DEV_CS) != SPI_OK) {
        DEBUG("[lis3dh] error while initializing CS pin\n");
        return -1;
    }

    return 0;
}

#elif defined (MODULE_LIS3DH_I2C)

static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Write multiple command */
    reg |= 0x80;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c);
    /* Perform the transaction */
    status = i2c_read_regs(dev->params.i2c, dev->params.addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c);

    DEBUG("LIS3DH [REG %02X]: <- ", (reg&0x7F));
    PRINTBUFF(data, length);

    return status;
}

static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Read multiple command */
    reg |= 0x80;

    DEBUG("LIS3DH [REG %02X]: -> %02Xh ", (reg&0x7F), reg);
    PRINTBUFF(data, length);

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c);
    /* Perform the transaction */
    status = i2c_write_regs(dev->params.i2c, dev->params.addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c);
    return status;
}

static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params) {
    dev->params = *params;

    i2c_acquire(dev->params.i2c);

    /* initialize the chip select line */
    i2c_init(dev->params.i2c);

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
int32_t lis3dh_read_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
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
int32_t lis3dh_write_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    int32_t ret;
    ret = _write(dev, reg, data, len);
    return ret;
}
/** @} */



/**
  * @defgroup  LIS3DH_Data_generation
  * @brief     This section group all the functions concerning data generation.
  * @{
  *
  */

/**
  * @brief  Temperature status register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_status_reg_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, buff, 1);
    return ret;
}
/**
  * @brief  Temperature data available.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tda in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_data_ready_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_aux_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, (uint8_t*)&status_reg_aux, 1);
    *val = status_reg_aux._3da;

    return ret;
}
/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tor in reg STATUS_REG_AUX
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temp_data_ovr_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_aux_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, (uint8_t*)&status_reg_aux, 1);
    *val = status_reg_aux._3or;

    return ret;
}
/**
  * @brief  Temperature output value.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_temperature_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_AUX_ADC3_L, buff, 1);
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
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_adc_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_AUX_ADC1_L, buff, 6);
    return ret;
}

/**
  * @brief  Auxiliary ADC.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      configure the auxiliary ADC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_aux_adc_set(lis3dh_t *dev, lis3dh_temp_en_t val)
{
    lis3dh_temp_cfg_reg_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    if (ret == 0) {
        if (val != LIS3DH_AUX_DISABLE) {
            /* Required in order to use auxiliary adc */
            ret = lis3dh_block_data_update_set(dev, PROPERTY_ENABLE);
        }
    }
    if (ret == 0) {
        temp_cfg_reg.temp_en = ( (uint8_t) val & 0x02U) >> 1;
        temp_cfg_reg.adc_pd  = (uint8_t) val &  0x01U;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    }
    return ret;
}

/**
  * @brief  Auxiliary ADC.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      configure the auxiliary ADC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_aux_adc_get(lis3dh_t *dev, lis3dh_temp_en_t *val)
{
    lis3dh_temp_cfg_reg_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    if ( ( temp_cfg_reg.temp_en & temp_cfg_reg.adc_pd ) == PROPERTY_ENABLE ) {
        *val = LIS3DH_AUX_ON_TEMPERATURE;
    }
    if ( ( temp_cfg_reg.temp_en  == PROPERTY_DISABLE ) &&
            ( temp_cfg_reg.adc_pd == PROPERTY_ENABLE ) ) {
        *val = LIS3DH_AUX_ON_PADS;
    } else {
        *val = LIS3DH_AUX_DISABLE;
    }
    return ret;
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  *                  and HR in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_set(lis3dh_t *dev, lis3dh_op_md_t val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
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
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    if (ret == 0) {
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_get(lis3dh_t *dev, lis3dh_op_md_t *val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
        if ( ctrl_reg1.lpen == PROPERTY_ENABLE ) {
            *val = LIS3DH_LP_8bit;
            DEBUG("LIS3DH_LP_8bit\n");
        }
        if (ctrl_reg4.hr == PROPERTY_ENABLE ) {
            *val = LIS3DH_HR_12bit;
            DEBUG("LIS3DH_HR_12bit\n");
        } else {
            *val = LIS3DH_NM_10bit;
            DEBUG("LIS3DH_NM_10bit\n");
        }
    }
    return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_set(lis3dh_t *dev, lis3dh_odr_t val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ctrl_reg1.odr = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      get the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_get(lis3dh_t *dev, lis3dh_odr_t *val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_on_outputs_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.fds = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

/**
  * @brief   High pass data from internal filter sent to output register
  *          and FIFO.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fds in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_on_outputs_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_bandwidth_set(lis3dh_t *dev, lis3dh_hpcf_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hpcf = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      get the values of hpcf in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_bandwidth_get(lis3dh_t *dev, lis3dh_hpcf_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_mode_set(lis3dh_t *dev, lis3dh_hpm_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hpm = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

/**
  * @brief  High-pass filter mode selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      get the values of hpm in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_mode_get(lis3dh_t *dev, lis3dh_hpm_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_set(lis3dh_t *dev, lis3dh_fs_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.fs = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Full-scale configuration.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      get the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_get(lis3dh_t *dev, lis3dh_fs_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.fs) {
    case LIS3DH_2g:
        *val = LIS3DH_2g;
        DEBUG("LIS3DH_2g\n");
        break;
    case LIS3DH_4g:
        *val = LIS3DH_4g;
        DEBUG("LIS3DH_4g\n");
        break;
    case LIS3DH_8g:
        *val = LIS3DH_8g;
        DEBUG("LIS3DH_8g\n");
        break;
    case LIS3DH_16g:
        *val = LIS3DH_16g;
        DEBUG("LIS3DH_16g\n");
        break;
    default:
        *val = LIS3DH_2g;
        DEBUG("Default LIS3DH_2g\n");
        break;
    }
    return ret;
}

/**
  * @brief  Block Data Update.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.bdu = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Block Data Update.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    *val = (uint8_t)ctrl_reg4.bdu;

    return ret;
}

/**
  * @brief  Reference value for interrupt generation.[set]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that contains data to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_filter_reference_set(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_REFERENCE, buff, 1);
    return ret;
}

/**
  * @brief  Reference value for interrupt generation.[get]
  *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_filter_reference_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_REFERENCE, buff, 1);
    return ret;
}
/**
  * @brief  Acceleration set of data available.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of zyxda in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ready_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_t status_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t*)&status_reg, 1);
    *val = status_reg._zyxda;
    DEBUG("Bit ZYXDA STATUS_REG is %d\n", status_reg._zyxda);
    return ret;
}
/**
  * @brief  Acceleration set of data overrun.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of zyxor in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ovr_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_t status_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t*)&status_reg, 1);
    *val = status_reg._zyxor;

    return ret;
}
/**
  * @brief  Acceleration output value.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_acceleration_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_X_L, buff, 6);
    DEBUG("Acceleration Raw Data: ");
    PRINTBUFF(buff, 6);
    return ret;
}

/**
  * @brief  Acceleration output value.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_acceleration_raw_axis_get(lis3dh_t *dev, uint8_t reg_axis, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, reg_axis, buff, 2);
    DEBUG("Acceleration Raw Data: ");
    PRINTBUFF(buff, 2);
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
  * @param  dev      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_device_id_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_WHO_AM_I, buff, 1);
    return ret;
}
/**
  * @brief  Self Test.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_self_test_set(lis3dh_t *dev, lis3dh_st_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.st = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Self Test.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of st in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_self_test_get(lis3dh_t *dev, lis3dh_st_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_format_set(lis3dh_t *dev, lis3dh_ble_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.ble = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      get the values of ble in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_format_get(lis3dh_t *dev, lis3dh_ble_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_boot_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.boot = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_boot_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.boot;

    return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      register STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_status_get(lis3dh_t *dev, lis3dh_status_reg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t*) val, 1);
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
  * @param  dev      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_set(lis3dh_t *dev, lis3dh_int1_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_CFG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Interrupt generator 1 configuration register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_get(lis3dh_t *dev, lis3dh_int1_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_CFG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Interrupt generator 1 source register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Registers INT1_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_source_get(lis3dh_t *dev, lis3dh_int1_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_SOURCE, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[set]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int1_ths_t int1_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    if (ret == 0) {
        int1_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    }
    return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int1_ths_t int1_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    *val = (uint8_t)int1_ths.ths;

    return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int1_duration_t int1_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    if (ret == 0) {
        int1_duration.d = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    }
    return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int1_duration_t int1_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    *val = (uint8_t)int1_duration.d;

    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LIS3DH_Interrupts_generator_2
  * @brief      This section group all the functions that manage the second
  *             interrupts generator
  * @{
  *
  */

/**
  * @brief  Interrupt generator 2 configuration register.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers INT2_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_conf_set(lis3dh_t *dev, lis3dh_int2_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_CFG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Interrupt generator 2 configuration register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers INT2_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_conf_get(lis3dh_t *dev, lis3dh_int2_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_CFG, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  Interrupt generator 2 source register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers INT2_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_source_get(lis3dh_t *dev, lis3dh_int2_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_SOURCE, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief   User-defined threshold value for xl interrupt event on
  *          generator 2.[set]
  *          LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg INT2_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int2_ths_t int2_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    if (ret == 0) {
        int2_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    }
    return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 2.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg INT2_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int2_ths_t int2_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    *val = (uint8_t)int2_ths.ths;

    return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized .[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d in reg INT2_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_duration_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int2_duration_t int2_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    if (ret == 0) {
        int2_duration.d = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    }
    return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d in reg INT2_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_gen_duration_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int2_duration_t int2_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    *val = (uint8_t)int2_duration.d;

    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Interrupt_pins
  * @brief     This section group all the functions that manage interrup pins
  * @{
  *
  */

/**
  * @brief  High-pass filter on interrupts/tap generator.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of hp in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_int_conf_set(lis3dh_t *dev, lis3dh_hp_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hp = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

/**
  * @brief  High-pass filter on interrupts/tap generator.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of hp in reg CTRL_REG2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_high_pass_int_conf_get(lis3dh_t *dev, lis3dh_hp_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    switch (ctrl_reg2.hp) {
    case LIS3DH_DISC_FROM_INT_GENERATOR:
        *val = LIS3DH_DISC_FROM_INT_GENERATOR;
        break;
    case LIS3DH_ON_INT1_GEN:
        *val = LIS3DH_ON_INT1_GEN;
        break;
    case LIS3DH_ON_INT2_GEN:
        *val = LIS3DH_ON_INT2_GEN;
        break;
    case LIS3DH_ON_TAP_GEN:
        *val = LIS3DH_ON_TAP_GEN;
        break;
    case LIS3DH_ON_INT1_INT2_GEN:
        *val = LIS3DH_ON_INT1_INT2_GEN;
        break;
    case LIS3DH_ON_INT1_TAP_GEN:
        *val = LIS3DH_ON_INT1_TAP_GEN;
        break;
    case LIS3DH_ON_INT2_TAP_GEN:
        *val = LIS3DH_ON_INT2_TAP_GEN;
        break;
    case LIS3DH_ON_INT1_INT2_TAP_GEN:
        *val = LIS3DH_ON_INT1_INT2_TAP_GEN;
        break;
    default:
        *val = LIS3DH_DISC_FROM_INT_GENERATOR;
        break;
    }
    return ret;
}

/**
  * @brief  Int1 pin routing configuration register.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int1_config_set(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG3, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Int1 pin routing configuration register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int1_config_get(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG3, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  int2_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG (34h) is set to 1.
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d4d_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_pin_detect_4d_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.d4d_int2 = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief  4D enable: 4D detection is enabled on INT2 pin when 6D bit on
  *         INT2_CFG (34h) is set to 1.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d4d_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.d4d_int2;

    return ret;
}

/**
  * @brief   Latch interrupt request on INT2_SRC (35h) register, with
  *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
  *          itself.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of lir_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_pin_notification_mode_set(lis3dh_t *dev,
        lis3dh_lir_int2_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.lir_int2 = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief   Latch interrupt request on INT2_SRC (35h) register, with
  *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
  *          itself.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of lir_int2 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int2_pin_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_int2_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    switch (ctrl_reg5.lir_int2) {
    case LIS3DH_INT2_PULSED:
        *val = LIS3DH_INT2_PULSED;
        break;
    case LIS3DH_INT2_LATCHED:
        *val = LIS3DH_INT2_LATCHED;
        break;
    default:
        *val = LIS3DH_INT2_PULSED;
        break;
    }
    return ret;
}

/**
  * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit
  *                    on INT1_CFG(30h) is set to 1.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d4d_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_pin_detect_4d_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.d4d_int1 = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit on
  *         INT1_CFG(30h) is set to 1.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of d4d_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.d4d_int1;

    return ret;
}

/**
  * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
  *          register cleared by reading INT1_SRC (31h) itself.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of lir_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_pin_notification_mode_set(lis3dh_t *dev,
        lis3dh_lir_int1_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.lir_int1 = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
  *          register cleared by reading INT1_SRC (31h) itself.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of lir_int1 in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_pin_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_int1_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    switch (ctrl_reg5.lir_int1) {
    case LIS3DH_INT1_PULSED:
        *val = LIS3DH_INT1_PULSED;
        break;
    case LIS3DH_INT1_LATCHED:
        *val = LIS3DH_INT1_LATCHED;
        break;
    default:
        *val = LIS3DH_INT1_PULSED;
        break;
    }
    return ret;
}

/**
  * @brief  Int2 pin routing configuration register.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CTRL_REG6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int2_config_set(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG6, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Int2 pin routing configuration register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CTRL_REG6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int2_config_get(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG6, (uint8_t*) val, 1);
    return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Fifo
  * @brief     This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFO enable.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.fifo_en = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

/**
  * @brief  FIFO enable.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.fifo_en;

    return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_watermark_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.fth = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_watermark_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    *val = (uint8_t)fifo_ctrl_reg.fth;

    return ret;
}

/**
  * @brief  Trigger FIFO selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tr in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_trigger_event_set(lis3dh_t *dev, lis3dh_tr_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.tr = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

/**
  * @brief  Trigger FIFO selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of tr in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_trigger_event_get(lis3dh_t *dev, lis3dh_tr_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    switch (fifo_ctrl_reg.tr) {
    case LIS3DH_INT1_GEN:
        *val = LIS3DH_INT1_GEN;
        break;
    case LIS3DH_INT2_GEN:
        *val = LIS3DH_INT2_GEN;
        break;
    default:
        *val = LIS3DH_INT1_GEN;
        break;
    }
    return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.fm = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_mode_get(lis3dh_t *dev, lis3dh_fm_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    switch (fifo_ctrl_reg.fm) {
    case LIS3DH_BYPASS_MODE:
        *val = LIS3DH_BYPASS_MODE;
        break;
    case LIS3DH_FIFO_MODE:
        *val = LIS3DH_FIFO_MODE;
        break;
    case LIS3DH_DYNAMIC_STREAM_MODE:
        *val = LIS3DH_DYNAMIC_STREAM_MODE;
        break;
    case LIS3DH_STREAM_TO_FIFO_MODE:
        *val = LIS3DH_STREAM_TO_FIFO_MODE;
        break;
    default:
        *val = LIS3DH_BYPASS_MODE;
        break;
    }
    return ret;
}

/**
  * @brief  FIFO status register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_status_get(lis3dh_t *dev, lis3dh_fifo_src_reg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  FIFO stored data level.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of fss in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_data_level_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.fss;

    return ret;
}
/**
  * @brief  Empty FIFO status flag.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of empty in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_empty_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.empty;

    return ret;
}
/**
  * @brief  FIFO overrun status flag.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ovrn_fifo in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_ovr_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.ovrn_fifo;

    return ret;
}
/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of wtm in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_fth_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.wtm;

    return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Tap_generator
  * @brief     This section group all the functions that manage the tap and
  *            double tap event generation
  * @{
  *
  */

/**
  * @brief  Tap/Double Tap generator configuration register.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_conf_set(lis3dh_t *dev, lis3dh_click_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_CFG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Tap/Double Tap generator configuration register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_conf_get(lis3dh_t *dev, lis3dh_click_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_CFG, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  Tap/Double Tap generator source register.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      registers CLICK_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_source_get(lis3dh_t *dev, lis3dh_click_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_SRC, (uint8_t*) val, 1);
    return ret;
}
/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[set]
  *         1 LSB = full scale/128
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    if (ret == 0) {
        click_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    }
    return ret;
}

/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[get]
  *         1 LSB = full scale/128
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    *val = (uint8_t)click_ths.ths;

    return ret;
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_notification_mode_set(lis3dh_t *dev, lis3dh_lir_click_t val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    if (ret == 0) {
        click_ths.lir_click = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    }
    return ret;
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_click_t *val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    switch (click_ths.lir_click) {
    case LIS3DH_TAP_PULSED:
        *val = LIS3DH_TAP_PULSED;
        break;
    case LIS3DH_TAP_LATCHED:
        *val = LIS3DH_TAP_LATCHED;
        break;
    default:
        *val = LIS3DH_TAP_PULSED;
        break;
    }
    return ret;
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse
  *         between the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_shock_dur_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_limit_t time_limit;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    if (ret == 0) {
        time_limit.tli = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    }
    return ret;
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse between
  *         the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_shock_dur_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_limit_t time_limit;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    *val = (uint8_t)time_limit.tli;

    return ret;
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_quiet_dur_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_latency_t time_latency;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    if (ret == 0) {
        time_latency.tla = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    }
    return ret;
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_quiet_dur_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_latency_t time_latency;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    *val = (uint8_t)time_latency.tla;

    return ret;
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the click-detection
  *         procedure can start, in cases where the device is configured
  *         for double-click detection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_double_tap_timeout_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_window_t time_window;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    if (ret == 0) {
        time_window.tw = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    }
    return ret;
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the
  *         click-detection procedure can start, in cases where the device
  *         is configured for double-click detection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_double_tap_timeout_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_window_t time_window;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    *val = (uint8_t)time_window.tw;

    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Activity_inactivity
  * @brief     This section group all the functions concerning activity
  *            inactivity functionality
  * @{
  *
  */

/**
  * @brief    Sleep-to-wake, return-to-sleep activation threshold in
  *           low-power mode.[set]
  *           1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of acth in reg ACT_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_act_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_act_ths_t act_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    if (ret == 0) {
        act_ths.acth = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    }
    return ret;
}

/**
  * @brief  Sleep-to-wake, return-to-sleep activation threshold in low-power
  *         mode.[get]
  *         1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of acth in reg ACT_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_act_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_act_ths_t act_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    *val = (uint8_t)act_ths.acth;

    return ret;
}

/**
  * @brief  Sleep-to-wake, return-to-sleep.[set]
  *         duration = (8*1[LSb]+1)/ODR
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of actd in reg ACT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_act_timeout_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_act_dur_t act_dur;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    if (ret == 0) {
        act_dur.actd = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    }
    return ret;
}

/**
  * @brief  Sleep-to-wake, return-to-sleep.[get]
  *         duration = (8*1[LSb]+1)/ODR
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of actd in reg ACT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_act_timeout_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_act_dur_t act_dur;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    *val = (uint8_t)act_dur.actd;

    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS3DH_Serial_interface
  * @brief     This section group all the functions concerning serial
  *            interface management
  * @{
  *
  */

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of sdo_pu_disc in reg CTRL_REG0
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_sdo_sa0_mode_set(lis3dh_t *dev, lis3dh_sdo_pu_disc_t val)
{
    lis3dh_ctrl_reg0_t ctrl_reg0;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    if (ret == 0) {
        ctrl_reg0.sdo_pu_disc = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    }
    return ret;
}

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of sdo_pu_disc in reg CTRL_REG0
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_sdo_sa0_mode_get(lis3dh_t *dev, lis3dh_sdo_pu_disc_t *val)
{
    lis3dh_ctrl_reg0_t ctrl_reg0;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    switch (ctrl_reg0.sdo_pu_disc) {
    case LIS3DH_PULL_UP_DISCONNECT:
        *val = LIS3DH_PULL_UP_DISCONNECT;
        break;
    case LIS3DH_PULL_UP_CONNECT:
        *val = LIS3DH_PULL_UP_CONNECT;
        break;
    default:
        *val = LIS3DH_PULL_UP_DISCONNECT;
        break;
    }
    return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  dev      read / write interface definitions
  * @param  val      change the values of sim in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_spi_mode_set(lis3dh_t *dev, lis3dh_sim_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.sim = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  dev      read / write interface definitions
  * @param  val      Get the values of sim in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_spi_mode_get(lis3dh_t *dev, lis3dh_sim_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.sim) {
    case LIS3DH_SPI_4_WIRE:
        *val = LIS3DH_SPI_4_WIRE;
        break;
    case LIS3DH_SPI_3_WIRE:
        *val = LIS3DH_SPI_3_WIRE;
        break;
    default:
        *val = LIS3DH_SPI_4_WIRE;
        break;
    }
    return ret;
}

/** @} */
  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @defgroup  LIS3DH_Global_Functions
  * @brief     This section group all the global functions
  *
  * @{
  *
  */

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
    uint8_t dev_id;

    _platform_init(dev, params);

    /* test connection to the device */
    lis3dh_device_id_get(dev, &dev_id);
    if (dev_id != LIS3DH_WHO_AM_I_RESPONSE) {
        /* chip is not responding correctly */
        DEBUG("[lis3dh] error reading the who am i reg [0x%02x]\n", (int)dev_id);
        return -1;
    }

    // /* Enable all axis */
    // lis3dh_axes_set(dev, /* FIXME: Insert property */);

    /* Enable Block Data Update */
    DEBUG("Enable Block Data Update\n");
    lis3dh_block_data_update_set(dev, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    DEBUG("Set Output Data Rate [%d]\n", dev->params.odr);
    lis3dh_data_rate_set(dev, dev->params.odr); 

    /* Set full scale */ 
    DEBUG("Set full scale [%d]\n", dev->params.scale); 
    lis3dh_full_scale_set(dev, dev->params.scale);

    /* Set Big endian output value */
    DEBUG("Set Big endian output value\n");
    lis3dh_data_format_set(dev, LIS3DH_MSB_AT_LOW_ADD);

    /* Enable temperature sensor */
    DEBUG("Enable temperature sensor\n");   
    lis3dh_aux_adc_set(dev, LIS3DH_AUX_ON_TEMPERATURE);

    /* Set device operation mode */
    DEBUG("Set device operation mode [%d]\n", dev->params.op_mode);   
    lis3dh_operating_mode_set(dev, dev->params.op_mode);

    return 0;
}


/**
 * @brief These functions convert raw-data into engineering units.
 * 
 * @param dev [description]
 * @param acc_raw [description]
 * 
 * @return [description]
 */
int32_t lis3dh_calculation_acceleration(lis3dh_t *dev, int16_t acc_raw) {
    lis3dh_op_md_t op_mode;
    lis3dh_fs_t scale;

    DEBUG("Get device operation mode\n");
    lis3dh_operating_mode_get(dev, &op_mode);
    DEBUG("Get full scale\n");
    lis3dh_full_scale_get(dev, &scale);

    DEBUG("acc raw data: %d[%04X]\n", acc_raw, acc_raw);

    switch (op_mode) {
        case LIS3DH_HR_12bit:
            switch (scale) {
                case LIS3DH_2g:
                    
                    return ((acc_raw / 16) * 1);
                    break;
                case LIS3DH_4g:
                    return ((acc_raw / 16) * 2);
                    break;
                case LIS3DH_8g:
                    return ((acc_raw / 16) * 4);
                    break;
                case LIS3DH_16g:
                    return ((acc_raw / 16) * 12);
                    break;
            } 
            break;
        case LIS3DH_NM_10bit:
            switch (scale) {
                case LIS3DH_2g:
                    return ((acc_raw / 64) * 4);
                    break;
                case LIS3DH_4g:
                    return ((acc_raw / 64) * 8);
                    break;
                case LIS3DH_8g:
                    return ((acc_raw / 64) * 16);
                    break;
                case LIS3DH_16g:
                    return ((acc_raw / 64) * 48);
                    break;
            }
            break;
        case LIS3DH_LP_8bit:
            switch (scale) {
                case LIS3DH_2g:
                    return ((acc_raw / 256) * 16);
                    break;
                case LIS3DH_4g:
                    return ((acc_raw / 256) * 32);
                    break;
                case LIS3DH_8g:
                    return ((acc_raw / 256) * 64);
                    break;
                case LIS3DH_16g:
                    return ((acc_raw / 256) * 192);
                    break;
            }
            break;
    }

    return 0;
}

int lis3dh_read_xyz(lis3dh_t *dev, lis3dh_acceleration_t *acceleration) {
    lis3dh_reg_t reg;
    uint8_t acc_raw[6] = {0x00};
    /* Read output only if new value available */
    do {
        lis3dh_xl_data_ready_get(dev, &reg.byte);
        DEBUG("Bit ZYXDA STATUS_REG is %d\n", reg.status_reg._zyxda);
        DEBUG("Bit ZYXOR STATUS_REG is %d\n", reg.status_reg._zyxor);
        if (reg.status_reg._zyxda) {
            /* Read accelerometer data */
            lis3dh_acceleration_raw_get(dev, acc_raw);
            acceleration->axis_x = lis3dh_calculation_acceleration(dev, ((acc_raw[0] << 8) | acc_raw[1]));
            acceleration->axis_y = lis3dh_calculation_acceleration(dev, ((acc_raw[2] << 8) | acc_raw[3]));
            acceleration->axis_z = lis3dh_calculation_acceleration(dev, ((acc_raw[4] << 8) | acc_raw[5]));
            DEBUG("Acceleration [mg]:%d\t%d\t%d\n", acceleration->axis_x, acceleration->axis_y, acceleration->axis_z);
        }
        if (reg.status_reg._zyxor) {
            lis3dh_acceleration_raw_get(dev, acc_raw);
        }
    } while(!reg.status_reg._zyxda);

    return 0;

}



int32_t lis3dh_calculation_temperature(lis3dh_t *dev, int16_t temp_raw) {
    lis3dh_op_md_t op_mode;

    lis3dh_operating_mode_get(dev, &op_mode);

    switch (op_mode) {
        case LIS3DH_HR_12bit:
            return (((temp_raw / 64) / 4) + 25);
            break;
        case LIS3DH_NM_10bit:
            return (((temp_raw / 64) / 4) + 25);
            break;
        case LIS3DH_LP_8bit:
            return (((temp_raw / 256 ) * 1) + 25);
            break;
    }

    return 0;
}


int lis3dh_read_temp(lis3dh_t *dev, int16_t *temperature_degC) {

    lis3dh_reg_t reg;
    uint8_t data_raw_temperature;

    /* Read output only if new value available */
    do {
        lis3dh_temp_data_ready_get(dev, &reg.byte);      
        if (reg.byte) {
            /* Read temperature data */
            lis3dh_temperature_raw_get(dev, &data_raw_temperature);
            *temperature_degC = ((((uint16_t)data_raw_temperature) << 8) | 0x00);
            *temperature_degC = lis3dh_calculation_temperature(dev, *temperature_degC);
            DEBUG("Temperature [degC]: %d\n", *temperature_degC);
        }
    } while(!reg.byte);

    return 0;
}
/** @} */
/** @} */
