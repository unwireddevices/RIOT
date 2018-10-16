/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lmp91000
 * @{
 *
 * @file
 * @brief       Definition for the LMP91000 Sensor AFE System
 * 
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef _LMP91000_H
#define _LMP91000_H



#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Status register bitfield definition */
#define LMP91000_NOT_READY                    (0x00) /* Not ready to accept other I2C commands */
#define LMP91000_READY                        (0x01) /* Ready to accept I2C commands */

/* Protection register */
#define LMP91000_WRITE_MODE                   (0x00) /* Writing on TIACN and REFCN registers allowed     */
#define LMP91000_READ_ONLY_MODE               (0x01) /* Writing on TIACN and REFCN registers not allowed */

/* TIA Control register */
#define LMP91000_TIA_GAIN_EXT                 (0x00) /* External resistance */
#define LMP91000_TIA_GAIN_2P75K               (0x04) /* 2.75K */
#define LMP91000_TIA_GAIN_3P5K                (0x08) /* 3.5K */
#define LMP91000_TIA_GAIN_7K                  (0x0C) /* 7K */
#define LMP91000_TIA_GAIN_14K                 (0x10) /* 14K */
#define LMP91000_TIA_GAIN_35K                 (0x14) /* 35K */
#define LMP91000_TIA_GAIN_120K                (0x18) /* 120K */
#define LMP91000_TIA_GAIN_350K                (0x1C) /* 350K */

#define LMP91000_RLOAD_10                     (0x00) /* 10 Ohm */
#define LMP91000_RLOAD_33                     (0x01) /* 10 Ohm */
#define LMP91000_RLOAD_50                     (0x02) /* 10 Ohm */
#define LMP91000_RLOAD_100                    (0x03) /* 10 Ohm */

/* Reference control reigster */
#define LMP91000_REF_SOURCE_INT               (0x00) /* Reference voltage source: Internal */
#define LMP91000_REF_SOURCE_EXT               (0x80) /* Reference voltage source: External */

#define LMP91000_INT_Z_20PCT                  (0x00) /* Internal zero: 20% of the source reference */
#define LMP91000_INT_Z_50PCT                  (0x20) /* Internal zero: 50% of the source reference */
#define LMP91000_INT_Z_67PCT                  (0x40) /* Internal zero: 67% of the source reference */
#define LMP91000_INT_Z_BYPASS                 (0x60) /* Internal zero bypassed                     */

#define LMP91000_BIAS_SIGN_NEG                (0x00) /* Negative (WE - RE) < 0V */
#define LMP91000_BIAS_SIGN_POS                (0x10) /* Positive (WE - RE) > 0V */

#define LMP91000_BIAS_00PCT                   (0x00) /* Bias: 00% of the source reference */
#define LMP91000_BIAS_01PCT                   (0x01) /* Bias: 01% of the source reference */
#define LMP91000_BIAS_02PCT                   (0x02) /* Bias: 02% of the source reference */
#define LMP91000_BIAS_04PCT                   (0x03) /* Bias: 04% of the source reference */
#define LMP91000_BIAS_06PCT                   (0x04) /* Bias: 06% of the source reference */
#define LMP91000_BIAS_08PCT                   (0x05) /* Bias: 08% of the source reference */
#define LMP91000_BIAS_10PCT                   (0x06) /* Bias: 10% of the source reference */
#define LMP91000_BIAS_12PCT                   (0x07) /* Bias: 12% of the source reference */
#define LMP91000_BIAS_14PCT                   (0x08) /* Bias: 14% of the source reference */
#define LMP91000_BIAS_16PCT                   (0x09) /* Bias: 16% of the source reference */
#define LMP91000_BIAS_18PCT                   (0x0A) /* Bias: 18% of the source reference */
#define LMP91000_BIAS_20PCT                   (0x0B) /* Bias: 20% of the source reference */
#define LMP91000_BIAS_22PCT                   (0x0C) /* Bias: 22% of the source reference */
#define LMP91000_BIAS_24PCT                   (0x0D) /* Bias: 24% of the source reference */

/* Mode control register */
#define LMP91000_FET_SHORT_DISABLED           (0x00) /* Short FET feature disabled */
#define LMP91000_FET_SHORT_ENABLED            (0x08) /* Short FET feature enabled */

#define LMP91000_OP_MODE_DEEP_SLEEP           (0x00) /* Mode of operation: Deep sleep */
#define LMP91000_OP_MODE_2_LEAD_GAL_CELL      (0x01) /* Mode of operation: Deep sleep */
#define LMP91000_OP_MODE_STAND_BY             (0x02) /* Mode of operation: Deep sleep */
#define LMP91000_OP_MODE_3_LEAD_AMP_CELL      (0x03) /* Mode of operation: Deep sleep */
#define LMP91000_OP_MODE_TEMP_MEAS_TIA_OFF    (0x06) /* Mode of operation: Deep sleep */
#define LMP91000_OP_MODE_TEMP_MEAS_TIA_ON     (0x07) /* Mode of operation: Deep sleep */

/**
 * @brief   LMP91000 hardware configuration parameters
 */
typedef struct {
    i2c_t   i2c;                        /**< I2C device  */
    uint8_t i2c_addr;                   /**< I2C address */
    gpio_t  module_en_pin;              /**< GPIO to module enable on/off */
} lmp91000_params_t;

/**
 * @brief   LMP91000 device descriptor
 */
typedef struct {
    m24sr_params_t params;              /**< Вevice configuration */
} m24sr_t;


int lmp91000_init_hw(lmp91000_t *dev, const lmp91000_params_t params);
int lmp91000_get_configure(lmp91000_t *dev, uint8_t *tiacn, uint8_t *refcn, uint8_t *modecn);
int lmp91000_set_configure(lmp91000_t *dev, const uint8_t tiacn, const uint8_t refcn, const uint8_t modecn);

#ifdef __cplusplus
}
#endif

#endif /* _LMP91000_H */