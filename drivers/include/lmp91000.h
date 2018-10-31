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

/**
 * @brief Status register value definition
 */
enum { 
    LMP91000_NOT_READY = 0x00,               /**< Not ready to accept other I2C commands (default) */
    LMP91000_READY     = 0x01                /**< Ready to accept I2C commands                     */
};

/**
 * @brief Protection register value definition
 */
enum {
    LMP91000_WRITE_MODE     = 0x00,         /**< Writing on TIACN and REFCN registers allowed               */
    LMP91000_READ_ONLY_MODE = 0x01          /**< Writing on TIACN and REFCN registers not allowed (default) */
};

/**
 * @name TIA Control register value definition
 * @{
 */

/**
 * @brief TIA feedback resistance selection value definition
 */
enum {
    LMP91000_TIA_GAIN_EXT     = 0x00,       /**< External resistance (default)    */
    LMP91000_TIA_GAIN_2KOHM75 = 0x01,       /**< Intertnal resistance is 2.75kOhm */
    LMP91000_TIA_GAIN_3KOHM5  = 0x02,       /**< Intertnal resistance is 3.5kOhm  */
    LMP91000_TIA_GAIN_7KOHM   = 0x03,       /**< Intertnal resistance is 7kOhm    */
    LMP91000_TIA_GAIN_14KOHM  = 0x04,       /**< Intertnal resistance is 14kOhm   */
    LMP91000_TIA_GAIN_35KOHM  = 0x05,       /**< Intertnal resistance is 35kOhm   */
    LMP91000_TIA_GAIN_120KOHM = 0x06,       /**< Intertnal resistance is 120kOhm  */
    LMP91000_TIA_GAIN_350KOHM = 0x07        /**< Intertnal resistance is 350kOhm  */
};

/**
 * @brief Load resistance selection value definition
 */
enum{
    LMP91000_RLOAD_10OHM  = 0x00,           /**< Load resistance is 10Ohm            */
    LMP91000_RLOAD_33OHM  = 0x01,           /**< Load resistance is 33Ohm            */
    LMP91000_RLOAD_50OHM  = 0x02,           /**< Load resistance is 50Ohm            */
    LMP91000_RLOAD_100OHM = 0x03            /**< Load resistance is 100Ohm (default) */
};

/**
 * @}
 */


/**
 * @name Reference control register value definition
 * @{
 */

/**
 * @brief Reference voltage source selection value definition
 */
enum {
    LMP91000_REF_SOURCE_INT = 0x00,         /**< Reference voltage source: Internal (default) */
    LMP91000_REF_SOURCE_EXT = 0x01          /**< Reference voltage source: External           */
};

/**
 * @brief Internal zero selection (percentage of the source reference) value definition
 */
enum {
    LMP91000_INT_Z_20PCT  = 0x00,           /**< Internal zero: 20% of the source reference           */
    LMP91000_INT_Z_50PCT  = 0x01,           /**< Internal zero: 50% of the source reference (default) */
    LMP91000_INT_Z_67PCT  = 0x02,           /**< Internal zero: 67% of the source reference           */
    LMP91000_INT_Z_BYPASS = 0x03            /**< Internal zero bypassed                               */
};

/**
 * @brief Selection of the Bias polarity value definition
 */
enum {
    LMP91000_BIAS_SIGN_NEG = 0x00,          /**< Negative (VoltageWE - VoltageRE) < 0V (default) */
    LMP91000_BIAS_SIGN_POS = 0x01           /**< Positive (VoltageWE - VoltageRE) > 0V           */
};

/**
 * @brief BIAS selection (percentage of source reference) value definition
 */
enum {
    LMP91000_BIAS_00PCT = 0x00,             /**< Bias: 00% of the source reference (default) */
    LMP91000_BIAS_01PCT = 0x01,             /**< Bias: 01% of the source reference           */
    LMP91000_BIAS_02PCT = 0x02,             /**< Bias: 02% of the source reference           */
    LMP91000_BIAS_04PCT = 0x03,             /**< Bias: 04% of the source reference           */
    LMP91000_BIAS_06PCT = 0x04,             /**< Bias: 06% of the source reference           */
    LMP91000_BIAS_08PCT = 0x05,             /**< Bias: 08% of the source reference           */
    LMP91000_BIAS_10PCT = 0x06,             /**< Bias: 10% of the source reference           */
    LMP91000_BIAS_12PCT = 0x07,             /**< Bias: 12% of the source reference           */
    LMP91000_BIAS_14PCT = 0x08,             /**< Bias: 14% of the source reference           */
    LMP91000_BIAS_16PCT = 0x09,             /**< Bias: 16% of the source reference           */
    LMP91000_BIAS_18PCT = 0x0A,             /**< Bias: 18% of the source reference           */
    LMP91000_BIAS_20PCT = 0x0B,             /**< Bias: 20% of the source reference           */
    LMP91000_BIAS_22PCT = 0x0C,             /**< Bias: 22% of the source reference           */
    LMP91000_BIAS_24PCT = 0x0D              /**< Bias: 24% of the source reference           */
};

/**
 * @}
 */

/**
 * @name Mode control register value definition
 * @{
 */

/**
 * @brief Shorting FET feature value definition
 */
enum{
    LMP91000_FET_SHORT_DISABLED = 0x00,     /**< Short FET feature disabled (default) */
    LMP91000_FET_SHORT_ENABLED  = 0x01      /**< Short FET feature enabled */
};

/**
 * @brief Mode of Operation selection value definition
 */
enum {
    LMP91000_OP_MODE_DEEP_SLEEP        = 0x00,      /**< Mode of operation: Deep sleep (default)                 */
    LMP91000_OP_MODE_2_LEAD_GAL_CELL   = 0x01,      /**< Mode of operation: 2-lead ground referred galvanic cell */
    LMP91000_OP_MODE_STAND_BY          = 0x02,      /**< Mode of operation: Standby                              */
    LMP91000_OP_MODE_3_LEAD_AMP_CELL   = 0x03,      /**< Mode of operation: 3-lead amperometric cell             */
    LMP91000_OP_MODE_TEMP_MEAS_TIA_OFF = 0x06,      /**< Mode of operation: Temperature measurement (TIA OFF)    */
    LMP91000_OP_MODE_TEMP_MEAS_TIA_ON  = 0x07       /**< Mode of operation: Temperature measurement (TIA ON)     */
};

/**
 * @}
 */

/**
 * @brief TIACN register configuration parameters
 */
typedef struct {
    uint8_t r_load:2;
    uint8_t tia_gain:3;
    uint8_t reserved:3;
} lmp91000_tiacn_t;

/**
 * @brief REFCN register configuration parameters
 */
typedef struct {
    uint8_t bias:4;
    uint8_t bias_sign:1;
    uint8_t int_z:2;
    uint8_t ref_source:1;
} lmp91000_refcn_t;

/**
 * @brief MODECN register configuration parameters
 */
typedef struct {
    uint8_t op_mode:3;
    uint8_t reserved:4;
    uint8_t fet_short:1;
} lmp91000_modecn_t;


/**
 * @brief LMP91000 registers configuration parameters
 */
typedef struct {
    lmp91000_tiacn_t  tiacn;
    lmp91000_refcn_t  refcn;
    lmp91000_modecn_t modecn;
} lmp91000_config_t;


/**
 * @brief   LMP91000 hardware configuration parameters
 */
typedef struct {
    i2c_t   i2c;                            /**< I2C device  */
    uint8_t i2c_addr;                       /**< I2C address */
    gpio_t  module_en_pin;                  /**< GPIO to module enable on/off */
} lmp91000_params_t;

/**
 * @brief   LMP91000 device descriptor
 */
typedef struct {
    lmp91000_params_t params;               /**< Device configuration */
} lmp91000_t;


/**
 * @brief   Status and error return codes enumeration
 */
enum {
    LMP91000_OK          =  0,             /**< Everything was fine      */
    LMP91000_NOBUS       = -1,             /**< Bus interface error      */
    LMP91000_NODEV       = -2,             /**< Unable to talk to device */
    LMP91000_ERROR       = -3,             /**< Any error                */
    LMP91000_ERROR_PARAM = -4,             /**< Error parameter          */
};


/**
 * @brief This function initializes I2C interface and GPIO for LMP91000
 * 
 * @param dev    Pointer to LMP91000 device descriptor
 * @param params Static device configuration
 * 
 * @return Error code
 */
int lmp91000_init_hw(lmp91000_t *dev, const lmp91000_params_t params);

/**
 * @brief This function configure full internal registers LMP91000
 * 
 * @param dev        Pointer to LMP91000 device descriptor
 * @param reg_config Internal registers configuration
 * 
 * @return Error code
 */
int lmp91000_set_configure(lmp91000_t *dev, lmp91000_config_t reg_config);

/**
 * @brief This function configure operation mode LMP91000
 * 
 * @param dev     Pointer to LMP91000 device descriptor
 * @param op_mode Operation mode (Deep sleep, 2-lead ground referred galvanic cell
 *                                Standby, 3-lead amperometric cell, 
 *                                Temperature measurement (TIA OFF), 
 *                                Temperature measurement (TIA ON) ) 
 * 
 * @return Error code
 */
int lmp91000_set_operation_mode(lmp91000_t *dev, uint8_t op_mode);

#ifdef __cplusplus
}
#endif

#endif /* _LMP91000_H */