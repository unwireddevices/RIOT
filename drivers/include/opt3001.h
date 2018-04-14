/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file		opt3001.h
 * @brief       driver for OPT3001 sensor
 * @author      Oleg Artamonov [info@unwds.com]
 * @author      Dmitriy Faychuk [checordog@gmail.com]
 */
#ifndef OPT3001_H_
#define OPT3001_H_

#include "thread.h"
#include "periph/i2c.h"

#include <stdlib.h>
#include <math.h>

/**
 * @brief Initial OPT3001 address on I2C bus
 */
#define OPT3001_ADDRESS 0x44 // ADDR = GND
/*
#define OPT3001_ADDRESS 0x45 // ADDR = VDD
#define OPT3001_ADDRESS 0x46 // ADDR = SDA
#define OPT3001_ADDRESS 0x47 // ADDR = SCL
*/

/* bytes are swapped */
#define OPT3001_CFG_FC_1    0x0000
#define OPT3001_CFG_FC_2    0x0100
#define OPT3001_CFG_FC_4    0x0200
#define OPT3001_CFG_FC_8    0x0300
#define OPT3001_CFG_MASK    0x0400
#define OPT3001_CFG_POLNEG  0x0000
#define OPT3001_CFG_POLPOS  0x0800
#define OPT3001_CFG_HYSTER  0x0000
#define OPT3001_CFG_LATCH   0x1000
#define OPT3001_CFG_FLAGL   0x2000
#define OPT3001_CFG_FLAGH   0x4000
#define OPT3001_CFG_CRF     0x8000
#define OPT3001_CFG_OVF     0x0001
#define OPT3001_CFG_SHDN    0x0000
#define OPT3001_CFG_SHOT    0x0002
#define OPT3001_CFG_CONT    0x0004
#define OPT3001_CFG_100MS   0x0000
#define OPT3001_CFG_800MS   0x0008
#define OPT3001_CFG_RNAUTO  0x00C0

#define OPT3001_CFG (OPT3001_CFG_FC_1 | OPT3001_CFG_SHOT | OPT3001_CFG_100MS | OPT3001_CFG_RNAUTO )
#define OPT3001_CFG_INT (OPT3001_CFG_CONT | OPT3001_CFG_800MS | OPT3001_CFG_RNAUTO)
#define OPT3001_CFG_DEFAULT 0x10C8

/**
 * @brief OPT3001 registers
 */
#define OPT3001_REG_RESULT        0x00
#define OPT3001_REG_CONFIG        0x01
#define OPT3001_REG_LOW			  0x02
#define OPT3001_REG_HIGH          0x03
#define OPT3001_REG_ID            0x7E
#define OPT3001_CHIP_ID           0x4954
#define OPT3001_REG_CONFIG_MASK   0xFE1F


/**
 * @brief Enum that defines the OPT3001 fault counter states
 */
typedef enum {
	OPT3001_FC_1 = (OPT3001_CFG_FC_1), 	/**< Threshold of faults is 1 */
	OPT3001_FC_2 = (OPT3001_CFG_FC_2), 	/**< Threshold of faults is 2 */
	OPT3001_FC_4 = (OPT3001_CFG_FC_4), 	/**< Threshold of faults is 4 */
	OPT3001_FC_8 = (OPT3001_CFG_FC_8) 	/**< Threshold of faults is 8 */
} opt3001_fault_counter_t;

/**
 * @brief Enum that defines the comparison modes of the OPT3001 Interrupt Reporting Mechanism
 */
typedef enum {
	OPT3001_LATCHED = (OPT3001_CFG_LATCH), 		/**< Latched Window-Style Comparison Mode, requires manual clearing of the INT pin */
	OPT3001_TRANSPARENT = (OPT3001_CFG_HYSTER)	/**< Transparent Hysteresis-Style Comparison Mode */
} opt3001_cmp_mode_t;


/**
 * @brief Enum that defines the OPT3001 end of conversion mode, which allow INT pin going active on every measurement completion
 */
typedef enum {
	OPT3001_NO_EOC,	/**< Disabled */
	OPT3001_EOC     /**< Enabled */
} opt3001_eoc_mode_t;

/**
 * @brief Enum that defines the OPT3001 polarity or active state of the INT pin
 */
typedef enum {
	OPT3001_IRQ_ON_LOW = (OPT3001_CFG_POLNEG),		/**< Active state will be LOW */
	OPT3001_IRQ_ON_HIGH = (OPT3001_CFG_POLPOS)		/**< Active state will be HIGH */
} opt3001_pol_t;


/**
 * @brief Structure that holds the OPT3001 driver internal state and parameters
 */
typedef struct {
    i2c_t i2c;    /**< Holds driver parameters */

} opt3001_t;


typedef struct {
    uint32_t luminocity;        /**< Illuminance in lux */
} opt3001_measure_t;

/**
 * @brief OPT3001 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev 		device structure pointer
 * @param[in] param 	OPT3001 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int opt3001_init(opt3001_t *dev);

/**
 * @brief Get OPT3001 measure
 *
 * @param[in] dev 		pointer to the initialized OPT3001 device
 * @param[in] measure 	pointer to the allocated memory
 *
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * opt3001_measure_t measure;
 *
 * opt3001_init(opt3001);
 * opt3001_measure(opt3001, &measure)
 */
uint32_t opt3001_measure(opt3001_t *dev, opt3001_measure_t *measure);

/**
 * @brief Init opt3001 Interrupt Reporting Mechanism
 *
 * @param[in] dev 	pointer to the initialized OPT3001 device
 * @param[in] cmp 	comparison modes of the IRM
 * @param[in] eoc	end of conversion mode of the IRM
 * @param[in] pol	polarity of the INT pin
 * @param[in] flt	fault counter threshold
 * @param[in] high	HIGH limit value [0.01 - 83865.6], obviously should be higher than LOW 
 * @param[in] low 	LOW limit value [0.01 - 83865.6], obviously should be lower than HIGH
 *
 * @return 0 if succeeded
 *
 */
int opt3001_init_int(opt3001_t *dev, opt3001_cmp_mode_t cmp, opt3001_eoc_mode_t eoc, 
		opt3001_pol_t pol, opt3001_fault_counter_t flt, 
		uint32_t high, uint32_t low);

/**
 * @brief OPT3001 INTpin/FlagH/FlagL clearing routine
 * @note Only works with Latched Window-style comparison mode
 *
 * @param[in] dev 	pointer to the initialized OPT3001 device
 *
 * @return 0 if succeeded
 *
 */
int opt3001_clear_int(opt3001_t *dev);


#endif /* OPT3001_H_ */
