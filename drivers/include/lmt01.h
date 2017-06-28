/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		lmt01.h
 * @brief       LMT01 temperature sensor driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef LMT01_H_
#define LMT01_H_

#include "periph/gpio.h"

/**
 * @brief Maximum idle time
 */
#define LMT01_MAX_IDLE_TIME_MS 30

/**
 * @brief Minimum timeout of sensor response in milliseconds
 */
#define LMT01_MIN_TIMEOUT_MS 40

/*
 * @brief Maximum duration of pulses train in milliseconds
 */
#define LMT01_MAX_PULSES_TRAIN_TIMEOUT_MS 50

/**
 * @brief Possible LMT01 device driver states
 */
typedef enum {
	LMT01_UNKNOWN,
	LMT01_DETECTED,
	LMT01_MEASURING,
	LMT01_VALID_DATA,
} lmt01_state_t;

/**
 * @brief Describes internal state of the device driver
 */
typedef struct {
	uint16_t pulse_count;				/**< Last pulse count */
	lmt01_state_t state;				/**< Sensor's last known state */

	bool do_count;						/**< Allowed to count pulses */
} lmt01_internal_t;

/**
 * @brief Describes settings and holds driver's internal state
 */
typedef struct {
	gpio_t en_pin;					/**< Sensor "enable" pin */
	gpio_t sens_pin;				/**< GPIO pin on which sensor is attached */
	lmt01_internal_t _internal;		/**< Internal sensor data */
} lmt01_t;

/**
 * @brief Initializes the LMT01 sensor structure
 *
 * @param[in]	*lmt01		structure to initialize
 *
 * @return	<0	in case of error
 * @return	0	on success
 */
int lmt01_init(lmt01_t *lmt01, gpio_t en_pin, gpio_t sens_pin);

/**
 * @brief Tries to detect (blocking mode) an lmt01 sensor
 *
 * @param[in]	*lmt01	initialized lmt01 structure
 * @param[int]	timeout_ms	detection timeout in milliseconds*
 */
bool lmt01_detect(lmt01_t *lmt01, uint32_t timeout_ms);

/**
 * @brief Returns temperature measure from the specified initialized sensor
 *
 * @param[in]	*lmt01	initialized lmt01 structure
 * @param[out]	*temp	temperature measurement in Celsius degrees
 *
 * @return >0 if success (pulses count)
 * @return 0 on failure
 */
int lmt01_get_temp(lmt01_t *lmt01, int *temp);

#endif /* LMT01_H_ */
