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
 * @file		evam8.h
 * @brief       U-blox EVA-M8 GNSS module driver implementation (I2C only)
 * @author      EP <ep@unwds.com>
 * @author      Oleg Artamonov <oleg@unwds.com>
 */
#ifndef evam8_H_
#define evam8_H_

#include "thread.h"
#include "periph/i2c.h"

#include <time.h>

/**
 * EVA-M8 NMEA message field indicies
 */
#define EVAM8_RMC_TIME_FIELD_IDX       1
#define EVAM8_RMC_VALID_FIELD_IDX      2
#define EVAM8_RMC_LAT_FIELD_IDX        3
#define EVAM8_RMC_NS_FIELD_IDX         4
#define EVAM8_RMC_LON_FIELD_IDX        5
#define EVAM8_RMC_EW_FIELD_IDX         6
#define EVAM8_RMC_VELOCITY_FIELD_IDX   7
#define EVAM8_RMC_DIRECTION_FIELD_IDX  8
#define EVAM8_RMC_DATE_FIELD_IDX       9

#define EVAM8_GGA_TIME_FIELD_IDX       1
#define EVAM8_GGA_LAT_FIELD_IDX        2
#define EVAM8_GGA_NS_FIELD_IDX         3
#define EVAM8_GGA_LON_FIELD_IDX        4
#define EVAM8_GGA_EW_FIELD_IDX         5
#define EVAM8_GGA_VALID_FIELD_IDX      6
#define EVAM8_GGA_SATS_FIELD_IDX       7
#define EVAM8_GGA_HDOP_FIELD_IDX       8
#define EVAM8_GGA_HEIGHT_FIELD_IDX     9

#define EVAM8_EOL '\n'

#define EVAM8_I2C_ADDR                 0x42
#define EVAM8_I2C_REG_DATA_AVAIL       0xFD
#define EVAM8_I2C_REG_DATA_DATA        0xFF

/**
 * @brief Input ring buffer size in bytes
 */
#define EVAM8_RXBUF_SIZE_BYTES (2048)

/**
 * @brief Reader&Parser thread stack size in bytes
 */
#define EVAM8_READER_THREAD_STACK_SIZE_BYTES (4096)

typedef struct {
	int lat;            /**< Latitude, degrees * 1E6 */
	int lon;            /**< Logitude, degrees * 1E6 */
    int velocity;       /**< Velocity, mm/s */
    int direction;      /**< Direction, degrees * 1E3, relative to north */
    int height;         /**< Height, centimeters */
	time_t time;        /**< Epoch */
    uint8_t satellites; /**< Number of satellites */
    bool valid;         /**< Data validity */
} evam8_gps_data_t;

/**
 * @biref Structure that holds the EVA-M8 driver parameters
 */
typedef struct {
	i2c_t i2c_dev;	                        /**< The device descriptor on which the EVA-M8 module is attached */
	void (*gps_cb)(evam8_gps_data_t data);	/**< Callback which called when module give us a valid GPS NMEA GMRC message */
} evam8_param_t;

/**
 * @biref Structure that holds the EVA-M8 driver internal state and parameters
 */
typedef struct {
	evam8_param_t params;					/**< Holds driver parameters */
	char *reader_stack;	                    /**< Reader thread stack, has to be allocated by the application */
    bool ready;                             /**< GPS is ready to accept commands */
} evam8_t;

typedef enum {
    EVAM8_POWERSAVE_STANDBY,
    EVAM8_POWERSAVE_BACKUP,
    EVAM8_POWERSAVE_FULLON,
} evam8_powersave_mode_t;

typedef enum {
    EVAM8_READY,
    EVAM8_TIMEOUT,
} evam8_status_t;

/**
 * @brief EVA-M8 driver initialization routine
 * @note Initializes the I2C device specified in parameters
 * @note Initialies input ring buffer with size evam8_RXBUF_SIZE_BYTES
 *
 * @param[out] dev device structure pointer
 * @param[in] param EVA-M8 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int evam8_init(evam8_t *dev, evam8_param_t *param);


/**
 * @brief Set EVA-M8 to the specified power mode
 */
void evam8_set_powersave(evam8_t *dev, evam8_powersave_mode_t mode);

/**
 * @brief Set EVA-M8 to periodic mode with specified parameters
 */
void evam8_set_periodic(evam8_t *dev, evam8_powersave_mode_t mode, int run, int sleep, int run_ext, int sleep_ext);

/**
 * @brief Checks if EVA-M8 is ready to accept commands
 *
 * @param[out] dev device structure pointer
 *
 * @return EVAM8_READY if modem is ready
 * @return EVAM8_TIMEOUT in case of error
 */
int evam8_is_ready(evam8_t *dev);

#endif /* evam8_H_ */
