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
 * @file		mt3333.h
 * @brief       Mediatek MT3333-based GNSS module driver
 * @author      Eugene Ponomarev
 */
#ifndef mt3333_H_
#define mt3333_H_

#include "thread.h"
#include "ringbuffer.h"
#include "periph/uart.h"

/**
 * MT3333 NMEA message field indicies
 */
#define MT3333_TIME_FIELD_IDX 1
#define MT3333_VALID_FIELD_IDX 2
#define MT3333_LAT_FIELD_IDX 3
#define MT3333_NS_FIELD_IDX 4
#define MT3333_LON_FIELD_IDX 5
#define MT3333_EW_FIELD_IDX 6
#define MT3333_DATE_FIELD_IDX 9

#define MT3333_UART_BAUDRATE 9600
#define MT3333_EOL '\n'

/**
 * @brief Input ring buffer size in bytes
 */
#define MT3333_RXBUF_SIZE_BYTES (128)

/**
 * @brief Parser buf size, must not be smaller than the biggest GNMRC message possible
 */
#define MT3333_PARSER_BUF_SIZE (256)

/**
 * @brief Reader&Parser thread stack size in bytes
 */
#define MT3333_READER_THREAD_STACK_SIZE_BYTES (2048)

typedef struct tm mt3333_tm_t;

typedef struct {
	char lat[15];
	char lon[15];

	char date[15];
	char time[15];

	bool n, e;
} mt3333_gps_data_t;

/**
 * @biref Structure that holds the MT3333 driver parameters
 */
typedef struct {
	uart_t uart;	/**< The device descriptor on which the MT3333 module is attached */
	void (*gps_cb)(mt3333_gps_data_t data);	/**< Callback which called when module give us a valid GPS NMEA GMRC message */
} mt3333_param_t;

/**
 * @biref Structure that holds the MT3333 driver internal state and parameters
 */
typedef struct {
	mt3333_param_t params;					/**< Holds driver parameters */

	char rxbuf[MT3333_RXBUF_SIZE_BYTES];	/**< Memory buffer for the ring buffer data */
	ringbuffer_t rxrb;						/**< Holds incoming data ring buffer */

	uint8_t *reader_stack;	                /**< Reader thread stack, has to be allocated by the application */
	kernel_pid_t reader_pid;				/**< Reader thread PID */
} mt3333_t;

/**
 * @brief MT3333 driver initialization routine
 * @note Initializes the UART device specified in parameters
 * @note Initialies input ring buffer with size mt3333_RXBUF_SIZE_BYTES
 *
 * @param[out] dev device structure pointer
 * @param[in] param MT3333 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int mt3333_init(mt3333_t *dev, mt3333_param_t *param);

#endif /* mt3333_H_ */
