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
 * @file		mhz19.h
 * @brief       CO2 sensor module MH-Z19 UART driver
 * @author      Dmitry Golik
 * @author      Oleg Artamonov
 */
#ifndef MHZ19_H_
#define MHZ19_H_

#include "thread.h"
#include "ringbuffer.h"
#include "periph/uart.h"

#define MHZ19_UART_BAUDRATE 9600

/**
 * @brief Input ring buffer size in bytes
 */
#define MHZ19_RXBUF_SIZE_BYTES (9)

/**
 * @brief Reader&Parser thread stack size in bytes
 */
#define MHZ19_READER_THREAD_STACK_SIZE_BYTES (2048)

#define MHZ19_TIMEOUT_SECONDS (2)

typedef struct {
	int16_t co2; /* ppm */
	int16_t temperature; /* 0.1 deg C */
	uint8_t validity; /* 64 means data is stable */
} mhz19_data_t;

/**
 * @biref Structure that holds MH-Z19 driver parameters
 */
typedef struct {
	uart_t uart;	/**< The device descriptor on which the MH-Z19 module is attached */
	void (*mhz19_cb)(mhz19_data_t data);	/**< Callback which called when module give us a valid values */
} mhz19_param_t;

/**
 * @biref Structure that holds MH-Z19 driver internal state and parameters
 */
typedef struct {
	mhz19_param_t params;					/**< Holds driver parameters */
	uint8_t rxbuf[MHZ19_RXBUF_SIZE_BYTES];     /**< Memory buffer for the data */
	uint8_t *reader_stack;	/**< Reader thread stack */
	kernel_pid_t reader_pid;				/**< Reader thread PID */
} mhz19_t;

/**
 * @brief MH-Z19 driver initialization routine
 * @note Initializes the UART device specified in parameters
 *
 * @param[out] dev device structure pointer
 * @param[in] param MH-Z19 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int mhz19_init(mhz19_t *dev, mhz19_param_t *param);

/**
 * @brief Send command to MH-Z19 to start measurements
 *
 * @param[out] dev device structure pointer
 *
 */
void mhz19_get(mhz19_t *dev);

#endif /* MHZ19_H_ */
