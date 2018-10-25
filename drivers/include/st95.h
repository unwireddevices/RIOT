/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file		st95.h
 * @brief       driver for ST95
 * @author      Mikhail Perkov
 */
#ifndef ST95_H_
#define ST95_H_

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "periph/spi.h"

/**
 * @brief ST95 return codes
*/
#define ST95_OK			1
#define ST95_ERROR		0
#define ST95_NO_DEVICE	0

/**
 * @brief   ST95 hardware and global parameters.
 */
typedef struct {
    uint8_t spi;        /**< SPI device */
    gpio_t cs_spi;      /**< SPI NSS pin */
    gpio_t irq_in;      /**< Interrupt input */
    gpio_t irq_out;     /**< Interrupt output */
    gpio_t ssi_0;       /**< Select serial communication interface */
    gpio_t ssi_1;       /**< Select serial communication interface */
} st95_params_t;

/**
 * @brief ST95 bus driver initialization routine
 *
 * @param[in] Device (SPI) to be used for ST95 communication
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int st95_init(st95_params_t * device);

int st95_echo(void);
int st95_idn(uint8_t * idn, uint8_t * length);
uint8_t st95_idle(void);
void st95_send_receive(uint8_t *data, uint8_t size, uint8_t topaz, uint8_t split_frame, uint8_t crc, uint8_t sign_bits);
uint8_t st95_receive(uint8_t * rxbuff);
int st95_get_uid(uint8_t * length_uid, uint8_t * uid, uint8_t * sak);
#endif /* ST95_H_ */
