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
 * @file		onewire.h
 * @brief       driver for 1-Wire bus
 * @author      Mikhail Perkov
 */
#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include "periph/uart.h"

#include <stdlib.h>


/**
 * @brief 1-Wire return codes
*/
#define OW_OK			1
#define OW_ERROR		0
#define OW_NO_DEVICE	0

/*
 * @brief 1-Wire bus device
 */
#ifdef ONEWIRE_BITBANG_MODE
    typedef gpio_t onewire_t;
#else
    typedef uart_t onewire_t;
#endif

/**
 * @brief 1-Wire bus driver initialization routine
 *
 * @param[in] Device (UART or GPIO) to be used for 1-Wire communication
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int onewire_init(onewire_t device);

/**
 * @brief 1-Wire bus reset / device detection
 *
 * @param[in] None
 *
 * @return 1 if device detected
 * @return 0 if device not detected
 */
uint8_t onewire_detect(void);

/**
 * @brief 1-Wire command of read byte from a bus
 *
 * @param[out] Byte from 1-Wire bus
 *
 */
uint8_t onewire_readbyte(void);

/**
 * @brief 1-Wire command of send byte to a bus
 *
 * @param[in] Byte to 1-Wire bus
 * @param[out] Byte sent to 1-Wire bus
 *
 */
uint8_t onewire_sendbyte(uint8_t byte);

#endif /* ONEWIRE_H_ */
