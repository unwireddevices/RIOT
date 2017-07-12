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
 * @brief Commands 1-Wire bus
*/



/**
 * @brief Status of the return of functions
*/
#define OW_OK			1
#define OW_ERROR		0
#define OW_NO_DEVICE	0

/*
 * @brief Type of 1-Wire bus device
 */
typedef uart_t onewire_t;

/**
 * @brief 1-Wire bus driver initialization routine
 *
 * @param[in] Number of the UART, which will using like as 1-Wire
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int ow_init(onewire_t device);

/**
 * @brief 1-Wire command of reset and presence device on a bus
 *
 * @param[in] None
 *
 * @return 1 if device detected
 * @return 0 if device not detected
 */
uint8_t ow_PRESENCE(void);

/**
 * @brief 1-Wire command of read bit from a bus
 *
 * @param[out] Bit from 1-Wire bus
 *
 */
uint8_t ow_ReadBit(void);

/**
 * @brief 1-Wire command of send bit to a bus
 *
 * @param[in] Bit to 1-Wire bus
 *
 */
void ow_SendBit(uint8_t bit);

/**
 * @brief 1-Wire command of read byte from a bus
 *
 * @param[out] Byte from 1-Wire bus
 *
 */
uint8_t ow_ReadByte(void);

/**
 * @brief 1-Wire command of send byte to a bus
 *
 * @param[in] Byte to 1-Wire bus
 * @param[out] Byte sent to 1-Wire bus
 *
 */
uint8_t ow_SendByte(uint8_t byte);

#endif /* ONEWIRE_H_ */
