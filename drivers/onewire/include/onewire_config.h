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
 * @file		onewire_config.h
 * @brief       Default configuration for 1-Wire bus
 * @author      Mikhail Perkov
 */
#ifndef ONEWIRE_CONFIG_H_
#define ONEWIRE_CONFIG_H_

/**
 * @brief 1-Wire delay before checking received byte
 */
#define OW_DELAY 25

/**
 * @brief Operations with bits
 */
#define _BITTING(n) (1<<n)
#define _CHKBIT(reg, n) (reg & _BITTING(n))
#define _SETBIT(reg,n) (reg |= _BITTING(n))


/**
 * @brief Baudrates of using UART
 */
#define ONEWIRE_BAUD_9600 9600 // 9600
#define ONEWIRE_BAUD_115200 115200 // 115200

/**
 * @brief Values bytes to UART for work with 1-Wire bus
 */
#define OW_RESET 0xF0
#define OW_WRITE_1 0xFF
#define OW_WRITE_0 0x00
#define OW_READ 0xFF


#endif /* ONEWIRE_CONFIG_H_ */
