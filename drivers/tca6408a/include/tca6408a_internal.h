/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_tca6408a
 * @{
 *
 * @file
 * @brief       Registers definition for the TI TCA6408A I/O Expander 
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef _TCA6408A_INTERNAL_H
#define _TCA6408A_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    TCA6408A registers
 * @{
 */

/**
 * @brief   Input status register
 */
#define TCA6408A_INPUT_REG                              0x00

/**
 * @brief   Output register to change state of output BIT set to 1, output set HIGH
 */
#define TCA6408A_OUTPUT_REG                             0x01

/**
 * @brief   Polarity inversion register. BIT '1' inverts input polarity of register 0x00
 */
#define TCA6408A_POLARITY_REG                           0x02

/**
 * @brief   Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output
 */
#define TCA6408A_CONFIG_REG                             0x03

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _TCA6408A_INTERNAL_H */
/** @} */