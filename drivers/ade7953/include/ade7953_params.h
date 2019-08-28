/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file		ade7953_params.h
 * @brief       driver for ADE7953
 * @author      Mikhail Perkov
 */

#ifndef ADE7953_PARAMS_H
#define ADE7953_PARAMS_H


#ifdef __cplusplus
extern "C" {
#endif
#define ADE7953_SPI_CLK                    SPI_CLK_1MHZ

#define ADE7953_POWER_ON_DELAY_MS 100

#define ADE7953_NO_RESPONSE_TIME_MS        5000
#define ADE7953_NO_RESPONSE_TIME_MIN_USEC  250


/**
 * @brief   Protocol select commands
 */


#ifdef __cplusplus
}
#endif

#endif /* ADE7953_PARAMS_H */
/** @} */
