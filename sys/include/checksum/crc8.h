/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_checksum_crc8     CRC8
 * @ingroup     sys_checksum
 * @brief       1-Wire CRC8 checksum algorithms
 *
 *
 * @{
 *
 * @file
 * @brief   crc8 definitions
 *
 * @author  
 */
#ifndef UCRC8_H_
#define UCRC8_H_

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#undef CRC8_USE_LOOKUP

/**
 * @brief   Calculate CRC16 (little-endian version)
 *
 * @param[in] data   Start of memory are to checksum
 * @param[in] length   Number of bytes in @p buf to calculate checksum for
 *
 * @return  Checksum of the specified memory area
 */
uint8_t crc8(const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* UCRC8_H_ */
/** @} */
