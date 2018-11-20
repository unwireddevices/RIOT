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
 * @file		iso14443a.h
 * @brief       protocol ISO14443A for ST95
 * @author      Mikhail Perkov
 */

#ifndef ISO14443A_H
#define ISO14443A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Command code */
#define ISO14443A_CMD_REQA                  0x26
#define ISO14443A_CMD_WUPA					0x52
#define ISO14443A_CMD_HLTA					0x50
#define ISO14443A_CMD_RATS  				0xE0
#define ISO14443A_CMD_PPS					0xD0

#define ISO14443A_OFFSET_UID_SELECT			2
#define ISO14443A_OFFSET_SAK_BYTE			ST95_DATA_OFFSET
#define ISO14443A_OFFSET_ATQA_FIRST_BYTE	ST95_DATA_OFFSET
#define ISO14443A_OFFSET_UID_SIZE			6

#define ISO14443A_MASK_UID_SIZE				0x03

/* Protocol select parameters 	*/
#define ST95_TX_RATE_14443A                 ST95_TX_RATE_106
#define ST95_RX_RATE_14443A                 ST95_RX_RATE_106

/* Transmission flags: command control byte value */
#define ISO14443A_NUM_SIGN_BIT_7			0x07
#define ISO14443A_NUM_SIGN_BIT_8			0x08
#define ISO14443A_APPEND_CRC			    0x20

#define ISO14443A_CMD_MAX_BYTE				16
#define ISO14443A_ANSWER_MAX_BYTE			32

/* Anticollison levels (commands) */
#define ISO14443A_SELECT_LVL1               0x93
#define ISO14443A_SELECT_LVL2	            0x95
#define ISO14443A_SELECT_LVL3	            0x97

/* NVB(Number of Valid Bits) bytes values */
#define ISO14443A_NVB_10					0x10
#define ISO14443A_NVB_20					0x20
#define ISO14443A_NVB_30					0x30
#define ISO14443A_NVB_40					0x40
#define ISO14443A_NVB_50					0x50
#define ISO14443A_NVB_60					0x60
#define ISO14443A_NVB_70					0x70

/* ATQ FLAG */
#define ISO14443A_ATQA_SINGLE	            0
#define	ISO14443A_ATQA_DOUBLE	            1
#define ISO14443A_ATQA_TRIPLE	            2
/* UID Sizes */
#define ISO14443A_UID_SINGLE	            4
#define ISO14443A_UID_DOUBLE	            7
#define ISO14443A_UID_TRIPLE	            10
#define ISO14443A_NUM_BYTE_SELECT           5							

#define ISO14443A_MASK_SAK_UID_NOT_COMPLETE 0x04
#define ISO14443A_SAK_UID_NOT_COMPLETE      0x04


int iso14443a_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);

#ifdef __cplusplus
}
#endif

#endif /* ISO14443A_H */
/** @} */
