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

#define ISO14443A_UID_LENGTH_MAX	            10

/* Command code */
#define ISO14443A_CMD_REQA                  0x26
#define ISO14443A_CMD_WUPA					0x52
#define ISO14443A_CMD_HLTA					0x50
#define ISO14443A_CMD_RATS  				0xE0
#define ISO14443A_CMD_PPS					0xD0
#define ISO14443A_CMD_DESELECT				0xC2

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

#define ISO14443A_APDU_CMD_MAX_BYTE			64

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
#define ISO14443A_FLAG_ATS_SUPPORTED		0x20

#define ISO14443A_TYPE_2		            2
#define ISO14443A_TYPE_4		            4
#define ISO14443A_TYPE_UNKNOWN		        0

/* RATS */
#define ISO14443A_CID_DEF                   0x00
#define ISO14443A_FSD_256                   0x08

/* ISO7816 and APDU */                                   
/*  Iblock */
#define ISO7816_IBLOCK_02    				0x02
#define ISO7816_IBLOCK_03					0x03

#define ISO7816_SELECT_FILE					0xA4
#define ISO7816_UPDATE_BINARY				0xD6
#define ISO7816_READ_BINARY  				0xB0

#define ISO7816_CLASS_0X00					0x00

#define ISO14443A_CRC16_LENGTH              2
#define ISO14443A_SERVICE_DATA_LENGTH       3

#define NDEF_OFFSET_PCB                     ST95_DATA_OFFSET
#define NDEF_MASK_BLOCK                     0xC0
#define NDEF_MASK_IBLOCK                    0x00
#define NDEF_MASK_RBLOCK                    0x80
#define NDEF_MASK_SBLOCK                    0xC0
#define NDEF_OK_SW_1                        0x90
#define NDEF_OK_SW_2                        0x00

#define NDEF_OFFSET_DATA (ST95_DATA_OFFSET + 1)

#define NDEF_PACK_OFFSET_DATA 0x02

/**
 * @brief   ISO14443A card parameters
 */
typedef struct {
    uint8_t type;                           /**<  */
    uint8_t sak;                            /**<  */
    uint8_t uid_length;                     /**<  */
    uint8_t uid[ISO14443A_UID_LENGTH_MAX];  /**<  */
    
    bool is_ats;                            /**<  */
    uint16_t cc_size;                       /**<  */
    uint16_t ndef_length;
    uint16_t ndef_id;                       /**<  */
    uint16_t ndef_read_max;              /**<  */
    uint16_t ndef_write_max;             /**<  */
    uint16_t ndef_size;                     /**<  */
}   __attribute__((packed)) iso14443a_picc_t;

int iso14443a_get_uid(const st95_t * dev, uint8_t * iso_rxbuf, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);
int iso14443a_write_tag(const st95_t * dev, uint8_t * txbuff, uint8_t length, uint8_t * rxbuff);
int iso14443a_read_tag(const st95_t * dev, uint8_t * data, uint16_t length, uint8_t * rxbuff);

#ifdef __cplusplus
}
#endif

#endif /* ISO14443A_H */
/** @} */
