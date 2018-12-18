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
#define ISO14443A_FLAG_ATS_SUPPORTED		0x20

/* ISO7816 and APDU */                                   
/*  Iblock */
#define ISO7816_IBLOCK_02    				0x02
#define ISO7816_IBLOCK_03					0x03
#define ISO7816_TOGGLE_IBLOCK(block) ((block == ISO7816_IBLOCK_02)? ISO7816_IBLOCK_03 : ISO7816_IBLOCK_02) 

#define ISO7816_SELECT_FILE					0xA4
#define ISO7816_UPDATE_BINARY				0xD6
#define ISO7816_READ_BINARY  				0xB0

#define ISO7816_CLASS_0X00					0x00
/* ADPU-Header command structure */
typedef struct
{
    uint8_t cla;  /* Command class */
    uint8_t ins;  /* Operation code */
    uint8_t p1;   /* Selection Mode */
    uint8_t p2;   /* Selection Option */
} __attribute__((packed)) apdu_header_t;

/* ADPU-Body command structure -----------------------------------------------*/
typedef struct 
{
    uint8_t lc;         						  /* Data field length */	
    uint8_t data[256];  							/* Command parameters */ // pointer on the transceiver buffer = *(ReaderRecBuf[CR95HF_DATA_OFFSET + ISO7816_ADPUOFFSET_DATA])
    uint8_t le;          						 /* Expected length of data to be returned */
} __attribute__((packed)) apdu_body_t;

/* ADPU Command structure ----------------------------------------------------*/
typedef struct
{
    apdu_header_t header;
    apdu_body_t body;
} __attribute__((packed)) iso7816_apdu_cmd_t;

/* SC response structure -----------------------------------------------------*/
typedef struct
{
    uint8_t sw1;         						 /* Command Processing status */
    uint8_t sw2;          						/* Command Processing qualification */
} __attribute__((packed)) iso7816_apdu_responce_t;


int iso14443a_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);

uint8_t _iso14443a_apdu(const st95_t * dev);

#ifdef __cplusplus
}
#endif

#endif /* ISO14443A_H */
/** @} */
