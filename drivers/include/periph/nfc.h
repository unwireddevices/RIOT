/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file        
 * @brief       
 * @author      Mikhail Perkov
 */

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_NFC_OK 		                0
#define NRF_NFC_ERROR 	                1

#define NRF_NFC_FRAMEDELAYMAX 0x0000FFFF    /**< 4832 usec */
#define NRF_NFC_FRAMEDELAYMIN 0x00000486  /**< FDT = 85 usec */
// #define NFC_FRAMEDELAYMIN 0x000004DF  /**< FDT = 91 usec */

#define NRF_NFC_ISO14443A_UID_LENGTH_MAX    10
#define NRF_NFC_NDEF_SIZE_MAX           (257) /**< !!! Max size may 4 bytes(0xFFFF) */
#define NRF_NFC_CC_FILE_SIZE            (0x000F) /**< 15 bytes */

#define NRF_NFC_ALL_INTERRUPTS 	        0x001D5CFF

#define NRF_NFC_ALL_ERRORS 	            0xDUL
#define NRF_NFC_ALL_RX_STATUS 	            0xDUL

#define NRF_NFC_MODE_UID_TAG                    1
#define NRF_NFC_MODE_DATA_RXTX                  0

#define NRF_NFC_CRC_SIZE                    2
#define NRF_NFC_RXTX_BUFFER_SIZE            NRF_NFC_NDEF_SIZE_MAX      /**< NFC Rx data buffer size */

/* ISO7816 and APDU */                                   
/*  Iblock */
#define NRF_NFC_ISO7816_IBLOCK_02    				0x02
#define NRF_NFC_ISO7816_IBLOCK_03					0x03

#define NRF_NFC_ISO7816_SELECT_FILE					0xA4
#define NRF_NFC_ISO7816_UPDATE_BINARY				0xD6
#define NRF_NFC_ISO7816_READ_BINARY  				0xB0

#define NRF_NFC_ISO14443A_CMD_RATS          0xE0
#define NRF_NFC_ISO14443A_CMD_HLTA			0x50
#define NRF_NFC_ISO14443A_CMD_DESELECT	    0xC2

#define NRF_NFC_NDEF_OK_SW_1                0x90
#define NRF_NFC_NDEF_OK_SW_2                0x00

/* Config ATS */
#define NRF_NFC_ATS_IS_TA1              1
#define NRF_NFC_ATS_IS_TB1              1
#define NRF_NFC_ATS_IS_TC1              1
#define NRF_NFC_ATS_IS_NAD              0
#define NRF_NFC_ATS_IS_CID              1
#define NRF_NFC_ATS_SFGI                0
#define NRF_NFC_ATS_FWI                 4

/**
 * @brief  States NDEF exchange */
typedef enum {
    NRF_NFC_STATE_NONE              = 0x00,
    NRF_NFC_STATE_RATS              = 0x01,
    NRF_NFC_STATE_CC_FILE           = 0x02,
    NRF_NFC_STATE_NDEF_FILE         = 0x03,
} ndef_exchange_state_t;

/**
 * @brief NFC ID size
 */
typedef enum
{
    NRF_NFC_UID_4_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Single, /**< Single size NFC ID (4 bytes). */
    NRF_NFC_UID_7_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Double, /**< Double size NFC ID (7 bytes). */
    NRF_NFC_UID_10_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Triple, /**< Triple size NFC ID (10 bytes). */
} nrf_nfc_id_size_t;


/**
 * @brief Protocol NFC (tag type)
 */
typedef enum
{
    NRF_NFC_PROTOCOL_TYPE2_TAG         = 0x0,  /**< Type 2 Tag platform. */
    NRF_NFC_PROTOCOL_TYPE4A_TAG        = 0x1,  /**< Type 4A Tag platform. */
} nrf_nfc_type_tag_t;


typedef struct {
    uint8_t * ndef_data;
    volatile uint8_t tx_length;
    uint16_t ndef_size;
    volatile uint8_t mode;
    ndef_exchange_state_t state;
}   __attribute__((packed)) nrf_nfc_tag_t;

void nfc_init(void);
uint8_t nfc_set_uid(uint8_t * uid, nrf_nfc_id_size_t size, nrf_nfc_type_tag_t tag_type);
uint8_t nfc_send_data(uint8_t * uid, nrf_nfc_id_size_t size, nrf_nfc_type_tag_t tag_type, uint8_t * data, uint16_t length);

#ifdef __cplusplus
}
#endif


