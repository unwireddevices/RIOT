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

#define NRF_NFC_OK 		0
#define NRF_NFC_ERROR 	1

/**
 * @brief NFC ID size
 */
typedef enum
{
    NFC_UID_4_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Single, /**< Single size NFC ID (4 bytes). */
    NFC_UID_7_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Double, /**< Double size NFC ID (7 bytes). */
    NFC_UID_10_BYTES = NFCT_SENSRES_NFCIDSIZE_NFCID1Triple, /**< Triple size NFC ID (10 bytes). */
} nfc_id_size_t;

/**
 * @brief Protocol NFC (tag type)
 */
typedef enum
{
    NFC_PROTOCOL_TYPE2_TAG         = 0x0,  /**< Type 2 Tag platform. */
    NFC_PROTOCOL_TYPE4A_TAG        = 0x1,  /**< Type 4A Tag platform. */
} nfc_type_tag_t;

void nfc_init(void);
uint8_t nfc_set_uid(uint8_t * uid, nfc_id_size_t size, nfc_type_tag_t tag_type);

#ifdef __cplusplus
}
#endif


