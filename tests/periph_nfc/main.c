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
 * @brief       Manual test application for NFC peripheral drivers
 * @author      Mikhail Perkov
 */

#include <stdio.h>
#include "board.h"
#include "periph/nfc.h"
#include "xtimer.h"

#define NFC_MODE_TEST (1)

static uint8_t uid_test[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0xAA, 0xBB, 0xCC };

#if NFC_MODE_TEST
// static uint8_t ndef_data[] = { 0xD1,                           // MB / ME / CF / 1 / IL / TNF
                             // 0x01,                                              // TYPE LENGTH
                             // 51,                                                // PAYLOAD LENTGH
                             // 'T',                                               // TYPE
                             // 0x02,                                              // Status
// 13, 10, 13, 10, 32, 32, 32, 32, 32, 
// 'M', 'i', 'k', 'r', 'o', 'E', 'l', 'e', 'k', 't', 'r', 'o', 'n', 'i', 'k', 'a', 
// 13, 10, 13, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
// 'N', 'F', 'C', ' ', 'c', 'l', 'i', 'c', 'k'
// };

static uint8_t ndef_data_text[] = {
0, 0,
13, 10,
35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
13, 10,
42, ' ', ' ', ' ', 'n', 'R', 'F', '5', '2', '8', '3', '2',
13, 10,
42, ' ', ' ', ' ', 'T', 'e', 's', 't', 'i', 'n', 'g', 
' ', 'N', 'F', 'C', ' ', 'D', 'a', 't', 'a', ' ', 'e', 'x', 'c', 'h',
'a', 'n', 'g', 'e', '.', '.', '.',
13, 10, 42, 13, 10, 
42, ' ', ' ', ' ', 'U', 'n', 'w', 'i', 'r', 'e', 'd', ' ', 'D', 'e', 'v', 'i', 'c', 'e', 's', 
' ', 'L', 'L', 'C',
13, 10,
42, ' ', ' ', ' ', 'w', 'w', 'w', '.', 'u', 'n', 'w', 'd', 's', '.', 'c', 'o', 'm',
13,10,
42, ' ', ' ', ' ', 'i', 'n', 'f', 'o', '@', 'u', 'n', 'w', 'd', 's', '.', 'c', 'o', 'm',
13,10,
42, ' ', ' ', ' ', 0x32, 0x30, 0x31, 0x39,
13,10,
35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
13, 10,
42, ' ', ' ', ' ', 'E', 'n', 'd',
13, 10,
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42,
13, 10,
42, ' ', ' ', ' ', 'T', 'e', 's', 't', 'e', 'd', ' ', 'b', 'y', ' ', 
'M', 'i', 'k', 'h', 'a', 'i', 'l', ' ', 'P', 'e', 'r', 'k', 'o', 'v',
13, 10,
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42,
13, 10,
37, 37, 13, 10, 36
};

#endif

int main(void)
{   
    nfc_init();

#if NFC_MODE_TEST
    puts("Start test NFC Data...\n");

    ndef_data_text[0] = (sizeof(ndef_data_text) >> 8) & 0xFF;
    ndef_data_text[1] = sizeof(ndef_data_text) & 0xFF;
    uint16_t len = ((ndef_data_text[0] << 8) | ndef_data_text[1]);
    
    printf("NDEF size: %d\n", len );
    for(uint32_t i = 2; i < len; i++) {
        printf("%c", ndef_data_text[i]);
    }
    printf("\n");
    nfc_send_data(uid_test, NRF_NFC_UID_7_BYTES, NRF_NFC_PROTOCOL_TYPE4A_TAG, ndef_data_text, len);
#else
    puts("Start test NFC UID...\n");
    /* Set UID 7 bytes */
    nfc_set_uid(uid_test, NRF_NFC_UID_7_BYTES, NRF_NFC_PROTOCOL_TYPE2_TAG);
#endif    
    return 0;
}