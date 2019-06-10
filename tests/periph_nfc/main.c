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

static uint8_t uid_test[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0xAA, 0xBB, 0xCC };
// static uint8_t data[10] = {0x00, 0x80, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0A, 0x0B, 0x0C };

// static uint8_t NDEF_RECORD[] = { 0xD1,                                              // MB / ME / CF / 1 / IL / TNF
                             // 0x01,                                              // TYPE LENGTH
                             // 51,                                                // PAYLOAD LENTGH
                             // 'T',                                               // TYPE
                             // 0x02,                                              // Status
                             // 'e', 'n',                                          // Language
// 13, 10, 13, 10, 32, 32, 32, 32, 32, 
// 'M', 'i', 'k', 'r', 'o', 'E', 'l', 'e', 'k', 't', 'r', 'o', 'n', 'i', 'k', 'a', 
// 13, 10, 13, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
// 'N', 'F', 'C', ' ', 'c', 'l', 'i', 'c', 'k'
// };

static uint8_t NDEF_RECORD[] = { 0xD1,                                              // MB / ME / CF / 1 / IL / TNF
                             0x01,                                              // TYPE LENGTH
                             51,                                                // PAYLOAD LENTGH
                             0x00,                                               // TYPE
                             0x02,                                              // Status
13, 10, 13, 10, 32, 32, 32, 32, 32, 
'M', 'i', 'k', 'r', 'o', 'E', 'l', 'e', 'k', 't', 'r', 'o', 'n', 'i', 'k', 'a', 
13, 10, 13, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
'N', 'F', 'C', ' ', 'c', 'l', 'i', 'c', 'k'
};

int main(void)
{   
    puts("Start test NFC UID...\n");
    nfc_init();

    // /* Set UID 7 bytes */
    nfc_set_uid(uid_test, NFC_UID_7_BYTES, NFC_PROTOCOL_TYPE4A_TAG);
    puts("Start test NFC Data...\n");
    // /* delay between changing */
    xtimer_sleep(10);
    nfc_send_data(uid_test, NFC_UID_7_BYTES, NFC_PROTOCOL_TYPE4A_TAG, NDEF_RECORD, 30);
    
    // /* Change UID to 10 bytes */
    // nfc_set_uid(uid_test1, NFC_UID_7_BYTES, NFC_PROTOCOL_TYPE2_TAG);

    // /* delay between changing */
    // xtimer_sleep(10);
    // /* Change UID to 4 bytes */
    // nfc_set_uid(uid_test, NFC_UID_10_BYTES, NFC_PROTOCOL_TYPE2_TAG);
    
    return 0;
}