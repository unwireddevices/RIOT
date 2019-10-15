/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

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
 * @file        ls-crypto.c
 * @brief       Cryptographic routines implementation
 * @author      Eugene Ponomarev
 */

#include <stdbool.h>

#include "random.h"
#include "assert.h"

#include "crypto/aes.h"
#include "crypto/ciphers.h"

#include "hashes/sha256.h"
#include "include/ls-crypto.h"

#include "byteorder.h"

#if !defined(UNWDS_MAC_LORAWAN)

typedef struct  __attribute__((packed)) {
    uint8_t fb;
    uint32_t u8_pad;
    uint8_t dir;
    le_uint32_t dev_addr;
    le_uint32_t fcnt;
    uint8_t u32_pad;
    uint8_t len;
} lorawan_block_t;

#ifdef __cplusplus
extern "C" {
#endif

ls_mic_t ls_calculate_mic(uint8_t *key, ls_frame_t *frame, uint8_t payload_size)
{
    /* Get pointer to the frame data after MIC field */
    uint8_t *ptr = ((uint8_t *) frame) + 4; /* Skip 1 byte of MHDR and 3 bytes of MIC */

    /* Calculate amount of data to check. Skip MHDR and MIC fields */
    uint8_t size = sizeof(ls_header_t) - 4 + sizeof(ls_payload_len_t) + payload_size;

    /* SHA-256 HMAC result */
    unsigned char hmac[SHA256_DIGEST_LENGTH];

    /* Calculate HMAC */
    hmac_sha256(key, LS_MIC_KEY_LEN, (unsigned *) ptr, size, hmac);

    /* Take first 3 bytes of hash as a MIC */
    ls_mic_t mic = (hmac[0] << 16)
                   | (hmac[1] << 8)
                   | (hmac[2]);

    return mic;
}

bool ls_validate_frame_mic(uint8_t *key, ls_frame_t *frame)
{
    /* Payload size is zero or matched to the AES block size + AES-CBC IV length */
    uint8_t payload_size = frame->payload.len;

    /* Compare MIC from header and actual */
    ls_mic_t expected_mic = ls_calculate_mic(key, frame, payload_size);
    ls_mic_t actual_mic = frame->header.mic;

    return actual_mic == expected_mic;
}

void ls_encrypt_frame(uint8_t *key_mic, uint8_t *key_aes, ls_frame_t *frame, size_t *newsize)
{
    *newsize = frame->payload.len;

    if (frame->payload.len > 0) {
        ls_encrypt_frame_payload(key_aes, frame);
    }
    else {
        *newsize = 0;
    }

    frame->header.mic = ls_calculate_mic(key_mic, frame, *newsize);
}

void ls_encrypt_frame_payload(uint8_t *key, ls_frame_t *frame)
{
	uint16_t size = frame->payload.len;

    if (size == 0) {
        return; /* Nothing to do with empty payload */
    }

    uint8_t s_block[AES_BLOCK_SIZE] = { 0x00, };

    lorawan_block_t a_block;
    uint16_t i;
    uint8_t buf_idx = 0;
    uint16_t ctr = 1;

    cipher_t context;

    cipher_init(&context, CIPHER_AES_128, key, AES_KEY_SIZE);

    a_block.fb = 0x1;
    a_block.u8_pad = 0;
    a_block.dir = frame->header.type;
    a_block.dev_addr = byteorder_btoll(byteorder_htonl(frame->header.dev_addr));
    a_block.fcnt = byteorder_btoll(byteorder_htonl(frame->header.fid));

    uint8_t *buffer = frame->payload.data;

    while (size >= AES_BLOCK_SIZE) {
        a_block.len = ((ctr) & 0xFF);
        ctr++;

        cipher_encrypt(&context, (uint8_t *) &a_block, s_block);
        for (i = 0; i < AES_BLOCK_SIZE; i++) {
            buffer[buf_idx + i] = buffer[buf_idx + i] ^ s_block[i];
        }

        size -= AES_BLOCK_SIZE;
        buf_idx += AES_BLOCK_SIZE;
    }

    if (size > 0) {
        a_block.len = ((ctr) & 0xFF);
        cipher_encrypt(&context, (uint8_t *) &a_block, s_block);
        for (i = 0; i < size; i++) {
            buffer[buf_idx + i] = buffer[buf_idx + i] ^ s_block[i];
        }
    }
}

inline void ls_decrypt_frame_payload(uint8_t *key, ls_frame_t *frame)
{
    ls_encrypt_frame_payload(key, frame);
}

void ls_derive_keys(ls_nonce_t dev_nonce, uint32_t app_nonce, ls_addr_t addr, uint8_t *key_mic, uint8_t *key_aes)
{
    assert(key_mic != NULL);

    /* Collect data into single buffer */
    uint32_t buf[3];
    buf[0] = dev_nonce;
    buf[1] = app_nonce;
    buf[2] = addr;

    /* Generate SHA-256 digest */
    uint8_t hash[SHA256_DIGEST_LENGTH];
    sha256((unsigned char *) buf, sizeof(buf), hash);

    /* First half of the hash is the MIC key */
    memcpy(key_mic, hash, 16);

    /* Second half is the AES key */
    if (key_aes != NULL) {
        memcpy(key_aes, hash + 16, 16);
    }
}

#endif

#ifdef __cplusplus
}
#endif
