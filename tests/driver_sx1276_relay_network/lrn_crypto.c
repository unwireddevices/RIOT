/*
 * Copyright (C) 2016 Unwired Devices
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
 * @file        lrn_crypto.c
 * @brief       Implementation of cryptographic routines
 * @author      Unwired Devices
 */

#include "random.h"

#include "crypto/aes.h"
#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"

#include "hashes/sha256.h"

#include "lrn_crypto.h"
#include "lrn.h"

#ifdef __cplusplus
extern "C" {
#endif

void aes_cbc_decrypt(uint8_t *buf, uint8_t buflen, uint8_t key[AES_KEY_SIZE], uint8_t iv[AES_BLOCK_SIZE], uint8_t *out)
{
    cipher_t cipher;

    cipher_init(&cipher, CIPHER_AES_128, key, AES_KEY_SIZE);
    cipher_decrypt_cbc(&cipher, iv, buf, buflen, out);
}

void aes_cbc_encrypt(uint8_t *buf, uint8_t buflen, uint8_t key[AES_KEY_SIZE], uint8_t iv[AES_BLOCK_SIZE], uint8_t *out)
{
    cipher_t cipher;

    cipher_init(&cipher, CIPHER_AES_128, key, AES_KEY_SIZE);
    cipher_encrypt_cbc(&cipher, iv, buf, buflen, out);
}

static inline int resize(uint8_t size)
{
    if (size % AES_BLOCK_SIZE != 0) {
        size += (AES_BLOCK_SIZE - (size % AES_BLOCK_SIZE));
    }

    return size;
}

lrn_mic_t calculate_mic(lrn_t *lrn, lrn_frame_t *frame, uint8_t payload_size)
{
    /* Get pointer to the frame data after MIC field */
    uint8_t *ptr = ((uint8_t *) frame) + 4; /* Skip 1 byte of MHDR and 3 bytes of MIC */

    /* Calculate amount of data to check. Skip MHDR and MIC fields */
    uint8_t size = sizeof(lrn_header_t) - 4 + sizeof(lrn_payload_len_t) + payload_size;

    /* SHA-256 HMAC result */
    unsigned char hmac[SHA256_DIGEST_LENGTH];

    /* Calculate HMAC */
    hmac_sha256(lrn->crypto_key, sizeof(lrn->crypto_key), (unsigned *) ptr, size, hmac);

    /* Take first 3 bytes of hash as a MIC */
    lrn_mic_t mic = (hmac[0] << 16)
                    | (hmac[1] << 8)
                    | (hmac[2]);

    return mic;
}

bool validate_frame_mic(lrn_t *lrn, lrn_frame_t *frame)
{
    /* Payload size is zero or matched to the AES block size + AES-CBC IV length */
    uint8_t payload_size = (frame->payload.len == 0) ? 0 : resize(frame->payload.len) + AES_BLOCK_SIZE;

    /* Compare MIC from header and actual */
    lrn_mic_t expected_mic = calculate_mic(lrn, frame, payload_size);
    lrn_mic_t actual_mic = frame->header.mic;

    return actual_mic == expected_mic;
}


static void generate_iv(uint8_t *iv)
{
    for (int i = 0; i < AES_BLOCK_SIZE; i++) {
        iv[i] = random_uint32_range(0, 255);
    }
}

int encrypt_frame_payload(lrn_t *lrn, lrn_payload_t *payload)
{
    if (payload->len == 0) {
        return 0; /* Nothing to do with empty payload */

    }
    uint8_t size = resize(payload->len);

    /* Get pointer to the payload buffer which starts after the payload length byte */
    uint8_t *ptr = payload->data;

    /* Generate random AES-CBC initialization vector */
    uint8_t iv[AES_BLOCK_SIZE];
    generate_iv(iv);

    /* Write IV to the end of the payload buffer */
    memcpy(ptr + size, iv, AES_BLOCK_SIZE);

    /* Buffer for the payload ciphertext */
    uint8_t bufout[LRN_PAYLOAD_SIZE_MAX];
    memset(bufout, 0, LRN_PAYLOAD_SIZE_MAX);

    /* Encrypt payload */
    aes_cbc_encrypt(ptr, size, lrn->crypto_key, iv, bufout);

    /* Copy the ciphertext back into the payload buffer */
    memcpy(payload->data, bufout, size);

    return size;
}

void decrypt_frame_payload(lrn_t *lrn, lrn_payload_t *payload)
{
    if (payload->len == 0) {
        return; /* Nothing to do with empty payload */

    }
    /* Payload is guaranteed to be matched to the AES block size */
    uint8_t size = resize(payload->len);

    /* Buffer for the payload ciphertext */
    uint8_t buf[LRN_PAYLOAD_SIZE_MAX];
    memset(buf, 0, LRN_PAYLOAD_SIZE_MAX);
    memcpy(buf, payload->data, size); /* Copy payload ciphertext into the buffer */

    /* Buffer for the payload cleartext */
    uint8_t bufout[LRN_PAYLOAD_SIZE_MAX];
    memset(bufout, 0, LRN_PAYLOAD_SIZE_MAX);

    /* Extract IV from the buffer */
    uint8_t iv[AES_BLOCK_SIZE];
    memcpy(iv, payload->data + size, AES_BLOCK_SIZE);

    /* Decipher the payload */
    aes_cbc_decrypt(buf, size, lrn->crypto_key, iv, bufout);

    /* Copy payload cleartext back to the payload buffer */
    memcpy(payload->data, bufout, payload->len);
}

#ifdef __cplusplus
}
#endif
