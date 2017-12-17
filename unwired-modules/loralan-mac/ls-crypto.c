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
 * @file        ls-crypto.c
 * @brief       Implementation of cryptographic routines
 * @author      Eugene Ponomarev
 */

#include <stdbool.h>

#include "random.h"
#include "assert.h"

#include "crypto/aes.h"
#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"

#include "hashes/sha256.h"
#include "include/ls-crypto.h"

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
	uint8_t payload_size;
    payload_size = resize(frame->payload.len);
    if (frame->payload.len > LS_ECB_ENCRYPTION_MAX_SIZE) {
        payload_size += AES_BLOCK_SIZE;
    }

    /* Compare MIC from header and actual */
    ls_mic_t expected_mic = ls_calculate_mic(key, frame, payload_size);
    ls_mic_t actual_mic = frame->header.mic;

    return actual_mic == expected_mic;
}


static void generate_iv(uint8_t *iv)
{
    for (uint32_t i = 0; i < AES_BLOCK_SIZE; i++) {
        iv[i] = random_uint32_range(0, 255);
    }
}

int ls_encrypt_frame_payload(uint8_t *key, ls_payload_t *payload)
{
    if (payload->len == 0) {
        return 0; /* Nothing to do with empty payload */
    }
	
    uint8_t size = resize(payload->len);
	/* Get pointer to the payload buffer which starts after the payload length byte */
    uint8_t *ptr = payload->data;
	
	/* Buffer for the payload ciphertext */
	uint8_t bufout[LS_PAYLOAD_SIZE_MAX];
	memset(bufout, 0, LS_PAYLOAD_SIZE_MAX);
	
	if (payload->len <= LS_ECB_ENCRYPTION_MAX_SIZE) {
		/* Add some salt */
		uint32_t i;
		for (i = payload->len; i < AES_BLOCK_SIZE; i++) {
			ptr[i] = random_uint32_range(0, 255);
		}
				
		/* Encrypt payload */
		cipher_context_t context;
		aes_init(&context, key, AES_KEY_SIZE);
		aes_encrypt(&context, ptr, bufout);
	}
	else {
		/* Generate random AES-CBC initialization vector */
		uint8_t iv[AES_BLOCK_SIZE];
		generate_iv(iv);

		/* Write IV to the end of the payload buffer */
		memcpy(ptr + size, iv, AES_BLOCK_SIZE);

		/* Encrypt payload */
		aes_cbc_encrypt(ptr, size, key, iv, bufout);
	}
	
	/* Copy the ciphertext back into the payload buffer */
	memcpy(payload->data, bufout, size);

    return size;
}

void ls_decrypt_frame_payload(uint8_t *key, ls_payload_t *payload)
{
    if (payload->len == 0) {
        return; /* Nothing to do with empty payload */
    }
	
    /* Payload is guaranteed to be matched to the AES block size */
    uint8_t size = resize(payload->len);

    /* Buffer for the payload ciphertext */
    uint8_t buf[LS_PAYLOAD_SIZE_MAX];
    memset(buf, 0, LS_PAYLOAD_SIZE_MAX);
    memcpy(buf, payload->data, size); /* Copy payload ciphertext into the buffer */

    /* Buffer for the payload cleartext */
    uint8_t bufout[LS_PAYLOAD_SIZE_MAX];
    memset(bufout, 0, LS_PAYLOAD_SIZE_MAX);

	if (payload->len <= LS_ECB_ENCRYPTION_MAX_SIZE) {
		/* Decrypt payload */
		cipher_context_t context;
		aes_init(&context, key, AES_KEY_SIZE);
		aes_decrypt(&context, buf, bufout);
	} else {
		/* Extract IV from the buffer */
		uint8_t iv[AES_BLOCK_SIZE];
		memcpy(iv, payload->data + size, AES_BLOCK_SIZE);

		/* Decipher the payload */
		aes_cbc_decrypt(buf, size, key, iv, bufout);
	}

    /* Copy payload cleartext back to the payload buffer */
    memcpy(payload->data, bufout, payload->len);
}

void ls_encrypt_frame(uint8_t *key_mic, uint8_t *key_aes, ls_frame_t *frame, size_t *newsize) {
    *newsize = ls_encrypt_frame_payload(key_aes, &frame->payload);
    if (frame->payload.len > LS_ECB_ENCRYPTION_MAX_SIZE) {
        /* AES-CBC initialization vector block is added */
        *newsize += AES_BLOCK_SIZE;
    }
    
	frame->header.mic = ls_calculate_mic(key_mic, frame, *newsize);
}

void ls_derive_keys(ls_nonce_t dev_nonce, uint32_t app_nonce, ls_addr_t addr, uint8_t *key_mic, uint8_t *key_aes) {
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

#ifdef __cplusplus
}
#endif
