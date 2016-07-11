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
 * @file        lrn_crypto.h
 * @brief       Definitions for cryptographic routines
 * @author      Unwired Devices
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_CRYPTO_LRN_CRYPTO_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_CRYPTO_LRN_CRYPTO_H_

#include "crypto/aes.h"

#include "lrn.h"
#include "lrn_mac.h"

/**
 * @brief Calculates Message Integrity Code for the specified frame
 *
 * @param	[IN]	lrn		the LRN stack state
 * @param	[IN]	frame	frame for which the MIC will be calculated
 *
 * @return MIC for the specified frame
 */
lrn_mic_t calculate_mic(lrn_t *lrn, lrn_frame_t *frame, uint8_t payload_size);

/**
 * @brief Validates Message Integrity Code for the specified frame
 *
 * @param	[IN]	lrn		the LRN stack state including cryptographic keys
 * @param	[IN]	frame	frame for which the MIC will be validated
 *
 * @return true if MIC is valid, false otherwise
 */
bool validate_frame_mic(lrn_t *lrn, lrn_frame_t *frame);

/**
 * @brief Encrypts payload of the specified frame in respect to LRN stack state.
 *
 * @param	[IN]	lrn		the LRN stack state including cryptographic keys
 * @param	[IN]	payload	pointer to the payload that will be modified to the encrypted one
 *
 * @return new size of the payload buffer
 */
int encrypt_frame_payload(lrn_t *lrn, lrn_payload_t *payload);

/**
 * @brief Decrypts the payload of the specified frame in respect to LRN stack state.
 *
 * @param	[IN]	lrn		the LRN stack state including cryptographic keys
 * @param	[IN]	payload	pointer to the payload that will be decrypted
 */
void decrypt_frame_payload(lrn_t *lrn, lrn_payload_t *payload);

void aes_cbc_decrypt(uint8_t* buf, uint8_t buflen, uint8_t key[AES_KEY_SIZE], uint8_t iv[AES_BLOCK_SIZE], uint8_t *out);
void aes_cbc_encrypt(uint8_t* buf, uint8_t buflen, uint8_t key[AES_KEY_SIZE], uint8_t iv[AES_BLOCK_SIZE], uint8_t *out);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_CRYPTO_LRN_CRYPTO_H_ */
