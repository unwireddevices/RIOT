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
 * @file        ls_crypto.h
 * @brief       Definitions for cryptographic routines
 * @author      Eugene Ponomarev
 */
#ifndef LS_CRYPTO_H_
#define LS_CRYPTO_H_

#include "crypto/aes.h"
#include "ls-mac-types.h"

#define LS_MIC_KEY_LEN AES_KEY_SIZE

#define LS_ECB_ENCRYPTION_MAX_SIZE 16

/**
 * @brief Cryptography settings for the device.
 */
typedef struct {
	uint8_t mic_key[LS_MIC_KEY_LEN];
	uint8_t aes_key[AES_KEY_SIZE];

	uint8_t join_key[AES_KEY_SIZE];
} ls_crypto_t;

/**
 * @brief Calculates Message Integrity Code for the specified frame
 *
 * @param	[IN]	ls		the ls stack state
 * @param	[IN]	frame	frame for which the MIC will be calculated
 *
 * @return MIC for the specified frame
 */
ls_mic_t ls_calculate_mic(uint8_t *key, ls_frame_t *frame, uint8_t payload_size);

/**
 * @brief Validates Message Integrity Code for the specified frame
 *
 * @param	[IN]	ls		the ls stack state including cryptographic keys
 * @param	[IN]	frame	frame for which the MIC will be validated
 *
 * @return true if MIC is valid, false otherwise
 */
bool ls_validate_frame_mic(uint8_t *key, ls_frame_t *frame);

/**
 * @brief Encrypts payload of the specified frame in respect to ls stack state.
 *
 * @param	[IN]	ls		the ls stack state including cryptographic keys
 * @param	[IN]	payload	pointer to the payload that will be modified to the encrypted one
 *
 * @return new size of the payload buffer
 */
int ls_encrypt_frame_payload(uint8_t *key, ls_payload_t *payload);

/**
 * @brief Decrypts the payload of the specified frame in respect to ls stack state.
 *
 * @param	[IN]	ls		the ls stack state including cryptographic keys
 * @param	[IN]	payload	pointer to the payload that will be decrypted
 */
void ls_decrypt_frame_payload(uint8_t *key, ls_payload_t *payload);

/**
 * @brief Encrypts frame payload and calculates frame's MIC
 *
 * @param	[IN]	*lsc		the ls stack state including cryptographic keys
 * @param	[IN]	*frame		the frame to work with
 * @param	[OUT]	*newsize	new size of payload (resizes after encryption)
 */
void ls_encrypt_frame(uint8_t *key_mic, uint8_t *key_aes, ls_frame_t *frame, size_t *newsize);

/**
 * @brief Derives keys from the nonce numbers
 *
 * @param	[IN]	dev_nonce	the device nonce number
 * @param	[IN]	app_nonce	the application nonce number
 * @param	[IN]	addr		device address
 * @param	[OUT]	*key_mic	key for the MIC calculation
 * @param	[OUT]	*key_aes	key for the AES encryption
 */
void ls_derive_keys(uint32_t dev_nonce, uint32_t app_nonce, ls_addr_t addr, uint8_t *key_mic, uint8_t *key_aes);

#endif /* LS_CRYPTO_H_ */
