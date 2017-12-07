/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file
 * @brief       
 * @author      Evgeniy Ponomarev
 */
#ifndef LORA_STAR_UNI_CONFIG_H_
#define LORA_STAR_UNI_CONFIG_H_

#include "nvram.h"
#include <stdint.h>
#include <stdbool.h>

#define CONFIG_EUI64_ADDR 0x00
#define CONFIG_EUI64_SIZE sizeof(uint64_t)

#define CONFIG_ADDR (4 + 8)
#define CONFIG_FORMAT_VER 0x01

typedef enum {
	ROLE_NO_EUI64 = 0,
	ROLE_NO_CFG,
    ROLE_EMPTY_KEY,
	ROLE_NORMAL,
} config_role_t;

#define CONFIG_MAGIC 0xCAFEBABE
#define ROLE_CONFIG_SIZE 128

typedef struct {
	uint64_t eui64;
	uint32_t crc;
} config_eui64_t;

typedef struct {
	uint32_t magic;							/**< Structure magic */
	uint8_t version;						/**< Structure version */
	uint64_t appid64;						/**< Application ID */
	uint8_t nwk_key[16];					/**< Network AES-128 key */
	uint8_t role_config[ROLE_CONFIG_SIZE];	/**< Role specific configuration block */
    uint32_t dev_nonce;
	uint32_t cfg_crc;						/**< Configuration's CRC block */
} nvram_config_t;

typedef struct {
	uint32_t magic;							/**< Structure magic */
	uint8_t version;						/**< Structure version */
	uint64_t appid64;						/**< Application ID */
	uint8_t nwk_key[16];					/**< Network AES-128 key */
	uint8_t role_config[ROLE_CONFIG_SIZE];	/**< Role specific configuration block */
	uint32_t cfg_crc;						/**< Configuration's CRC block */
} nvram_old_config_t;

#define CONFIG_SIZE (sizeof(nvram_config_t))

/* CONFIG_ADDR + CONFIG_SIZE + 4 must be < 256 */

#define UNWDS_CONFIG_BASE_ADDR (256)
#define UNWDS_CONFIG_BLOCK_SIZE_BYTES (24)

bool load_eui64_nvram(nvram_t *nvram);
bool write_eui64_nvram(uint64_t eui);

bool save_config_nvram(nvram_t *nvram);
bool load_config_nvram(nvram_t *nvram);
bool clear_nvram(void);
bool clear_nvram_modules(int modid);
void config_reset_nvram(nvram_t *nvram);
config_role_t config_get_role(void);

/* Device specific settings */
bool config_write_main_block(uint64_t appid64, uint8_t joinkey[16], uint32_t devnonce);
uint64_t config_get_nodeid(void);
uint64_t config_get_appid(void);
uint8_t *config_get_joinkey(void);
uint32_t config_get_devnonce(void);

/* Role specific settings */
bool config_write_role_block(uint8_t *buf, size_t size);
bool config_read_role_block(uint8_t *buf, size_t size);

nvram_t *config_get_nvram(void);

#endif /* LORA_STAR_UNI_CONFIG_H_ */
