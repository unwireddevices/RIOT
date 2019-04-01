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
 * @file        ls-config.c
 * @brief       Common LoRaLAN device runtime configuration functions
 * @author      Evgeniy Ponomarev
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "periph/eeprom.h"
#include "checksum/crc16_ccitt.h"

#include "ls-config.h"

static nvram_config_t config;
static bool config_valid = false;

static bool key_valid = false;

static config_eui64_t eui64;
static bool eui64_valid = false;

uint16_t get_crc(uint8_t *buf, size_t size)
{
    return crc16_ccitt_calc(buf, size);
}

bool check_crc_config(nvram_config_t *cfg, uint32_t crc)
{
    uint16_t actual_crc = get_crc((uint8_t *) cfg, CONFIG_SIZE - 4);

    return actual_crc == crc;
}

bool check_crc_eui64(config_eui64_t *cfg, uint32_t crc)
{
    uint16_t actual_crc = get_crc((uint8_t *) cfg, CONFIG_EUI64_SIZE);

    return actual_crc == crc;
}

void config_reset_nvram(void)
{
    memset(&config, 0, sizeof(nvram_config_t));

    config_valid = false;

    save_config_nvram();
}

bool load_config_nvram(void)
{
    nvram_config_t temp_config;
    memset(&temp_config, 0, sizeof(nvram_config_t));

    if (eeprom_read(CONFIG_ADDR, (uint8_t *)&temp_config, CONFIG_SIZE)) {
        /* Check CRC */
        if (!check_crc_config(&temp_config, temp_config.cfg_crc)) {
            /* let's check if it's an old config version */
            
            nvram_old_config_t old_config;
            if (eeprom_read(CONFIG_ADDR, (uint8_t *) &old_config, OLD_CONFIG_SIZE)) {
                uint16_t actual_crc = get_crc((uint8_t *) &old_config, OLD_CONFIG_SIZE - 4);
                
                if (actual_crc != old_config.cfg_crc) {
                    puts("CRC is wrong");
                    return false;
                } else {
                    puts ("Converting old config to a new one");
                    config.appid64 = old_config.appid64;
                    config.dev_nonce = old_config.dev_nonce;

                    memcpy(&config.nwk_key, &old_config.nwk_key, sizeof(config.nwk_key));
                    
                    save_config_nvram();
                }
            }
        } else {
            memcpy(&config, &temp_config, sizeof(nvram_config_t));
        }
		
		for (uint32_t i=0; i<16; i++) {
			if (config.nwk_key[i] > 0) {
				key_valid = true;
			}
		}
		
        config_valid = true;

        return true;
    }

    return false;
}

bool save_eui64_nvram(void)
{
    /* Calculate checksum */
    eui64.crc = get_crc((uint8_t *) &eui64, CONFIG_EUI64_SIZE);

    /* Write to NVRAM */
    return (eeprom_write(CONFIG_EUI64_ADDR, (uint8_t *) &eui64, sizeof(config_eui64_t)) > 0);
}

bool save_config_nvram(void)
{
    /* Calculate checksum excluding old CRC field at the end*/
    config.cfg_crc = get_crc((uint8_t *) &config, CONFIG_SIZE - 4);

    /* Write to NVRAM */
    return (eeprom_write(CONFIG_ADDR, (uint8_t *) &config, sizeof(nvram_config_t)) > 0);
}

bool clear_nvram(void)
{
    eeprom_erase();
    
    return true;
}

bool clear_nvram_modules(int modid)
{
    if (modid == 0) {
        /* to fix later: clear till last module or end of EEPROM */
        eeprom_clear(UNWDS_CONFIG_BASE_ADDR, 50*UNWDS_CONFIG_BLOCK_SIZE_BYTES);
    }
    
    return true;
}

config_role_t config_get_role(void)
{
	if (!eui64_valid) {
        return ROLE_NO_EUI64;
    }

    if (!config_valid) {
    	return ROLE_NO_CFG;
    }

	if (!key_valid) {
		return ROLE_EMPTY_KEY;
	}

    if (config_valid) {
        return ROLE_NORMAL;
    }

    return ROLE_NO_CFG;
}

bool config_write_main_block(uint64_t appid64, uint8_t appkey[16], uint32_t devnonce)
{
	config.dev_nonce = devnonce;
    config.appid64 = appid64;
    memcpy(config.nwk_key, appkey, 16);

    return save_config_nvram();
}

bool load_eui64_nvram(void)
{
    config_eui64_t temp_eui64;

    if (eeprom_read(CONFIG_EUI64_ADDR, (uint8_t *) &temp_eui64, sizeof(config_eui64_t))) {
        /* Check CRC */
        if (!check_crc_eui64(&temp_eui64, temp_eui64.crc)) {
            puts("EUI64 CRC check failed");
            return false;
        }

        memcpy(&eui64, &temp_eui64, sizeof(config_eui64_t));
        eui64_valid = true;

        return true;
    }

    return false;
}

bool write_eui64_nvram(uint64_t eui)
{
    eui64.eui64 = eui;

    return save_eui64_nvram();
}

uint64_t config_get_nodeid(void)
{
    return eui64.eui64;
}

uint64_t config_get_appid(void)
{
    return config.appid64;
}

uint8_t *config_get_appkey(void)
{
    return config.nwk_key;
}

uint8_t *config_get_appskey(void)
{
    return config.apps_key;
}

uint8_t *config_get_nwkskey(void)
{
    return config.nwks_key;
}

void config_set_appskey(uint8_t *appskey)
{
    memcpy(config.apps_key, appskey, sizeof(config.apps_key));
}

void config_set_nwkskey(uint8_t *nwkskey)
{
    memcpy(config.nwks_key, nwkskey, sizeof(config.nwks_key));
}

uint32_t config_get_devnonce(void)
{
	return config.dev_nonce;
}

bool config_write_role_block(uint8_t *buf, size_t size)
{
    if (size > ROLE_CONFIG_SIZE) {
        return false;
    }

    memcpy(config.role_config, buf, size);

    return save_config_nvram();
}

bool config_read_role_block(uint8_t *buf, size_t size)
{
    if (size > ROLE_CONFIG_SIZE) {
        return false;
    }

    memcpy(buf, config.role_config, size);
    return true;
}

#ifdef __cplusplus
}
#endif
