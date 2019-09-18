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
 * @file        ls-settings.c
 * @brief       Common LoRaLAN device settings
 * @author      Oleg Artamonov
 */
#ifndef UNWIRED_MODULES_LORA_STAR_SETTINGS_H_
#define UNWIRED_MODULES_LORA_STAR_SETTINGS_H_

#include <stdbool.h>

#include "ls-mac-types.h"
#include "unwds-common.h"

#define UNWDS_LS_SETTINGS_CONFIG_VERSION 0x4

#define UNWDS_MAX_MODULES       256
#define UNWDS_MAX_MODULES_BYTES (UNWDS_MAX_MODULES/8)

/* Max size is ROLE_CONFIG_SIZE = 128 bytes */
typedef struct {
    uint8_t         config_version;
    uint8_t         is_config_valid;    /**< 1 if config is valid */
    uint8_t         region_index;       /**< Selected channels region index */
    ls_channel_t    channel;            /**< Radio channel */
    ls_datarate_t   dr;                 /**< Default data rate */
    ls_node_class_t nodeclass;          /**< Default node class */
    uint8_t         max_retr;           /**< Attempts to retry if ack was not received / join unsuccessful */
    bool            no_join;            /**< Statically personalized device, no join required to send data */
    bool            req_time;           /**< Request time after join */
    ls_addr_t       dev_addr;           /**< Predefined device's network address */
    uint8_t         enabled_mods[UNWDS_MAX_MODULES_BYTES];    /**< Defines ability mask - list of enabled UNWDS modules */
    bool            confirmation;       /**< Require confirmation when sending packets */
    bool            adr;                /**< Enable Adaptive Data Rate */
} node_role_settings_t;

node_role_settings_t unwds_get_node_settings(void);
void unwds_set_region(int region);
void unwds_set_nojoin(bool nojoin);
void unwds_set_addr(ls_addr_t dev_addr);
void unwds_set_channel(ls_channel_t channel);
void unwds_set_dr(ls_datarate_t dr);
void unwds_set_max_retr(uint8_t max_retr);
void unwds_set_adr(bool adr);
void unwds_set_class(ls_node_class_t nodeclass);
void unwds_set_module(uint8_t modid, bool enable);
void unwds_set_cnf(bool confirmation);

int unwds_config_save(void);
bool unwds_config_load(void);
void unwds_config_reset(void);

#endif /* UNWIRED_MODULES_LORA_STAR_SETTINGS_H_ */
