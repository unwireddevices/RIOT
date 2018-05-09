/*
 * Copyright (C) 2017-2018 Unwired Devices [info@unwds.com]
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
 * @file        ls-settings.c
 * @brief       Common LoRaLAN device settings
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ls-settings.h"
#include "ls-config.h"

#include "unwds-common.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static node_role_settings_t node_settings;

/* gateway before ver. 1.63 settings structure */
typedef struct {
    bool is_valid;
    ls_channel_t channel;
    ls_datarate_t dr;
    bool region_not_set;
    uint8_t region_index;
} old_role_settings_t;

static old_role_settings_t old_node_settings;

node_role_settings_t unwds_get_node_settings(void) {
    return node_settings;
}

void unwds_set_region(int region) {
    node_settings.region_index = region;
    return;
}

void unwds_set_nojoin(bool nojoin) {
    node_settings.no_join = nojoin;
    return;
}

void unwds_set_channel(ls_channel_t channel) {
    node_settings.channel = channel;
    return;
}

void unwds_set_dr(ls_datarate_t dr) {
    node_settings.dr = dr;
    return;
}

void unwds_set_max_retr(uint8_t max_retr) {
    node_settings.max_retr = max_retr;
    return;
}

void unwds_set_class(ls_node_class_t nodeclass) {
    node_settings.nodeclass = nodeclass;
    return;
}

void unwds_set_addr(ls_addr_t dev_addr) {
    node_settings.dev_addr = dev_addr;
    return;
}

void unwds_set_module(uint8_t modid, bool enable) {
    
    if (unwds_is_module_exists(modid))
    {    
        uint8_t index = modid / 32;
        uint32_t mask = (uint32_t) (1 << (modid % 32));
        
        if (enable) {
            /* Enable module */
            node_settings.enabled_mods[index] |= mask;
            printf("mod: %s [%d] enabled. Save and reboot to apply changes\n", unwds_get_module_name(modid), modid);
        }
        else {
            /* Disable module */
            node_settings.enabled_mods[index] &= ~(mask);
            printf("mod: %s [%d] disabled. Save and reboot to apply changes\n", unwds_get_module_name(modid), modid);
        }
    }
}

int unwds_config_save(void)
{  
    node_settings.is_config_valid = 1;
    node_settings.config_version = UNWDS_LS_SETTINGS_CONFIG_VERSION;
    return config_write_role_block((uint8_t *) &node_settings, sizeof(node_role_settings_t));
}

void unwds_config_reset(void) {
    puts("[node] Node was not configured properly, setting default values");
    
    node_settings.region_index = 1;
    node_settings.channel = 0;
    node_settings.dr = LS_DR0;
    node_settings.max_retr = 3;
    node_settings.nodeclass = LS_ED_CLASS_C;
    
    memset((void *)node_settings.enabled_mods, 0, sizeof(node_settings.enabled_mods));

    node_settings.no_join = 0;
    node_settings.dev_addr = 0; 
    
    /* Modules enabled by default */
    unwds_set_module(UNWDS_CONFIG_MODULE_ID, true);
    
    puts("Saving node configuration to EEPROM...");
    unwds_config_save();
    puts("Done.");
}

bool unwds_config_load(void)
{
    if (!config_read_role_block((uint8_t *) &node_settings, sizeof(node_role_settings_t))) {
        puts("[node] Unable to load role specific configuration");

        return false;
    }
    
    if (node_settings.config_version < UNWDS_LS_SETTINGS_CONFIG_VERSION) {
        config_read_role_block((uint8_t *) &old_node_settings, sizeof(old_role_settings_t));
        if (old_node_settings.is_valid == 1) {
            puts("[node] Converting gateway configuration to a new format");
            
            node_settings.channel = old_node_settings.channel;
            node_settings.dr = old_node_settings.dr;
            node_settings.region_index = old_node_settings.region_index;
            
            puts("Saving new gateway configuration to EEPROM...");
            unwds_config_save();
            puts("Done.");
        }
    }
    
    if (node_settings.is_config_valid != 1) {
        /* Reset to default settings */
        unwds_config_reset();
    }

    return node_settings.is_config_valid;
}

#ifdef __cplusplus
}
#endif
