/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup net_gnrc_lorawan
 * @{
 *
 * @file
 * @brief   GNRC LoRaWAN region specific functions
 *
 * @author  José Ignacio Alamos <jose.alamos@haw-hamburg.de>
 */
#ifndef NET_GNRC_LORAWAN_REGION_H
#define NET_GNRC_LORAWAN_REGION_H

#include "net/gnrc/lorawan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default LoRaWAN channels for supported regions
 */
 
typedef enum {
    GNRC_LORAWAN_REGION_EU868 = 0,
    GNRC_LORAWAN_REGION_RU864 = 1,
    GNRC_LORAWAN_REGION_KZ865 = 2,
} lorawan_regions_list_t;

typedef struct {
    uint32_t freq[3];
    uint32_t channels;
    uint32_t rx2_freq;
    uint8_t  tx_power[GNRC_LORAWAN_TXPOWER_NUMOF];
} lorawan_region_settings_t;

static const lorawan_region_settings_t gnrc_lorawan_region[] = {
    {
        /* EU868 */
        .freq = {868100000UL, 868300000UL, 868500000UL},
        .channels = 3,
        .rx2_freq = 869525000UL,
        .tx_power = {14, 12, 10, 8, 6, 4, 2, 0},
    },
    {
        /* RU864 */
        .freq = {868900000UL, 869100000UL, 0},
        .channels = 2,
        .rx2_freq = 869100000UL,
        .tx_power = {14, 12, 10, 8, 6, 4, 2, 0},
    },
    {
        /* KZ865 */
        .freq = {865100000UL, 865300000UL, 865500000UL},
        .channels = 3,
        .rx2_freq = 866700000UL,
        .tx_power = {14, 12, 10, 8, 6, 4, 2, 0},
    }
};

/**
 * @brief Process Channel Frequency list frame
 *
 * @param[in] mac pointer to the MAC descriptor
 * @param[in] cflist the CFList to be processed
 */
void gnrc_lorawan_process_cflist(gnrc_lorawan_t *mac, uint8_t *cflist);

/**
 * @brief Get the datarate of the first reception windows
 *
 * @param[in] dr_up the datarate of the transmission
 * @param[in] dr_offset the offset of the first reception window
 *
 * @return datarate
 */
uint8_t gnrc_lorawan_rx1_get_dr_offset(uint8_t dr_up, uint8_t dr_offset);

/**
 * @brief Check if a datarate is valid in the current region
 *
 * @param[in] dr the datarate to be checked
 *
 * @return true if datarate is valid
 * @return false otherwise
 */
int gnrc_lorawan_validate_dr(uint8_t dr);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_LORAWAN_REGION_H */
