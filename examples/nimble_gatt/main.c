/*
 * Copyright (C) 2018 Freie Universität Berlin
 *               2018 Codecoup
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       BLE peripheral example using NimBLE
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Andrzej Kaczmarek <andrzej.kaczmarek@codecoup.pl>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "nimble_riot.h"
#include "net/bluetil/ad.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *device_name = "NimBLE on RIOT";

static void start_advertise(void);

static void update_ad(void)
{
    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    bluetil_ad_t ad;
    bluetil_ad_init_with_flags(&ad, buf, sizeof(buf), BLUETIL_AD_FLAGS_DEFAULT);
    bluetil_ad_add_name(&ad, device_name);
    ble_gap_adv_set_data(ad.buf, ad.pos);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status) {
                start_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            start_advertise();
            break;
    }

    return 0;
}

static void start_advertise(void)
{
    struct ble_gap_adv_params advp;
    int rc;

    memset(&advp, 0, sizeof advp);
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(nimble_riot_own_addr_type, NULL, BLE_HS_FOREVER,
                           &advp, gap_event_cb, NULL);
    assert(rc == 0);
    (void)rc;
}

int main(void)
{
    puts("NimBLE GATT Server Example");

    int rc = 0;

    /* verify and add our custom services */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    /* set the device name */
    ble_svc_gap_device_name_set(device_name);
    /* reload the GATT server to link our added services */
    ble_gatts_start();

    /* configure and set the advertising data */
    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    bluetil_ad_t ad;
    bluetil_ad_init_with_flags(&ad, buf, sizeof(buf), BLUETIL_AD_FLAGS_DEFAULT);
    bluetil_ad_add_name(&ad, device_name);
    ble_gap_adv_set_data(ad.buf, ad.pos);

    /* start to advertise this node */
    start_advertise();

    return 0;
}
