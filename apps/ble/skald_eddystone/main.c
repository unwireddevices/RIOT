/*
 * Copyright (C) 2018 Freie Universität Berlin
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
 * @brief       BLE Eddystone beacon example using Skald
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include "stdio.h"

#include "net/skald/eddystone.h"
#include "periph/pm.h"
#include "periph/gpio.h"
#include "cpu.h"
#include "lptimer.h"

/* Example of an Eddystone URI:
 * - namespace (ASCII): 'supercool!'
 * - instance  (ASCII): `_RIOT_` */
#define URI_NAMESPACE   { 0x73, 0x75, 0x70, 0x65, 0x72, \
                          0x63, 0x6f, 0x6f, 0x6c, 0x21 }
#define URI_INSTANCE    { 0x5f, 0x52, 0x49, 0x4f, 0x54, 0x5f }

/* Advertise this short URL, points to https://www.unwireddevices.com/ */
#define URL             "unwds.com"

/* Calibrated TX power value */
#define TX_PWR          (0U)


/* Allocate two advertising contexts, one for Eddystone-URL and one for
 * Eddystone-URI */
// static skald_ctx_t _ctx_uid;
// static skald_ctx_t _ctx_url;
static skald_ctx_t _ctx_tlm;

int main(void)
{
    puts("Eddystone-TLM beacon test");
    puts("Every 10 secs it emits 3 to 4 packets with 100 ms interval");
    puts("Data and packets counter update interval 10 secs");

    /* Advertise the defined URI */
    // skald_eddystone_uid_t uid = { URI_NAMESPACE, URI_INSTANCE };
    // skald_eddystone_uid_adv(&_ctx_uid, &uid, TX_PWR);

    /* Also advertise the defined TLM */
    
    while(1)
    {
        cpu_update_status();
        printf("VBATT: %d, TEMP: %d\n", cpu_status.voltage.vdd, cpu_status.temp.core_temp);
        skald_eddystone_tlm_adv(&_ctx_tlm, 3300, cpu_status.temp.core_temp * 1000);
        gpio_set(LED0_PIN);
        lptimer_sleep(500);
        skald_adv_stop(&_ctx_tlm);
        gpio_clear(LED0_PIN);
        lptimer_sleep(10000);
    }

    return 0;
}
