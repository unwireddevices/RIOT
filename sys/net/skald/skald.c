/*
 * Copyright (C) 2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_skald
 * @{
 *
 * @file
 * @brief       Skald's link layer implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 * @}
 */

#include <stdint.h>

#include "assert.h"
#include "random.h"
#include "luid.h"

#include "net/netdev/ble.h"
#include "net/skald.h"
#include "periph/rtt.h"

/* Include fitting radio driver */
#ifdef MODULE_NRFBLE
    #include "nrfble.h"
    /* Add other BLE radio drivers once implemented - and potentially move to
    *  auto-init at some point */
#else
    #error "[skald] error: unable to find any netdev-ble capable radio"
#endif /* MODULE_NRFBLE */

#define ENABLE_DEBUG            (0)
#include "debug.h"

#define JITTER_MIN              (0U)                /* 0ms */
#define JITTER_MAX              (10000U)            /* 10ms */

#define ADV_CHAN_NUMOF          sizeof(_adv_chan)   /* Number of cannals */
#define ADV_AA                  (0x8e89bed6)        /* Access address */
#define ADV_CRC                 (0x00555555)        /* CRC initializer */

static const uint8_t _adv_chan[] = SKALD_ADV_CHAN;

static netdev_ble_ctx_t _ble_ctx = {
    .aa.u32 = ADV_AA,
    .crc = ADV_CRC,
};

static netdev_t *_radio;

/* Function prototypes */
static void _stop_radio(void);
static void _sched_next(skald_ctx_t *ctx);
static void _on_adv_evt(void *arg);
static void _on_radio_evt(netdev_t *netdev, netdev_event_t event);

static void _stop_radio(void)
{
    netdev_ble_stop(_radio);
    _radio->context = NULL;
}

static void _sched_next(skald_ctx_t *ctx)
{
#ifndef SKALD_ON_RTT
    ctx->last += SKALD_INTERVAL;
#else
    ctx->last += (SKALD_INTERVAL / 1000);
#endif /* SKALD_ON_RTT */

    /* Schedule next advertising event, adding a random jitter between
     * 0ms and 10ms (see spec v5.0-vol6-b-4.4.2.2.1) */
#ifndef SKALD_ON_RTT
    ctx->last += random_uint32_range(JITTER_MIN, JITTER_MAX);
#else
    ctx->last += random_uint32_range(JITTER_MIN, (JITTER_MAX / 1000));
    ctx->last &= RTT_MAX_VALUE; 
#endif /* SKALD_ON_RTT */

    /* Compensate the time passed since the timer triggered last by using the
     * current value of the timer */
#ifndef SKALD_ON_RTT
    xtimer_set(&ctx->timer, (ctx->last - xtimer_now_usec())); 
#else
    rtt_set_alarm(ctx->last, _on_adv_evt, ctx);
#endif /* SKALD_ON_RTT */
    
}

static void _on_adv_evt(void *arg)
{
    /* Does not work without a delay of 10 μs */
    xtimer_usleep(10);

    skald_ctx_t *ctx = (skald_ctx_t *)arg;

    /* Advertise on the next adv channel - or skip this event if the radio is
     * busy */
    if ((ctx->cur_chan < ADV_CHAN_NUMOF) && (_radio->context == NULL)) {
        _radio->context = ctx;
        _ble_ctx.chan = _adv_chan[ctx->cur_chan];
        netdev_ble_set_ctx(_radio, &_ble_ctx);
        netdev_ble_send(_radio, &ctx->pkt);
        ++ctx->cur_chan;
    }
    else {
        ctx->cur_chan = 0;
        _sched_next(ctx);
    }
}

static void _on_radio_evt(netdev_t *netdev, netdev_event_t event)
{
    (void)netdev;
	
    if (event == NETDEV_EVENT_TX_COMPLETE) {
        skald_ctx_t *ctx = _radio->context;
        _stop_radio();
#ifndef SKALD_ON_RTT
        xtimer_set(&ctx->timer, 150);
#else
        xtimer_usleep(150);
        _on_adv_evt(ctx);
#endif /* SKALD_ON_RTT */
    }
}

void skald_init(void)
{
    assert(dev);

    rtt_init();

    /* Setup and a fitting radio driver - potentially move to auto-init at some
     * point */
#if defined(MODULE_NRFBLE)
    _radio = nrfble_setup();
#endif

    _radio->event_callback = _on_radio_evt;
    _radio->driver->init(_radio);
}

void skald_adv_start(skald_ctx_t *ctx)
{
    assert(ctx);

    /* Make sure the given context is not advertising at the moment */
    skald_adv_stop(ctx);

    /* Initialize advertising context */
#ifndef SKALD_ON_RTT
    ctx->timer.callback = _on_adv_evt;
    ctx->timer.arg = ctx;
    ctx->last = xtimer_now_usec();
#else
    ctx->last = rtt_get_counter();
#endif /* SKALD_ON_RTT */

    ctx->cur_chan = 0;
    ctx->pkt.flags = (BLE_ADV_NONCON_IND | BLE_LL_FLAG_TXADD);

    /* Start advertising */
    _sched_next(ctx);
}

void skald_adv_stop(skald_ctx_t *ctx)
{
    assert(ctx);

#ifndef SKALD_ON_RTT
    xtimer_remove(&ctx->timer);
#else
    //
#endif /* SKALD_ON_RTT */

    if (_radio->context == (void *)ctx) {
        _stop_radio();
    }
}

void skald_generate_random_addr(uint8_t *buf)
{
    assert(buf);

    luid_custom(buf, BLE_ADDR_LEN, 1);

    /* Swap byte 0 and 5, so that the unique byte given by luid does not clash
     * with universal/local and individual/group bits of address */
    uint8_t tmp = buf[5];
    buf[5] = buf[0];

    /* Make address individual and local */
    buf[0] = ((tmp & 0xfc) | 0x02);
}
