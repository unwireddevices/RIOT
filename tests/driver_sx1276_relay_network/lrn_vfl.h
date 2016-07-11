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
 * @file		lrn_vfl.h
 * @brief       Definitions for the list of frames which are waiting for acknowledge
 * @author      Evgeniy Ponomarev
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_VFL_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_VFL_H_

#include <stdint.h>
#include <stdbool.h>

#include "xtimer.h"
#include "mutex.h"

#include "lrn_mac_types.h"

#define LRN_DST_ACK_TIMEOUT 1e6 * 30 /* 30 seconds */
#define LRN_NH_ACK_TIMEOUT 1e6 * 15 /* 15 seconds */

#define MAX_VFL_ENTRIES 10

typedef struct lrn_vfl_enty_s {
	lrn_frame_t frame;
	uint32_t time;

	bool nh_ack;	/**< Next hop is acknowledged for this frame */
	bool dst_ack;	/**< Destination is acknowledged for this frame */


	bool expired;
} lrn_vfl_entry_t;

typedef struct {
	uint8_t free_list[MAX_VFL_ENTRIES];
	lrn_vfl_entry_t list[MAX_VFL_ENTRIES];
	xtimer_t timer;

	uint8_t size;
	mutex_t mutex;

	void (*frame_ack_expired_cb)(void *arg, lrn_frame_t *frame);
	void *frame_ack_expired_cb_arg;
} lrn_vfl_t;

void lrn_vfl_init(lrn_vfl_t *vfl);
void lrn_vfl_add(lrn_vfl_t *vfl, lrn_frame_t *frame, bool nh_ack, bool dst_ack);
bool lrn_vfl_empty(lrn_vfl_t *vfl);
void lrn_vfl_timer_cb(void *lrn);
void lrn_vfl_print(lrn_vfl_t *vfl);
void lrn_vfl_remove(lrn_vfl_t *vfl, lrn_frame_id_t fid, bool nh_ack);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_VFL_H_ */
