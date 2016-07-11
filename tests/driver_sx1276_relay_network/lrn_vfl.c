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
 * @file		lrn_vfl.c
 * @brief       LRN Victim Frame list implementation
 * @author      Evgeniy Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "xtimer.h"
#include "mutex.h"

#include "lrn_mac_types.h"
#include "lrn_vfl.h"

void lrn_vfl_init(lrn_vfl_t *vfl) {
	vfl->timer.arg = vfl;
	vfl->timer.callback = lrn_vfl_timer_cb;

	mutex_init(&vfl->mutex);

	memset(&vfl->free_list, 1, MAX_VFL_ENTRIES);
}

static int get_frame_index(lrn_vfl_t *vfl, lrn_frame_id_t fid) {
	for(int i = 0; i < MAX_VFL_ENTRIES; i++) {
		if (vfl->free_list[i])
			continue;

		/* Same FID and type */
		if (vfl->list[i].frame.header.fid == fid) {
			return i;
		}
	}


	return -1;
}

static void reset_timer(lrn_vfl_t *vfl) {
	if (lrn_vfl_empty(vfl))
		return;

	int min_t = 0;
	for (int i = 0; i < MAX_VFL_ENTRIES; i++) {
		if (!vfl->free_list[i]) {
			/* Skip expired frames */
			if (vfl->list[i].expired)
				continue;

			if (min_t == 0 || vfl->list[i].time < min_t) {
				min_t = vfl->list[i].time;
			}
		}
	}

	uint32_t now = xtimer_now();
	if (min_t > now) {
		puts("vfl: timer set");
		xtimer_set(&vfl->timer, min_t - now);
	} else {
		puts("vfl: curr > min. Timer didn't shot?");
		//xtimer_set(&vfl->timer, 1);
	}
}

void lrn_vfl_add(lrn_vfl_t *vfl, lrn_frame_t *frame, bool nh_ack, bool dst_ack) {
	/* Makes no sense to add frame witout pending acknowledges */
	if (!nh_ack && !dst_ack)
		return;

	if (vfl->size == MAX_VFL_ENTRIES) {
		puts("vfl: list overflow!");
		return;
	}

	/* Look for the existing frame */
	int idx = get_frame_index(vfl, frame->header.fid);

	/* Frame already in victim list */
	if (idx != -1) {
		/* Restart frame if it was expired */
		vfl->list[idx].expired = false;

		/* Update it's acknowledges */
		if (vfl->list[idx].nh_ack)
			vfl->list[idx].nh_ack = !nh_ack;

		if (vfl->list[idx].dst_ack)
			vfl->list[idx].dst_ack = !dst_ack;

		/* Update it's time */
		if (nh_ack)
			vfl->list[idx].time = xtimer_now() + LRN_NH_ACK_TIMEOUT;
		else if (dst_ack && vfl->list[idx].nh_ack)
			vfl->list[idx].time = xtimer_now() + LRN_DST_ACK_TIMEOUT;

		reset_timer(vfl);

		return;
	}

	/* Frame is not in list, look for a free cell for it */
	for(int i = 0; i < MAX_VFL_ENTRIES; i++) {
		if (vfl->free_list[i]) {
			/* Occupy cell */
			vfl->free_list[i] = 0;

			/* Write an entry */
			lrn_vfl_entry_t e;
			e.frame = *frame;
			e.expired = false;
			e.nh_ack = !nh_ack;
			e.dst_ack = !dst_ack;

			if (nh_ack)
				e.time = xtimer_now() + LRN_NH_ACK_TIMEOUT;
			else if (dst_ack)
				e.time = xtimer_now() + LRN_DST_ACK_TIMEOUT;

			vfl->list[i] = e;

			/* Increase item counter */
			vfl->size++;
			break;
		}
	}

	/* Reset timer for the nearest expiration timestamp */
	reset_timer(vfl);
}

void lrn_vfl_timer_cb(void *arg) {
	lrn_vfl_t *vfl = (lrn_vfl_t *) arg;

	uint32_t curr_time = xtimer_now();

	int c = irq_disable();

	/* Expire frames */
	for(int i = 0; i < MAX_VFL_ENTRIES; i++) {
		if (vfl->free_list[i])
			continue;

		lrn_vfl_entry_t *curr = &vfl->list[i];

		if (curr->expired)
			continue;

		if (curr->time <= curr_time) {
			curr->expired = true;
		}
	}

	/* Process expired frames */
	for (int i = 0; i < MAX_VFL_ENTRIES; i++) {
		lrn_vfl_entry_t *curr = &vfl->list[i];

		if (!curr->expired)
			continue;

		/* Tell the LRN stack about expired frame */
		vfl->frame_ack_expired_cb(vfl->frame_ack_expired_cb_arg, &curr->frame);
	}

	irq_restore(c);
}

void lrn_vfl_remove(lrn_vfl_t *vfl, lrn_frame_id_t fid, bool nh_ack) {
	int idx = get_frame_index(vfl, fid);

	if (idx == -1)
		return;

	lrn_vfl_entry_t *e = &vfl->list[idx];

	if (!e->nh_ack)
		e->nh_ack = nh_ack;

	if (!e->dst_ack)
		e->dst_ack = !nh_ack;

	/* Both types acknowledged, remove the frame */
	if (e->nh_ack && e->dst_ack) {
		vfl->free_list[idx] = 1;
		vfl->size--;

		return;
	}

	/* NH acknowledged, the DST ACK remains */
	if (e->nh_ack && !e->dst_ack)
		e->time = xtimer_now() + (LRN_DST_ACK_TIMEOUT - LRN_NH_ACK_TIMEOUT);
}

void lrn_vfl_print(lrn_vfl_t *vfl) {
	printf("list: ");
    lrn_vfl_entry_t *curr;

	for (int i = 0; i < MAX_VFL_ENTRIES; i++) {
		if (vfl->free_list[i])
			continue;

		curr = &vfl->list[i];
		printf("%u ", (unsigned) curr->time);
	}

	puts("");
}

bool lrn_vfl_empty(lrn_vfl_t *vfl) {
	return (vfl->size == 0);
}

#ifdef __cplusplus
}
#endif
