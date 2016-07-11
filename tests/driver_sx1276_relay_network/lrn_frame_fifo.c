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
 * @file		lrn_frame_fifo.c
 * @brief       LRN Frame FIFO implementation
 * @author      Evgeniy Ponomarev
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "mutex.h"
#include "irq.h"

#include "lrn_mac_types.h"
#include "lrn_frame_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif



void lrn_frame_fifo_init(lrn_frame_fifo_t *fifo) {
	mutex_init(&fifo->mutex);
	fifo->front = fifo->rear = -1;
}

bool lrn_frame_fifo_pop(lrn_frame_fifo_t *fifo, lrn_frame_t *frame) {
	if (lrn_frame_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);

	*frame = fifo->fifo[fifo->front];

	if (fifo->front == fifo->rear) {
		fifo->front = fifo->rear = -1;

		mutex_unlock(&fifo->mutex);
		return true;
	}
	fifo->front = (fifo->front + 1) % LRN_MAX_FRAME_FIFO_SIZE;

	mutex_unlock(&fifo->mutex);
	return true;
}

bool lrn_frame_fifo_push(lrn_frame_fifo_t *fifo, lrn_frame_t *frame) {
	if (lrn_frame_fifo_full(fifo)) {
		return false;
	}

	int c = irq_disable();

	if (lrn_frame_fifo_empty(fifo)) {
		fifo->front = fifo->rear = 0;
	} else {
		fifo->rear = (fifo->rear + 1) % LRN_MAX_FRAME_FIFO_SIZE;
	}

	fifo->fifo[fifo->rear] = *frame;

	irq_restore(c);

	return true;
}

bool lrn_frame_fifo_full(lrn_frame_fifo_t *fifo) {
	return ((fifo->rear + 1) % LRN_MAX_FRAME_FIFO_SIZE) == fifo->front;
}

bool lrn_frame_fifo_empty(lrn_frame_fifo_t *fifo) {
	return (fifo->front == -1 && fifo->rear == -1);
}

#ifdef __cplusplus
}
#endif
