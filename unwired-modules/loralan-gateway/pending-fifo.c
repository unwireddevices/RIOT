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
 * @file		gc_pending_fifo.c
 * @brief       ls Frame FIFO implementation
 * @author      Evgeniy Ponomarev
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pending-fifo.h"
#include "mutex.h"
#include "irq.h"

#ifdef __cplusplus
extern "C" {
#endif

void gc_pending_fifo_init(gc_pending_fifo_t *fifo) {
	mutex_init(&fifo->mutex);
	fifo->front = fifo->rear = -1;
}

bool gc_pending_fifo_pop(gc_pending_fifo_t *fifo, char *buf) {
	if (gc_pending_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);

	memcpy(buf, fifo->fifo[fifo->front], GC_MAX_REPLY_LEN);

	if (fifo->front == fifo->rear) {
		fifo->front = fifo->rear = -1;

		mutex_unlock(&fifo->mutex);
		return true;
	}

	fifo->front = (fifo->front + 1) % GC_MAX_PENDING;

	mutex_unlock(&fifo->mutex);
	return true;
}

bool gc_pending_fifo_push(gc_pending_fifo_t *fifo, char buf[GC_MAX_REPLY_LEN]) {
	if (gc_pending_fifo_full(fifo)) {
		return false;
	}

	int c = irq_disable();

	if (gc_pending_fifo_empty(fifo)) {
		fifo->front = fifo->rear = 0;
	} else {
		fifo->rear = (fifo->rear + 1) % GC_MAX_PENDING;
	}

	memcpy(fifo->fifo[fifo->rear], buf, GC_MAX_REPLY_LEN);

	irq_restore(c);

	return true;
}

bool gc_pending_fifo_full(gc_pending_fifo_t *fifo) {
	return ((fifo->rear + 1) % GC_MAX_PENDING) == fifo->front;
}

bool gc_pending_fifo_empty(gc_pending_fifo_t *fifo) {
	return (fifo->front == -1 && fifo->rear == -1);
}

#ifdef __cplusplus
}
#endif
