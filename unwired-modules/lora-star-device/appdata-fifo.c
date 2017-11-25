/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file		appdata-fifo.c
 * @brief       Application data FIFO implementation
 * @author      Eugene Ponomarev [ep@unwds.com]
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "appdata-fifo.h"
#include "mutex.h"
#include "irq.h"

#ifdef __cplusplus
extern "C" {
#endif


void appdata_fifo_init(appdata_fifo_t *fifo) {
	mutex_init(&fifo->mutex);
	fifo->front = fifo->start = -1;
}

bool appdata_fifo_pop(appdata_fifo_t *fifo, appdata_fifo_entry_t *e) {
	if (appdata_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);

	if (e != NULL) {
		*e = fifo->fifo[fifo->front];
	}

	if (fifo->front == fifo->start) {
		fifo->front = fifo->start = -1;

		mutex_unlock(&fifo->mutex);
		return true;
	}
	fifo->front = (fifo->front + 1) % APPDATA_FIFO_SIZE;

	mutex_unlock(&fifo->mutex);
	return true;
}

bool appdata_fifo_peek(appdata_fifo_t *fifo, appdata_fifo_entry_t *e) {
	if (appdata_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);
	*e = fifo->fifo[fifo->front];
	mutex_unlock(&fifo->mutex);

	return true;
}

bool appdata_fifo_push(appdata_fifo_t *fifo, uint8_t *buf, size_t bufsize, uint8_t id, bool is_confirmed, bool is_with_ack) {
	if (appdata_fifo_full(fifo)) {
		return false;
	}

	int c = irq_disable();

	if (appdata_fifo_empty(fifo)) {
		fifo->front = fifo->start = 0;
	} else {
		fifo->start = (fifo->start + 1) % APPDATA_FIFO_SIZE;
	}

	appdata_fifo_entry_t *e = &fifo->fifo[fifo->start];
	memset(e, 0, sizeof(appdata_fifo_entry_t));
	memcpy(e->data, buf, bufsize);

	e->size = bufsize;
	e->id = id;

	e->is_confirmed = is_confirmed;
	e->is_with_ack = is_with_ack;

	irq_restore(c);

	return true;
}

bool appdata_fifo_full(appdata_fifo_t *fifo) {
	return ((fifo->start + 1) % APPDATA_FIFO_SIZE) == fifo->front;
}

bool appdata_fifo_empty(appdata_fifo_t *fifo) {
	return (fifo->front == -1 && fifo->start == -1);
}

int appdata_fifo_size(appdata_fifo_t *fifo) {
	if (appdata_fifo_empty(fifo)) {
		return 0;
	}

	int size = fifo->start - fifo->front;

	/* Front may be behind of start */
	if (size < 0)
		size = -size;

	return size;
}

void appdata_fifo_clear(appdata_fifo_t *fifo) {
	int c = irq_disable();
	fifo->front = fifo->start = -1;
	irq_restore(c);
}


#ifdef __cplusplus
}
#endif
