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
 * @file		ls_frame_fifo.c
 * @brief       ls Frame FIFO implementation
 * @author      Evgeniy Ponomarev
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "include/ls-frame-fifo.h"
#include "mutex.h"
#include "irq.h"

#include "ls-mac-types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ls_frame_fifo_init(ls_frame_fifo_t *fifo) {
	mutex_init(&fifo->mutex);
	fifo->front = fifo->rear = -1;
}

bool ls_frame_fifo_pop(ls_frame_fifo_t *fifo, ls_frame_t *frame) {
	if (ls_frame_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);

	if (frame != NULL) {
		*frame = fifo->fifo[fifo->front];
	}

	if (fifo->front == fifo->rear) {
		fifo->front = fifo->rear = -1;

		mutex_unlock(&fifo->mutex);
		return true;
	}
	fifo->front = (fifo->front + 1) % LS_MAX_FRAME_FIFO_SIZE;

	mutex_unlock(&fifo->mutex);
	return true;
}

bool ls_frame_fifo_peek(ls_frame_fifo_t *fifo, ls_frame_t *frame) {
	if (ls_frame_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);
	*frame = fifo->fifo[fifo->front];
	mutex_unlock(&fifo->mutex);

	return true;
}

bool ls_frame_fifo_push(ls_frame_fifo_t *fifo, ls_frame_t *frame) {
	if (ls_frame_fifo_full(fifo)) {
		return false;
	}

	int c = irq_disable();

	if (ls_frame_fifo_empty(fifo)) {
		fifo->front = fifo->rear = 0;
	} else {
		fifo->rear = (fifo->rear + 1) % LS_MAX_FRAME_FIFO_SIZE;
	}

	fifo->fifo[fifo->rear] = *frame;

	irq_restore(c);

	return true;
}

bool ls_frame_fifo_full(ls_frame_fifo_t *fifo) {
	return ((fifo->rear + 1) % LS_MAX_FRAME_FIFO_SIZE) == fifo->front;
}

bool ls_frame_fifo_empty(ls_frame_fifo_t *fifo) {
	return (fifo->front == -1 && fifo->rear == -1);
}

int ls_frame_fifo_size(ls_frame_fifo_t *fifo) {
	if (ls_frame_fifo_empty(fifo)) {
		return 0;
	}

	int size = fifo->rear - fifo->front;

	/* Front may be ahead of rear */
	if (size < 0)
		size = -size;

	return size;
}

void ls_frame_fifo_clear(ls_frame_fifo_t *fifo) {
	int c = irq_disable();
	fifo->front = fifo->rear = -1;
	irq_restore(c);
}

#ifdef __cplusplus
}
#endif
