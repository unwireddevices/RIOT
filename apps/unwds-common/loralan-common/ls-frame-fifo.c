/*
 * Copyright (C) 2016-2019 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

bool ls_frame_fifo_replace(ls_frame_fifo_t *fifo, ls_frame_t *frame) {
	if (ls_frame_fifo_empty(fifo)) {
		return false;
	}

	mutex_lock(&fifo->mutex);
	fifo->fifo[fifo->front] = *frame;
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

	return (size + 1);
}

void ls_frame_fifo_clear(ls_frame_fifo_t *fifo) {
	int c = irq_disable();
	fifo->front = fifo->rear = -1;
	irq_restore(c);
}

#ifdef __cplusplus
}
#endif
