/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

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
