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
 * @file		pending-fifo.h
 * @brief       FIFO definitions
 * @author      Eugene Ponomarev
 */
#ifndef PENDING_FIFO_H_
#define PENDING_FIFO_H_

#include <stdbool.h>

#include "mutex.h"

#define GC_MAX_PENDING 16
#define GC_MAX_REPLY_LEN 256

/**
 * @brief describes the frame queue.
 */
typedef struct {
	char fifo[GC_MAX_PENDING][GC_MAX_REPLY_LEN];	/**< Queue data */

	int front;	/**< Pointer to the queue's front */
	int rear;	/**< Pointer to the queue's start */

	mutex_t mutex; /**< FIFO's mutex */
} gc_pending_fifo_t;

/**
 * @brief initialies the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 */
void gc_pending_fifo_init(gc_pending_fifo_t *fifo);

/**
 * @brief evicts element from the end of a queue.
 *
 * @param	[IN]	*fifo	pointer to the FIFO structure
 * @param	[OUT]	*reply	pointer to the buf to write
 *
 * @return false if queue is empty
 */
bool gc_pending_fifo_pop(gc_pending_fifo_t *fifo, char *buf);

/**
 * @brief inserts element into the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*buf	pointer to the reply buffer to insert
 *
 * @return 	false if frame is full
 */
bool gc_pending_fifo_push(gc_pending_fifo_t *fifo, char buf[GC_MAX_REPLY_LEN]);

/**
 * @biref checks that queue is empty or not.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is empty
 */
bool gc_pending_fifo_empty(gc_pending_fifo_t *fifo);

/**
 * @brief checks that queue is full.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is full
 */
bool gc_pending_fifo_full(gc_pending_fifo_t *fifo);

#endif /* PENDING_FIFO_H_ */
