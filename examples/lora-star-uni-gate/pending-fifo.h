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
