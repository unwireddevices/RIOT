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
 * @file		ls_fifo.h
 * @brief       FIFO definitions
 * @author      Eugene Ponomarev
 */
#ifndef LS_FRAME_FIFO_H_
#define LS_FRAME_FIFO_H_

#include <stdbool.h>

#include "mutex.h"
#include "ls-mac-types.h"

/**
 * @brief The biggest possible queue size. Must be power of 2.
 */
#define LS_MAX_FRAME_FIFO_SIZE 4

/**
 * @brief describes the frame queue.
 */
typedef struct {
	ls_frame_t fifo[LS_MAX_FRAME_FIFO_SIZE];	/**< Queue data */

	int front;	/**< Pointer to the queue's front */
	int rear;	/**< Pointer to the queue's start */

	mutex_t mutex; /**< FIFO's mutex */
} ls_frame_fifo_t;

/**
 * @brief initialies the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 */
void ls_frame_fifo_init(ls_frame_fifo_t *fifo);

/**
 * @brief evicts element from the end of a queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the frame to write the output
 *
 * @return false if queue is empty
 */
bool ls_frame_fifo_pop(ls_frame_fifo_t *fifo, ls_frame_t *frame);

/**
 * @brief polls element from the end of a queue but doesn't evicts it.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the frame to write the output
 *
 * @return false if queue is empty
 */
bool ls_frame_fifo_peek(ls_frame_fifo_t *fifo, ls_frame_t *frame);

/**
 * @brief replaces oldest element in FIFO.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the replacement frame
 *
 * @return false if queue is empty
 */
bool ls_frame_fifo_replace(ls_frame_fifo_t *fifo, ls_frame_t *frame);

/**
 * @brief inserts element into the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the frame to insert
 *
 * @return 	false if frame is full
 */
bool ls_frame_fifo_push(ls_frame_fifo_t *fifo, ls_frame_t *frame);

/**
 * @biref checks that queue is empty or not.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is empty
 */
bool ls_frame_fifo_empty(ls_frame_fifo_t *fifo);

/**
 * @brief checks that queue is full.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is full
 */
bool ls_frame_fifo_full(ls_frame_fifo_t *fifo);

/**
 * @brief Gets number of elements currently in queue
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	0 if queue is empty, number of elements in queue otherwise
 */
int ls_frame_fifo_size(ls_frame_fifo_t *fifo);

/**
 * @brief clears the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 */
void ls_frame_fifo_clear(ls_frame_fifo_t *fifo);

#endif /* LS_FRAME_FIFO_H_ */
