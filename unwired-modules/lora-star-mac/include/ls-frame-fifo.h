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
 * @brief The biggest possible queue size.
 */
#define LS_MAX_FRAME_FIFO_SIZE 8

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
