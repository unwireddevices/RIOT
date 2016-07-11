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
 * @file		lrn_fifo.h
 * @brief       FIFO definitions
 * @author      Unwired Devices
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_FRAME_FIFO_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_FRAME_FIFO_H_

#include <stdbool.h>

#include "mutex.h"

#include "lrn_mac_types.h"

/**
 * @brief The biggest possible queue size.
 */
#define LRN_MAX_FRAME_FIFO_SIZE 10

/**
 * @brief describes the frame queue.
 */
typedef struct {
	lrn_frame_t fifo[LRN_MAX_FRAME_FIFO_SIZE];	/**< Queue data */

	int front;	/**< Pointer to the queue's front */
	int rear;	/**< Pointer to the queue's start */

	mutex_t mutex; /**< FIFO's mutex */
} lrn_frame_fifo_t;

/**
 * @brief initialies the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 */
void lrn_frame_fifo_init(lrn_frame_fifo_t *fifo);

/**
 * @brief evicts element from the end of a queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the frame to write the output
 *
 * @return false if queue is empty
 */
bool lrn_frame_fifo_pop(lrn_frame_fifo_t *fifo, lrn_frame_t *frame);

/**
 * @brief inserts element into the queue.
 *
 * @param	*fifo	pointer to the FIFO structure
 * @param	*frame	pointer to the frame to insert
 *
 * @return 	false if frame is full
 */
bool lrn_frame_fifo_push(lrn_frame_fifo_t *fifo, lrn_frame_t *frame);

/**
 * @biref checks that queue is empty or not.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is empty
 */
bool lrn_frame_fifo_empty(lrn_frame_fifo_t *fifo);

/**
 * @brief checks that queue is full.
 *
 * @param	*fifo	pointer to the FIFO structure
 *
 * @return	true if queue is full
 */
bool lrn_frame_fifo_full(lrn_frame_fifo_t *fifo);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_FRAME_FIFO_H_ */
