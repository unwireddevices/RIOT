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
 * @file		appdata-fifo.h
 * @brief       Application data FIFO definitions
 * @author      Eugene Ponomarev [ep@unwds.com]
 */
#ifndef APPDATA_FIFO_H_
#define APPDATA_FIFO_H_

#include <stdint.h>
#include "mutex.h"

/**
 * @brief Maximum application data payload size in bytes.
 */
#define APPDATA_FIFO_MAX_APPDATA_SIZE 32

/**
 * @brief Maximum number of entries to store in FIFO.
 */
#define APPDATA_FIFO_SIZE 2

typedef struct {
	uint8_t data[APPDATA_FIFO_MAX_APPDATA_SIZE];	/**< Application data */
	uint8_t size;									/**< Size of data */
	uint8_t id;										/**< ID of data packet to avoid sending duplicates via network */

	bool is_confirmed;								/**< Data requires confirmation */
	bool is_with_ack;								/**< Implicit ACK to the app. data previously received */
} appdata_fifo_entry_t;

/**
 * @brief describes the frame queue.
 */
typedef struct {
	appdata_fifo_entry_t fifo[APPDATA_FIFO_SIZE];	/**< Queue data */

	int front;	/**< Pointer to the queue's front */
	int start;	/**< Pointer to the queue's start */

	mutex_t mutex; /**< FIFO's mutex */
} appdata_fifo_t;

void appdata_fifo_init(appdata_fifo_t *fifo);

bool appdata_fifo_pop(appdata_fifo_t *fifo, appdata_fifo_entry_t *e);

bool appdata_fifo_peek(appdata_fifo_t *fifo, appdata_fifo_entry_t *e);

bool appdata_fifo_push(appdata_fifo_t *fifo, uint8_t *buf, size_t bufsize, uint8_t id, bool is_confirmed, bool is_with_ack);

bool appdata_fifo_full(appdata_fifo_t *fifo);

bool appdata_fifo_empty(appdata_fifo_t *fifo);

int appdata_fifo_size(appdata_fifo_t *fifo);

void appdata_fifo_clear(appdata_fifo_t *fifo);


#endif /* APPDATA_FIFO_H_ */
