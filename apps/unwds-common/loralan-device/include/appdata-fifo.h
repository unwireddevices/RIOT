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
