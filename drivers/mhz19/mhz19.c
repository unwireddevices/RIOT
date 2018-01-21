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
 * @file		mhz19.c
 * @brief       CO2 sensor module MH-Z19 UART driver implementation
 * @author      Dmitry Golik
 * @author      Oleg Artamonov
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "thread.h"
#include "assert.h"
#include "rtctimers-millis.h"
#include "periph/uart.h"

#include "mhz19.h"

#define MHZ19_RXBUF_SIZE (9)

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t mhz19_command;
static msg_t mhz19_msg;
static rtctimers_millis_t timeout_timer;
static int num_bytes_received = 0;

static void rx_cb(void *arg, uint8_t data)
{
	mhz19_t *dev = (mhz19_t *) arg;
    
    /* first byte of sensor response is always 0xFF */
    if ((num_bytes_received == 0) && (data != 0xFF)) {
        return;
    }

    dev->rxbuf[num_bytes_received++] = data;
    
    /* full data block is MHZ19_RXBUF_SIZE (9 bytes) */
	if (num_bytes_received == MHZ19_RXBUF_SIZE) {
        mhz19_msg.content.value = 0;
        msg_send(&mhz19_msg, dev->reader_pid);
		num_bytes_received = 0;
	}
}

static uint8_t mhz19_checksum(uint8_t *data) {
    uint8_t checksum = 0;
    for (int i = 1; i < 8; i++) {
        checksum += data[i];
    }
    
    return (1 + (~checksum));
}

static int parse(mhz19_t *dev, mhz19_data_t *data) {
    if (dev->rxbuf[0] != 0xff) {
        /* invalid data */
        return -1;
    }
    
    /*
    if (dev->rxbuf[1] != mhz19_command) {
        return -2;
    }
    */
    
    if (dev->rxbuf[8] != mhz19_checksum(dev->rxbuf)) {
        /* invalid checksum */
        return -3;
    }
    
    data->co2 = dev->rxbuf[2] * 256 + dev->rxbuf[3];
    data->temperature = dev->rxbuf[4] - 40;
    data->validity = dev->rxbuf[5];
    
    return 0;
}

static void *reader(void *arg) {
    mhz19_t *dev = (mhz19_t *) arg;

    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);
    
    mhz19_data_t data;
    
    while (1) {
        msg_receive(&msg);
        rtctimers_millis_remove(&timeout_timer);        
        memset(&data, 0, sizeof(data));

        if ((!msg.content.value) && (!parse(dev, &data))) {
        	if (dev->params.mhz19_cb != NULL)
        		dev->params.mhz19_cb(data);
        }
    }
    return NULL;
}

void mhz19_get(mhz19_t *dev) {
     /* 0x86 command to get data from sensor */
    mhz19_command = 0x86;
    uint8_t data[9] = {0xFF, 0x01, mhz19_command };
    data[8] = mhz19_checksum(data);    
    uart_write(dev->params.uart, (uint8_t *)data, sizeof(data));
    
    /* if MH-Z19 wouldn't respond in MHZ19_TIMEOUT_SECONDS seconds, send empty data to application */
    mhz19_msg.content.value = 1;
    rtctimers_millis_set_msg(&timeout_timer, 1000*MHZ19_TIMEOUT_SECONDS, &mhz19_msg, dev->reader_pid);
}

int mhz19_init(mhz19_t *dev, mhz19_param_t *param) {
	assert(dev != NULL);
	assert(param != NULL);

	/* Copy parameters */
	dev->params = *param;

	/* Initialize the UART */
	if (uart_init(dev->params.uart, MHZ19_UART_BAUDRATE, rx_cb, dev)) {
		return -1;
	}

	/* Create reader thread */
	dev->reader_pid = thread_create((char *) dev->reader_stack, MHZ19_READER_THREAD_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, 0, reader, dev, "MH-Z19 reader");
	if (dev->reader_pid <= KERNEL_PID_UNDEF) {
		return -2;
	}

	return 1;
}

#ifdef __cplusplus
}
#endif
