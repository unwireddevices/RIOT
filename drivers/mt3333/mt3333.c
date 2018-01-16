/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		mt3333.c
 * @brief       Mediatek MT3333-based GNSS module driver implementation
 * @author      EP <ep@unwds.com>
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "thread.h"
#include "assert.h"
#include "ringbuffer.h"
#include "periph/uart.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "mt3333.h"

#ifdef __cplusplus
extern "C" {
#endif

mt3333_t *_dev;

static void rx_cb(void *arg, uint8_t data)
{
	mt3333_t *dev = (mt3333_t *) arg;

	/* Insert received character into ring buffer */
	ringbuffer_add_one(&dev->rxrb, data);

    /* Notify parser thread about ready message */
    if (data == MT3333_EOL) {
        msg_t msg;
        msg_send(&msg, dev->reader_pid);
    }
}

static int get_csv_field(char *buf, int fieldno, char *field, int maxlen) {
	int len = strlen(buf);

	int current_field = 0;
	int fieldpos = 0;

	for (int i = 0; i < len; i++) {
		if (buf[i] == ',') {
			current_field++;
			continue;
		}

		if (current_field == fieldno) {
			if (fieldpos == maxlen)
				break;

			field[fieldpos++] = buf[i];
		}

		if (current_field > fieldno)
			break;
	}

	if (maxlen > 1) {
		field[fieldpos] = '\0';
	}

	return fieldpos + 1;
}

/**
 * @brief Parses GPS data in NMEA format
 */
static bool parse(mt3333_t *dev, char *buf, mt3333_gps_data_t *data) {
	/* We're interested in G/NRMC packets */
	if (strstr(buf, "RMC") == NULL) {
		return false;
	}

	/* Check validity sign */
	char valid;
	if (get_csv_field(buf, MT3333_VALID_FIELD_IDX, &valid, 1)) {
		if (valid != 'A')
			return false;
	}

	char ns, ew;

	if (!get_csv_field(buf, MT3333_TIME_FIELD_IDX, data->time, 15))
		return false;

	if (!get_csv_field(buf, MT3333_LAT_FIELD_IDX, data->lat, 15))
		return false;

	if (!get_csv_field(buf, MT3333_LON_FIELD_IDX, data->lon, 15))
		return false;

	if (!get_csv_field(buf, MT3333_NS_FIELD_IDX, &ns, 1))
		return false;

	if (!get_csv_field(buf, MT3333_EW_FIELD_IDX, &ew, 1))
		return false;

	if (!get_csv_field(buf, MT3333_DATE_FIELD_IDX, data->date, 15))
		return false;

	/* Check N/S E/W polarity */
	data->e = (ew == 'E');
	data->n = (ns == 'N');

	return true;
}

static void *reader(void *arg) {
	mt3333_t *dev = (mt3333_t *) arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    mt3333_gps_data_t data;
    memset(&data, 0, sizeof(data));

    char buf[MT3333_PARSER_BUF_SIZE] = { '\0' };

    while (1) {
        msg_receive(&msg);

        /* Collect input string from the ring buffer */
        char c;
        int i = 0;
        do {
        	c = ringbuffer_get_one(&dev->rxrb);
        	buf[i++] = c;
        } while (c != MT3333_EOL);

        /* Strip the string just in case that there's a garbage after EOL */
        buf[i] = '\0';

        /* Parse received string */
        if (parse(dev, buf, &data)) {
        	if (dev->params.gps_cb != NULL)
        		dev->params.gps_cb(data);
        }
    }

    return NULL;
}

static void mt3333_send_at_command(char *command) {
    uint8_t checksum = 0;
    int i;
    for (i = 0; i < strlen(command); i++) {
        checksum ^= (uint8_t)command[i];
    }
    
    /* command + delimeters + checksum + CRLF + ending 0 */
    if (strlen(command) + 2 + 2 + 3 > 100) {
        DEBUG("[mt3333 ] command too long\n");
        return;
    }
    
    /* first 100 bytes are free to use */
    char *cmd = (char *)_dev->reader_stack;
    
    snprintf(cmd, 2, "$");
    strcat(cmd, command);
    strcat(cmd, "*");
    snprintf(cmd, 5, "\r\n%02x", checksum);
    
    uart_write(_dev->params.uart, (uint8_t *)cmd, strlen(cmd));
}

void mt3333_set_baudrate(int baudrate) {
    char cmd[20] = {};
    snprintf(cmd, 20, "PQBAUD,W,%d", baudrate);
    mt3333_send_at_command(cmd);
    
    _dev->params.baudrate = baudrate;
    uart_init(_dev->params.uart, _dev->params.baudrate, rx_cb, _dev);
}

void mt3333_set_glp(bool enabled) {
    char cmd[20] = {};
    if (enabled) {
        snprintf(cmd, 20, "PQGLP,W,1,1");
    } else {
        snprintf(cmd, 20, "PQGLP,W,0,1");
    }
    mt3333_send_at_command(cmd);
}

void mt3333_set_powersave(mt3333_powersave_mode_t mode) {
    char cmd[20] = {};
    if (mode == MT3333_POWERSAVE_STANDBY) {
        snprintf(cmd, 20, "PMTK161,0");
    } else if (mode == MT3333_POWERSAVE_BACKUP) {
        snprintf(cmd, 20, "PMTK225,4");
    } else if (mode == MT3333_POWERSAVE_FULLON) {
        snprintf(cmd, 20, "PMTK225,0");
    }
    mt3333_send_at_command(cmd);
}

void mt3333_set_periodic(mt3333_powersave_mode_t mode, int run, int sleep, int run_ext, int sleep_ext) {
    char cmd[50] = {};
    snprintf(cmd, 50, "PMTK225,%d,%d,%d,%d,%d", (int)mode, run, sleep, run_ext, sleep_ext);
    mt3333_send_at_command(cmd);
}

int mt3333_init(mt3333_t *dev, mt3333_param_t *param) {
	assert(dev != NULL);
	assert(param != NULL);

	/* Copy parameters */
	dev->params = *param;
    
    _dev = dev;
    
    /* Create reader thread */
	dev->reader_pid = thread_create(dev->reader_stack + 100 + MT3333_RXBUF_SIZE_BYTES,
                                    MT3333_READER_THREAD_STACK_SIZE_BYTES - 100 - MT3333_RXBUF_SIZE_BYTES,
                                    THREAD_PRIORITY_MAIN - 1, 0, reader, dev, "MT3333 reader");
	if (dev->reader_pid <= KERNEL_PID_UNDEF) {
		return -2;
	}
    
    dev->rxbuf = dev->reader_stack + 100;

	/* Initialize the input ring buffer */
	ringbuffer_init(&dev->rxrb, dev->rxbuf, MT3333_RXBUF_SIZE_BYTES);

	/* Initialize the UART */
	if (uart_init(dev->params.uart, dev->params.baudrate, rx_cb, dev)) {
		return -1;
	}

	return 0;
}

#ifdef __cplusplus
}
#endif
