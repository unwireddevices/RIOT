/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "mt3333.h"
#include "thread.h"
#include "assert.h"

#include "rtctimers-millis.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

static kernel_pid_t reader_pid;
static char *rxbuf;
static char *nmea_buf;

static void rx_cb(void *arg, uint8_t data)
{
    (void)arg;
    static int rx_cnt;

    rxbuf[rx_cnt++] = data;
    
    /* Notify parser thread about RMC message */
    if (data == MT3333_EOL) {
        memcpy(nmea_buf, rxbuf, rx_cnt);
        nmea_buf[rx_cnt] = 0;
        msg_t msg;
        msg_send(&msg, reader_pid);
        rx_cnt = 0;
    }
    
    if (rx_cnt == MT3333_RXBUF_SIZE_BYTES) {
        rx_cnt = 0;
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
static bool parse_rmc(char *buf, mt3333_gps_data_t *data) {
    /* Check validity sign */
    char valid;
    if (get_csv_field(buf, MT3333_RMC_VALID_FIELD_IDX, &valid, 1)) {
        if (valid != 'A') {
            data->valid = false;
            DEBUG("[gps] Data not valid\n");
        } else {
            data->valid = true;
            DEBUG("[gps] Data valid\n");
        }
    }

    char sign = 'N';
    char tmp[15];
    
    if (!get_csv_field(buf, MT3333_RMC_NS_FIELD_IDX, &sign, 1))
        return false;
    
    int s1, s2, s3;
    if (get_csv_field(buf, MT3333_RMC_LAT_FIELD_IDX, tmp, sizeof(tmp))) {
        sscanf(tmp, "%d.%04d", &s1, &s2);
    }
    
    data->lat = (s1/100);
    s1 -= data->lat * 100;
    data->lat *= 1000000;
    data->lat += (1000000*s1)/60;
    data->lat += (10*s2)/6;
    
    if (sign == 'S') {
        data->lat = -data->lat;
    }
    
    DEBUG("[gps] Latitude: %d\n", data->lat);
    
    if (!get_csv_field(buf, MT3333_RMC_LON_FIELD_IDX, tmp, sizeof(tmp)))
        return false;
    if (!get_csv_field(buf, MT3333_RMC_EW_FIELD_IDX, &sign, 1))
        return false;
    
    if (sscanf(tmp, "%d.%04d", &s1, &s2) != 2)
        return false;
    
    data->lon = (s1/100);
    s1 -= data->lon * 100;
    data->lon *= 1000000;
    data->lon += (1000000*s1)/60;
    data->lon += (10*s2)/6;
    
    if (sign == 'W') {
        data->lon = -data->lon;
    }
    
    DEBUG("[gps] Longitude: %d\n", data->lon);

    struct tm time;
    if (get_csv_field(buf, MT3333_RMC_DATE_FIELD_IDX, tmp, sizeof(tmp))) {
        sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3);
    }
    
    time.tm_mday = s1;
    time.tm_mon = s2;
    time.tm_year = 100 + s3;
    
    DEBUG("[gps] Date: %d.%d.%d\n", s1, s2, s3);
    
    if (get_csv_field(buf, MT3333_RMC_TIME_FIELD_IDX, tmp, sizeof(tmp))) {
        sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3);
    }
    
    time.tm_hour = s1;
    time.tm_min = s2;
    time.tm_sec = 100 + s3;
    
    data->time = mktime(&time);
    
    DEBUG("[gps] Time: %d:%d:%d\n", s1, s2, s3);
    
    if (!get_csv_field(buf, MT3333_RMC_VELOCITY_FIELD_IDX, tmp, sizeof(tmp)))
        return false;
    
    if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2)
        return false;
    
    data->velocity = ((1000*s1 + 10*s2)*514)/1000; // mm/s
    DEBUG("[gps] Velocity: %d mm/s\n", data->velocity);
    
    if (!get_csv_field(buf, MT3333_RMC_DIRECTION_FIELD_IDX, tmp, sizeof(tmp)))
        return false;
    
    if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2)
        return false;
    data->direction = (1000*s1 + 10*s2);
    DEBUG("[gps] Direction: %d millidegrees\n", data->direction);

    return true;
}

/**
 * @brief Parses GPS data in NMEA format
 */
static bool parse_gga(char *buf, mt3333_gps_data_t *data) {
    /* Check validity sign */
    char valid;
    if (get_csv_field(buf, MT3333_GGA_VALID_FIELD_IDX, &valid, 1)) {
        if ((valid != '1') || (valid != '2')) {
            data->valid = false;
            DEBUG("[gps] Data not valid\n");
        } else {
            data->valid = true;
            DEBUG("[gps] Data valid\n");
        }
    }

    char sign = 'N';
    char tmp[15];
    
    if (!get_csv_field(buf, MT3333_GGA_NS_FIELD_IDX, &sign, 1))
        return false;
    
    int s1, s2, s3;
    if (get_csv_field(buf, MT3333_GGA_LAT_FIELD_IDX, tmp, sizeof(tmp))) {
        sscanf(tmp, "%d.%04d", &s1, &s2);
    }
    
    data->lat = (s1/100);
    s1 -= data->lat * 100;
    data->lat *= 1000000;
    data->lat += (1000000*s1)/60;
    data->lat += (10*s2)/6;
    
    if (sign == 'S') {
        data->lat = -data->lat;
    }
    
    DEBUG("[gps] Latitude: %d\n", data->lat);
    
    if (!get_csv_field(buf, MT3333_GGA_LON_FIELD_IDX, tmp, sizeof(tmp)))
        return false;
    if (!get_csv_field(buf, MT3333_GGA_EW_FIELD_IDX, &sign, 1))
        return false;
    
    if (sscanf(tmp, "%d.%04d", &s1, &s2) != 2)
        return false;
    
    data->lon = (s1/100);
    s1 -= data->lon * 100;
    data->lon *= 1000000;
    data->lon += (1000000*s1)/60;
    data->lon += (10*s2)/6;
    
    if (sign == 'W') {
        data->lon = -data->lon;
    }
    
    DEBUG("[gps] Longitude: %d\n", data->lon);

    struct tm time;
    time.tm_mday = 1;
    time.tm_mon = 0;
    time.tm_year = 0;
    
    if (get_csv_field(buf, MT3333_GGA_TIME_FIELD_IDX, tmp, sizeof(tmp))) {
        sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3);
    }
    
    time.tm_hour = s1;
    time.tm_min = s2;
    time.tm_sec = 100 + s3;
    
    data->time = mktime(&time);
    
    DEBUG("[gps] Time: %d:%d:%d\n", s1, s2, s3);
    
    data->velocity = 0; /* no velocity in GGA data */
    data->direction = 0; /* no direction in GGA data */

    return true;
}

static void *reader(void *arg) {
	mt3333_t *dev = (mt3333_t *) arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    mt3333_gps_data_t data;

    while (1) {
        msg_receive(&msg);

        /* filter out non-RMC messages */
        /* proper message is $GPRMC/$GNRMC/$GLRMC */
        /* so we are skipping first 3 symbols and looking for 'RMC' */
        DEBUG("[GPS] %s", nmea_buf);
        if (memcmp(&nmea_buf[3], "RMC", 3) == 0) {
            dev->ready = true;
            /* parse RMC message */
            if (parse_rmc(nmea_buf, &data)) {
                if (dev->params.gps_cb != NULL)
                    dev->params.gps_cb(data);
            }
        } else {
            if (memcmp(&nmea_buf[3], "GGA", 3) == 0) {
                dev->ready = true;
                /* if there's no RMC, there may be GGA message */
                if (parse_gga(nmea_buf, &data)) {
                    if (dev->params.gps_cb != NULL) {
                        dev->params.gps_cb(data);
                    }
                }
                
                if (!dev->ready) {
                    /* check for any meaningful message */
                    if (!strcmp(nmea_buf, MT3333_READY_STR) ||
                        !memcmp(&nmea_buf[3], "GSV", 3)     ||
                        !memcmp(&nmea_buf[3], "GSA", 3)     ||
                        !memcmp(&nmea_buf[3], "GGA", 3))
                    {
                        dev->ready = true;
                    }
                }
            }
        }
    }

    return NULL;
}

int mt3333_is_ready(mt3333_t *dev) {
    (void)dev;
    
    dev->ready = false;
    
    /* 3 seconds timeout */
    int counter = 0;
    int timeout = 3000/20;
    do {
        rtctimers_millis_sleep(20);
        counter++;
    } while ((!dev->ready) && (counter < timeout));
    
    if (dev->ready) {
        return MT3333_READY;
    }
    
    return MT3333_TIMEOUT;
}

static void mt3333_send_at_command(mt3333_t *dev, char *command) {
    uint8_t checksum = 0;
    uint32_t i;
    for (i = 0; i < strlen(command); i++) {
        checksum ^= (uint8_t)command[i];
    }
    
    /* command + delimeters + checksum + CRLF + ending 0 */
    char cmd[30];
    if (strlen(command) + 2 + 2 + 3 > sizeof(cmd)) {
        DEBUG("[mt3333 ] command too long\n");
        return;
    }
    
    snprintf(cmd, 2, "$");
    strcat(cmd, command);
    strcat(cmd, "*");
    
    char buf[5];
    snprintf(buf, 5, "%02X\r\n", checksum);
    strcat(cmd, buf);
    
    mt3333_is_ready(dev);
    uart_write(dev->params.uart, (uint8_t *)cmd, strlen(cmd));
    
    DEBUG("GPS command: %s\n", cmd);
}

void mt3333_set_baudrate(mt3333_t *dev, int baudrate) {
    char cmd[20] = {};
    snprintf(cmd, 20, "PMTK251,%d", baudrate);
    mt3333_send_at_command(dev, cmd);
    
    dev->params.baudrate = baudrate;
    uart_init(dev->params.uart, dev->params.baudrate, rx_cb, dev);
}

void mt3333_set_powersave(mt3333_t *dev, mt3333_powersave_mode_t mode) {
    char cmd[20] = {};
    if (mode == MT3333_POWERSAVE_STANDBY) {
        snprintf(cmd, 20, "PMTK161,0");
        mt3333_send_at_command(dev, cmd);
    } else {
        snprintf(cmd, 20, "PMTK225,0");
        mt3333_send_at_command(dev, cmd);
        snprintf(cmd, 20, "PMTK225,%d", (mode == MT3333_POWERSAVE_BACKUP)? 4:0);
        mt3333_send_at_command(dev, cmd);
    }
}

void mt3333_set_periodic(mt3333_t *dev, mt3333_powersave_mode_t mode, int run, int sleep, int run_ext, int sleep_ext) {
    char cmd[50] = {};
    snprintf(cmd, 50, "PMTK225,%d,%d,%d,%d,%d", (int)mode, run, sleep, run_ext, sleep_ext);
    mt3333_send_at_command(dev, cmd);
}

int mt3333_init(mt3333_t *dev, mt3333_param_t *param) {
	assert(dev != NULL);
	assert(param != NULL);

	/* Copy parameters */
	dev->params = *param;
    
    /* Create reader thread */
	reader_pid = thread_create(dev->reader_stack + 2*MT3333_RXBUF_SIZE_BYTES,
                                    MT3333_READER_THREAD_STACK_SIZE_BYTES - 2*MT3333_RXBUF_SIZE_BYTES,
                                    THREAD_PRIORITY_MAIN - 1, 0, reader, dev, "MT3333 reader");
	if (reader_pid <= KERNEL_PID_UNDEF) {
		return -2;
	}
    
    /* Initialize message buffer */
    nmea_buf = dev->reader_stack;    
    /* Initialize input buffer */
    rxbuf = dev->reader_stack + MT3333_RXBUF_SIZE_BYTES;

	/* Initialize the UART */
	if (uart_init(dev->params.uart, dev->params.baudrate, rx_cb, NULL)) {
		return -1;
	}

	return 0;
}

#ifdef __cplusplus
}
#endif
