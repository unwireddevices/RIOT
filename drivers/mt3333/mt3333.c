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
static rtctimers_millis_t timer_cback;
static uint32_t time_previous;

typedef enum {
    MT3333_MSG_PARSE,
    MT3333_MSG_CBACK,
} mt3333_int_msg_t;

static msg_t parse_msg = { .type = MT3333_MSG_PARSE };
static msg_t cback_msg = { .type = MT3333_MSG_CBACK };

static void rx_cb(void *arg, uint8_t data)
{
    (void)arg;
    static int rx_cnt;

    rxbuf[rx_cnt++] = data;
    
    /* Notify parser thread about RMC message */
    if (data == MT3333_EOL) {
        memcpy(nmea_buf, rxbuf, rx_cnt);
        nmea_buf[rx_cnt] = 0;
        msg_send(&parse_msg, reader_pid);
        rx_cnt = 0;
    }
    
    if (rx_cnt == MT3333_RXBUF_SIZE_BYTES) {
        rx_cnt = 0;
    }
}

static bool nmea_crc_check(char *buf) {
    int i = 0;
    while (buf[i] != '*') {
        if (!buf[++i]) {
            return false;
        }
    }
    
    uint8_t crc_calc = 0;
    for (int k = 1; k < i; k++) {
        crc_calc ^= (uint8_t)buf[k];
    }

    return (strtoul(&buf[i + 1], NULL, 16) == crc_calc);
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
static bool parse_nmea(char *buf, mt3333_gps_data_t *data) {
    typedef enum {
        NMEA_RMC,
        NMEA_GGA,
        NMEA_UNKNOWN,
    } nmea_message_type_t;
    
    nmea_message_type_t nmea_type = NMEA_UNKNOWN;
    
    if (memcmp(&nmea_buf[3], "RMC", 3) == 0) {
        nmea_type = NMEA_RMC;
        DEBUG("[mt3333] RMC message\n");
    }
    else
    if (memcmp(&nmea_buf[3], "GGA", 3) == 0) {
        nmea_type = NMEA_GGA;
        DEBUG("[mt3333] GGA message\n");
    }
    else {
        DEBUG("[mt3333] Unknown message\n");
        return false;
    }

    if ((rtctimers_millis_now() - time_previous) > 500) {
        memset((void*) data, 0, sizeof(mt3333_gps_data_t));
    }
    time_previous = rtctimers_millis_now();

    /* Check validity sign */
    char valid;
    if (((nmea_type == NMEA_RMC) && get_csv_field(buf, MT3333_RMC_VALID_FIELD_IDX, &valid, 1)) ||
        ((nmea_type == NMEA_GGA) && get_csv_field(buf, MT3333_GGA_VALID_FIELD_IDX, &valid, 1)))
    {
        if ((valid == 'A') || (valid == '1') || (valid == '2')) {
            data->valid = true;
            DEBUG("[mt3333] Data valid\n");
        } else {
            data->valid = false;
            DEBUG("[mt3333] Data not valid\n");
        }
    }

    char sign = 'N';
    char tmp[15];
    int s1, s2, s3;
    uint8_t field;

    if (data->lat == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = MT3333_RMC_LAT_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = MT3333_GGA_LAT_FIELD_IDX;
                break;
            default:
                field = 0;
                break;
        }
        
        if (field) {
            get_csv_field(buf, field, tmp, sizeof(tmp));
            get_csv_field(buf, field + 1, &sign, 1);
            
            if (sscanf(tmp, "%d.%04d", &s1, &s2) != 2) {
                return false;
            }

            data->lat = (s1/100);
            s1 -= data->lat * 100;
            data->lat *= 1000000;
            data->lat += (1000000*s1)/60;
            data->lat += (10*s2)/6;
            
            if (sign == 'S') {
                data->lat = -data->lat;
            }
            
            DEBUG("[mt3333] Latitude: %d\n", data->lat);
        }
    }
    
    if (data->lon == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = MT3333_RMC_LON_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = MT3333_GGA_LON_FIELD_IDX;
                break;
            default:
                field = 0;
                break;
        }
        
        if (field) {
            get_csv_field(buf, field, tmp, sizeof(tmp));
            get_csv_field(buf, field + 1, &sign, 1);

            if (sscanf(tmp, "%d.%04d", &s1, &s2) != 2) {
                return false;
            }
            
            data->lon = (s1/100);
            s1 -= data->lon * 100;
            data->lon *= 1000000;
            data->lon += (1000000*s1)/60;
            data->lon += (10*s2)/6;
            
            if (sign == 'W') {
                data->lon = -data->lon;
            }
            
            DEBUG("[mt3333] Longitude: %d\n", data->lon);
        }
    }
    
    struct tm time;

    if (data->time == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = MT3333_RMC_TIME_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = MT3333_GGA_TIME_FIELD_IDX;
                break;
            default:
                field = 0;
                break;
        }
        
        if (field) {
            get_csv_field(buf, field, tmp, sizeof(tmp));
            
            if (sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3) != 3) {
                return false;
            }
            
            time.tm_hour = s1;
            time.tm_min = s2;
            time.tm_sec = 100 + s3;
            
            DEBUG("[mt3333] Time: %d:%d:%d\n", s1, s2, s3);
        }
    }
    
    /* RMC only data */
    if (nmea_type == NMEA_RMC) {
        if (get_csv_field(buf, MT3333_RMC_VELOCITY_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2) {
                return false;
            }
            
            data->velocity = ((1000*s1 + 10*s2)*514)/1000; // mm/s
            DEBUG("[mt3333] Velocity: %d mm/s\n", data->velocity);
        }
        
        if (get_csv_field(buf, MT3333_RMC_DIRECTION_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2) {
                return false;
            }
            
            data->direction = (1000*s1 + 10*s2);
            DEBUG("[mt3333] Direction: %d millidegrees\n", data->direction);
        }
        
        if (get_csv_field(buf, MT3333_RMC_DATE_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3) != 3) {
                return false;
            }
            
            time.tm_mday = s1;
            time.tm_mon = s2;
            time.tm_year = 100 + s3;
        
            DEBUG("[mt3333] Date: %d.%d.%d\n", s1, s2, s3);
        }
    }
    
    /* GGA only data */
    if (nmea_type == NMEA_GGA) {
        if (get_csv_field(buf, MT3333_GGA_HEIGHT_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%01d", &s1, &s2) != 2) {
                return false;
            }
            
            data->height = s1*100 + s2*10; // cm
            DEBUG("[mt3333] Height: %d cm\n", data->height);
        }
        
        if (get_csv_field(buf, MT3333_GGA_SATS_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d", &s1) != 1) {
                return false;
            }
            
            data->satellites = s1;
            DEBUG("[mt3333] Satellites: %d\n", data->satellites);
        }
    }
    
    data->time = mktime(&time);

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
        
        switch (msg.type) {
            case MT3333_MSG_PARSE:
                DEBUG("[mt3333] %s", nmea_buf);
        
                if (nmea_crc_check(nmea_buf)) {
                    dev->ready = true;
                    
                    /* parse NMEA message */
                    if ((dev->params.gps_cb) && (parse_nmea(nmea_buf, &data))) {
                        /* 500 ms delay allows to parse multiple NMEA messages from the same packet */
                        rtctimers_millis_set_msg(&timer_cback, 500, &cback_msg, reader_pid);
                    }
                } else {
                    DEBUG("[mt3333] NMEA CRC error\n");
                }
                break;
            case MT3333_MSG_CBACK:
                dev->params.gps_cb(data);
                break;
            default:
                break;
        }
    }

    return NULL;
}

int mt3333_is_ready(mt3333_t *dev) {
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
    if (mt3333_is_ready(dev) == MT3333_TIMEOUT) {
        DEBUG("[mt3333] GPS not ready\n");
        return;
    }

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

    uart_write(dev->params.uart, (uint8_t *)cmd, strlen(cmd));
    
    DEBUG("[mt3333] command: %s\n", cmd);
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
