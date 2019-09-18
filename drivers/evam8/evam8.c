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
 * @file		evam8.c
 * @brief       U-blox EVA-M8 GNSS module driver implementation (I2C only)
 * @author      EP <ep@unwds.com>
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "evam8.h"
#include "thread.h"
#include "assert.h"

#include "lptimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

static kernel_pid_t reader_pid = KERNEL_PID_UNDEF;
static char *nmea_buf;
static lptimer_t timer_request;
static uint32_t time_previous;

static msg_t parse_msg;

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
static bool parse_nmea(char *buf, evam8_gps_data_t *data) {
    typedef enum {
        NMEA_RMC,
        NMEA_GGA,
        NMEA_UNKNOWN,
    } nmea_message_type_t;
    
    nmea_message_type_t nmea_type = NMEA_UNKNOWN;
    
    if (memcmp(&nmea_buf[3], "RMC", 3) == 0) {
        nmea_type = NMEA_RMC;
        DEBUG("[evam8] RMC message\n");
    }
    else
    if (memcmp(&nmea_buf[3], "GGA", 3) == 0) {
        nmea_type = NMEA_GGA;
        DEBUG("[evam8] GGA message\n");
    }
    else {
        DEBUG("[evam8] Unknown message\n");
        return false;
    }

    if ((lptimer_now_msec() - time_previous) > 500) {
        memset((void*) data, 0, sizeof(evam8_gps_data_t));
    }
    time_previous = lptimer_now_msec();

    /* Check validity sign */
    char valid;
    if (((nmea_type == NMEA_RMC) && get_csv_field(buf, EVAM8_RMC_VALID_FIELD_IDX, &valid, 1)) ||
        ((nmea_type == NMEA_GGA) && get_csv_field(buf, EVAM8_GGA_VALID_FIELD_IDX, &valid, 1)))
    {
        if ((valid == 'A') || (valid == '1') || (valid == '2')) {
            data->valid = true;
            DEBUG("[evam8] Data valid\n");
        } else {
            data->valid = false;
            DEBUG("[evam8] Data not valid\n");
            data->lat = 0;
            data->lon = 0;
            data->velocity = 0;
            data->direction = 0;
            return true;
        }
    }

    char sign = 'N';
    char tmp[15];
    int s1, s2, s3;
    uint8_t field;

    if (data->lat == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = EVAM8_RMC_LAT_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = EVAM8_GGA_LAT_FIELD_IDX;
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
            
            DEBUG("[evam8] Latitude: %d\n", data->lat);
        }
    }
    
    if (data->lon == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = EVAM8_RMC_LON_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = EVAM8_GGA_LON_FIELD_IDX;
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
            
            DEBUG("[evam8] Longitude: %d\n", data->lon);
        }
    }
    
    struct tm time;

    if (data->time == 0) {
        switch (nmea_type) {
            case NMEA_RMC:
                field = EVAM8_RMC_TIME_FIELD_IDX;
                break;
            case NMEA_GGA:
                field = EVAM8_GGA_TIME_FIELD_IDX;
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
            
            DEBUG("[evam8] Time: %d:%d:%d\n", s1, s2, s3);
        }
    }
    
    /* RMC only data */
    if (nmea_type == NMEA_RMC) {
        if (get_csv_field(buf, EVAM8_RMC_VELOCITY_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2) {
                return false;
            }
            
            data->velocity = ((1000*s1 + 10*s2)*514)/1000; // mm/s
            DEBUG("[evam8] Velocity: %d mm/s\n", data->velocity);
        }
        
        if (get_csv_field(buf, EVAM8_RMC_DIRECTION_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%02d", &s1, &s2) != 2) {
                return false;
            }
            
            data->direction = (1000*s1 + 10*s2);
            DEBUG("[evam8] Direction: %d millidegrees\n", data->direction);
        }
        
        if (get_csv_field(buf, EVAM8_RMC_DATE_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%02d%02d%02d", &s1, &s2, &s3) != 3) {
                return false;
            }
            
            time.tm_mday = s1;
            time.tm_mon = s2;
            time.tm_year = 100 + s3;
        
            DEBUG("[evam8] Date: %d.%d.%d\n", s1, s2, s3);
        }
    }
    
    /* GGA only data */
    if (nmea_type == NMEA_GGA) {
        if (get_csv_field(buf, EVAM8_GGA_HEIGHT_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d.%01d", &s1, &s2) != 2) {
                return false;
            }
            
            data->height = s1*100 + s2*10; // cm
            DEBUG("[evam8] Height: %d cm\n", data->height);
        }
        
        if (get_csv_field(buf, EVAM8_GGA_SATS_FIELD_IDX, tmp, sizeof(tmp))) {
            if (sscanf(tmp, "%d", &s1) != 1) {
                return false;
            }
            
            data->satellites = s1;
            DEBUG("[evam8] Satellites: %d\n", data->satellites);
        }
    }
    
    data->time = mktime(&time);

    return true;
}

static void *reader(void *arg) {
	evam8_t *dev = (evam8_t *) arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    evam8_gps_data_t data;

    while (1) {
        msg_receive(&msg);
        
        uint16_t bytes_ready = 0;
        int res;
        
        i2c_acquire(dev->params.i2c_dev);
        res = i2c_read_regs(dev->params.i2c_dev, EVAM8_I2C_ADDR, EVAM8_I2C_REG_DATA_AVAIL, &bytes_ready, 2, 0);

        if ((res == 0) && (bytes_ready != 0)) {
            int k = 0;
            
            /* reading data in 16b chunks */
            /* if there's no more data, 0xFF will be read */
            do {
                res = i2c_read_regs(dev->params.i2c_dev, EVAM8_I2C_ADDR, EVAM8_I2C_REG_DATA_DATA, &nmea_buf[16*k], 16, 0);
                k++;
            } while ((k < EVAM8_RXBUF_SIZE_BYTES/16) && (nmea_buf[16*k - 1] != 0xFF));
            
            /* set end of string by replacing first 0xFF */
            for (int i = 0; i < 16; i++) {
                if (nmea_buf[16*(k - 1) + i] == 0xFF) {
                    nmea_buf[16*(k - 1) + i] = 0;
                    break;
                }
            }

            DEBUG("[evam8] %s\n", nmea_buf);
            
            int nmea_msg_start = 0;
            
            for (int i = 0; i < EVAM8_RXBUF_SIZE_BYTES; i++) {
                if (nmea_buf[i] == 0) {
                    break;
                }

                if (nmea_buf[i] == EVAM8_EOL) {
                    while ((nmea_buf[nmea_msg_start] != '$') && (nmea_msg_start < i)) {
                        nmea_msg_start++;
                    }
                    
                    if (nmea_msg_start < i) {
                        if (nmea_crc_check(&nmea_buf[nmea_msg_start])) {
                            dev->ready = true;
                            
                            /* parse NMEA message */
                            if ((dev->params.gps_cb) && (parse_nmea(&nmea_buf[nmea_msg_start], &data))) {
                                dev->params.gps_cb(data);
                            }
                        } else {
                            DEBUG("[evam8] NMEA CRC error\n");
                        }
                    } else {
                        DEBUG("[evam8] No NMEA message\n");
                    }
                    nmea_msg_start = i+1;
                }
            }
        }
        
        i2c_release(dev->params.i2c_dev);

        /* every 500 ms */
        lptimer_set_msg(&timer_request, 500, &parse_msg, reader_pid);
    }

    return NULL;
}

int evam8_is_ready(evam8_t *dev) {
    dev->ready = false;
    
    /* 3 seconds timeout */
    int counter = 0;
    int timeout = 3000/20;
    do {
        lptimer_sleep(20);
        counter++;
    } while ((!dev->ready) && (counter < timeout));
    
    if (dev->ready) {
        return EVAM8_READY;
    }
    
    return EVAM8_TIMEOUT;
}

void evam8_set_powersave(evam8_t *dev, evam8_powersave_mode_t mode) {
    (void)dev;
    
    char cmd[20] = {};
    if (mode == EVAM8_POWERSAVE_STANDBY) {
        snprintf(cmd, 20, "PMTK161,0");
//        evam8_send_at_command(dev, cmd);
    } else {
        snprintf(cmd, 20, "PMTK225,0");
//        evam8_send_at_command(dev, cmd);
        snprintf(cmd, 20, "PMTK225,%d", (mode == EVAM8_POWERSAVE_BACKUP)? 4:0);
//        evam8_send_at_command(dev, cmd);
    }
}

void evam8_set_periodic(evam8_t *dev, evam8_powersave_mode_t mode, int run, int sleep, int run_ext, int sleep_ext) {
    (void)dev;
    
    char cmd[50] = {};
    snprintf(cmd, 50, "PMTK225,%d,%d,%d,%d,%d", (int)mode, run, sleep, run_ext, sleep_ext);
//    evam8_send_at_command(dev, cmd);
}

int evam8_init(evam8_t *dev, evam8_param_t *param) {
	assert(dev != NULL);
	assert(param != NULL);

	/* Copy parameters */
	dev->params = *param;
    
    /* Initialize the I2C */
    i2c_acquire(dev->params.i2c_dev);
	i2c_init(dev->params.i2c_dev);
    i2c_release(dev->params.i2c_dev);
    
    /* Create reader thread */
    if (reader_pid == KERNEL_PID_UNDEF) {
        reader_pid = thread_create(dev->reader_stack + EVAM8_RXBUF_SIZE_BYTES,
                                        EVAM8_READER_THREAD_STACK_SIZE_BYTES - EVAM8_RXBUF_SIZE_BYTES,
                                        THREAD_PRIORITY_MAIN - 1, 0, reader, dev, "EVA-M8 reader");
        if (reader_pid <= KERNEL_PID_UNDEF) {
            return -2;
        }
    }
    
    /* Initialize message buffer */
    nmea_buf = dev->reader_stack;

    /* every 500 ms */
    lptimer_set_msg(&timer_request, 500, &parse_msg, reader_pid);

	return 0;
}

#ifdef __cplusplus
}
#endif
