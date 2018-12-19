/*
 * Copyright (C) 2017-2018 Unwired Devices [info@unwds.com]
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
 * @file
 * @brief
 * @author      Katerina Gorinskaya
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#include <stdio.h>
#include <random.h>
#include "xtimer.h"
#include "checksum/ucrc16.h"
#include "hashes/sha256.h"
#include "string.h"
#include "thread.h"
#include "periph/adc.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/cpuid.h"

#include "ps.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define SOILSENSOR_STACK_SIZE       768

#define ADC_MOIST                   0
#define ADC_TEMP                    1
#define ADC_VREF                    2
#define MIN_MOISTURE                500
#define MAX_MOISTURE                1800
#define RE_PIN                      1
#define DE_PIN                      2
#define UART_1                      1

#define ADDRESS_SIZE                8                       // bytes 0-7 are for the device address
#define OFFSET_TYPE                 ADDRESS_SIZE            // byte 8 is device type
#define OFFSET_BYTE_MOISTURE        ADDRESS_SIZE + 1        // byte 9 is for moisture
#define OFFSET_BYTE_TEMP            ADDRESS_SIZE + 2        // byte 10 is for temperature
#define OFFSET_BYTE_VOLT            ADDRESS_SIZE + 3        // byte 10 is for temperature
#define OFFSET_BYTE_CRC             ADDRESS_SIZE + 4        // byte 11 is for CRC
#define NUMBER_BYTE_CRC             2
#define BUF_SIZE                    ADDRESS_SIZE + NUMBER_BYTE_CRC + 4

#define SEED_CRC                    42

enum {
    TYPE_NODATA         = 0,
    TYPE_ASK            = 1,
    TYPE_ADDRESS        = 2,
    TYPE_SOIL_SENSOR    = 3,
} data_types_t;

static uint8_t address_uart[ADDRESS_SIZE] = {};

static uint8_t buf_in[BUF_SIZE] = {};     /* buffer for the received data */

static uint8_t num_bytes_received = 0;
static msg_t send_msg;
static kernel_pid_t process_pid;

typedef struct {
    uint8_t moisture;
    uint8_t temperature;
    uint8_t voltage;
} sensor_data_t;

static uint32_t time_from_start, time_from_last_byte;

/* Flushing buffer before receiving data  */
static void flush_uart_buf(void)
{
    num_bytes_received = 0;
}

/* Reading moisture and temperature value  */
static sensor_data_t readval(void)
{
    int sum1 = 0, sum2 = 0;
    int moisture = 0, temp = 0;
    int n = 5;
    for (int i = 0; i < n; i++) {
        adc_init(ADC_MOIST);
        moisture = adc_sample(ADC_MOIST, ADC_RES_12BIT);
        
        adc_init(ADC_TEMP);
        temp = adc_sample(ADC_TEMP, ADC_RES_12BIT);
        sum1 += moisture;
        sum2 += temp;
    }
    moisture = (100*sum1)/n;
    
    /* Vref ADC channel */
    adc_init(ADC_VREF);
    int vdda = adc_sample(ADC_VREF, ADC_RES_12BIT);
    printf("Supply voltage: %d mV\n", vdda);
    
    moisture -= (100*MIN_MOISTURE);
    moisture /= (MAX_MOISTURE - MIN_MOISTURE);
    moisture = 100 - moisture;
    
    if (moisture > 100) {
        moisture = 100;
    }
    
    if (moisture < 0) {
        moisture = 0;
    }
    
    printf("Moisture: %d %%\n", moisture);
    
    temp = (10*vdda*sum2)/(4096 * n); /* mV*10 */
    
    temp -= 5000;
    temp /= 100;
    
    printf("Temperature: %d C\n", temp);
    
    sensor_data_t data;
    data.voltage = vdda/25;
    data.moisture = moisture;
    data.temperature = 50 + temp;
    
    return data;
}

/* Verifying if a request for an address has come  */
static int request_for_address(uint8_t *buffer)
{
    uint8_t address_request[BUF_SIZE];
    memset(address_request, 0x11, BUF_SIZE - 2);
    
    uint16_t crc = ucrc16_calc_be(buffer, BUF_SIZE-NUMBER_BYTE_CRC, UCRC16_CCITT_POLY_BE, SEED_CRC);
    address_request[OFFSET_BYTE_CRC] = crc & 0xff;
    address_request[OFFSET_BYTE_CRC + 1] = crc >> 8;
    
    return memcmp(buffer, address_request, BUF_SIZE);
}

volatile uint8_t byte_received = 0;
volatile uint8_t last_byte_value = 0;
volatile uint8_t ignore_rx_data = 0;

static int collision_detect(uint8_t *data)
{
    flush_uart_buf();
    
    ignore_rx_data = 1;
    
    for (uint8_t i = 0; i < BUF_SIZE; i++) {
        byte_received = 0;
        uart_write(UART_1, &data[i], 1);
        
        int count = 0;
        while (!byte_received) {
            xtimer_usleep(100);
            if (++count > 10) {
                puts("No echo detected");
                return 1;
            }
        }

        if (last_byte_value != data[i]) {
            printf("Collision detected, %02x vs %02x", data[i], last_byte_value);
            /* random delay 30 to 300 ms */
            xtimer_usleep(1000*random_uint32_range(30, 300));

            ignore_rx_data = 0;
            return 1;
        }
    }

    ignore_rx_data = 0;
    return 0;
}

/* Collision prevention */
static void send_data(uint8_t *data, bool wait)
{
    /* wait for other transfers to finish */
    if (wait)
    {
        while (xtimer_now_usec() < time_from_last_byte + 30000)
        {
            xtimer_usleep(1000);
        }
    }

    printf("Data: 0x");
    for (uint8_t i = 0; i < BUF_SIZE; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
    
    while (collision_detect(data)) { }
}

/* Verifying сhecksum  */
static int verify_crc (uint8_t data [])
{
    uint16_t crc_uint16_t = ucrc16_calc_be(data, BUF_SIZE-NUMBER_BYTE_CRC, UCRC16_CCITT_POLY_BE, SEED_CRC);
    uint8_t crc1 = crc_uint16_t & 0xff;
    uint8_t crc2 = crc_uint16_t >> 8;
    if ((crc1 != data[OFFSET_BYTE_CRC]) || (crc2 != data[OFFSET_BYTE_CRC+1])) {
        printf("\nError: wrong CRC \n");
        return 0;
    }
    else return 1;
}

static void prepare_data(uint8_t *buf)
{
    sensor_data_t data = readval();
    
    memcpy(buf, address_uart, ADDRESS_SIZE);
    
    buf[OFFSET_TYPE] = TYPE_SOIL_SENSOR;
    buf[OFFSET_BYTE_MOISTURE] = data.moisture;
    buf[OFFSET_BYTE_TEMP] = data.temperature;
    buf[OFFSET_BYTE_VOLT] = data.voltage;
    
    uint16_t crc = ucrc16_calc_be(buf, BUF_SIZE-NUMBER_BYTE_CRC, UCRC16_CCITT_POLY_BE, SEED_CRC);
    buf[OFFSET_BYTE_CRC] = crc & 0xff;
    buf[OFFSET_BYTE_CRC+1] = crc >> 8;
}

static void *processing_thread(void *arg)
{
    (void)arg;
    
    uint8_t buf_out[BUF_SIZE];
    msg_t msg;
    
    while (1) {
        msg_receive(&msg);
        
        printf("Received: 0x");
        for (int i = 0; i<BUF_SIZE; i++) {
            printf ("%02X", buf_in[i]);
        }
        printf("\n");
        
        if (!verify_crc(buf_in)) {
            puts("CRC error");
            continue;
        }
        
        /* Broadcast request for device address */
        if (request_for_address(buf_in) == 1) {
            puts("Broadcast request for device address");
            
            /* replying with data is ok */
            prepare_data(buf_out);
            
            /* random delay 30 to 300 ms */
            xtimer_usleep(1000*random_uint32_range(30, 300));
            send_data(buf_out, true);
            
            puts("Address successfully sent");
            
            continue;
        }
        
        /* Verifying if destination address is ours */
        if (memcmp(buf_in, address_uart, ADDRESS_SIZE)) {
            puts("Address mismatch, ignoring packet");
            continue;
        }
        
        if (buf_in[OFFSET_TYPE] == TYPE_ASK) {
            puts("Data requested");
            prepare_data(buf_out);
            send_data(buf_out, false);
            puts("Data successfully sent");
        } else {
            puts("Unknown request code, ignoring packet");
        }
    }
    return NULL;
}

/* UART IRQ callback */
static void uart_input(void *arg, uint8_t data)
{
    (void)arg;
    
    if (ignore_rx_data) {
        last_byte_value = data;
        byte_received = 1;
        return;
    }
    
    time_from_start = xtimer_now_usec();

    // Time to receive one byte is ~20ms, if more => error
    if (time_from_start > time_from_last_byte + (30000)) {
        flush_uart_buf();
    }
    
    time_from_last_byte = xtimer_now_usec();
    buf_in[num_bytes_received] = data;
    num_bytes_received++;
    
    if (num_bytes_received >= BUF_SIZE) {
        num_bytes_received = 0;
        msg_send(&send_msg, process_pid);
    }
    
    return;
}

int main(void)
{
    int period = 10;
    
    puts("*****************************");
    puts("72 MHz capacitive soil moisture sensor");
    puts("Firmware ver. 1.00");
    puts("(c) 2018 Unwired Devices LLC");
    
    xtimer_init();
    
    /* generate 64-bit address */
    uint8_t cpuid[CPUID_LEN];
    cpuid_get(cpuid);
    memcpy(address_uart, sha256((uint8_t *)cpuid, CPUID_LEN, NULL), 8);

    uint32_t *add_ptr = (uint32_t *)address_uart;
    random_init(*add_ptr);
    
    printf("64-bit device address: 0x");
    for (int i = 0; i < ADDRESS_SIZE; i++) {
        printf("%02X", address_uart[i]);
    }
    printf("\n");
    printf("Measurement period: %d s\n", period);
    
    puts("*****************************");
    
    /* enable MCO */
    gpio_init_af(GPIO_PIN(PORT_A, 8), GPIO_AF0);

    uart_init(UART_1, 9600, uart_input, NULL);
    
    static char stack[SOILSENSOR_STACK_SIZE];
    process_pid = thread_create(stack, SOILSENSOR_STACK_SIZE, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, processing_thread, NULL, "data");
    
    while(1) {
        if (period) {
            xtimer_sleep(period);
            puts("Sending data without request");
            
            uint8_t buf_out[BUF_SIZE];
            prepare_data(buf_out);
            send_data(buf_out, true);
            
            puts("Data successfully sent");
            ps();
        } else {
            xtimer_sleep(1);
        }
    }
    return 0;
}