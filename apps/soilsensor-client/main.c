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
 * @file
 * @brief
 * @author      Katerina Gorinskaya
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#include <stdio.h>
#include <random.h>
#include "xtimer.h"
#include "checksum/ucrc16.h"
#include "luid.h"
#include "string.h"
#include "thread.h"
#include "periph/adc.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/flashpage.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define SOILSENSOR_STACK_SIZE       768

#define ADC_MOIST                   0
#define ADC_TEMP                    1
#define ADC_VREF                    2

#define RE_PIN                      1
#define DE_PIN                      2
#define UART_1                      1

#define START_BYTE                  0x55 // byte 0
#define ADDRESS_SIZE                8   // bytes 1-8 are for the device address
#define OFFSET_TYPE                 9   // byte 9 is device type
#define OFFSET_CMD                  10  // byte 10 is command code
#define OFFSET_BYTE_MOISTURE        11  // byte 11 is for moisture
#define OFFSET_BYTE_TEMP            12  // byte 12 is for temperature
#define OFFSET_BYTE_CRC             13  // bytes 13-14 is for CRC
#define CRC_SIZE                    2
#define BUF_SIZE                    OFFSET_BYTE_CRC + CRC_SIZE

#define SEED_CRC                    42

enum {
    SOILSENSOR_CMD_DATA     = 0,
    SOILSENSOR_CMD_DATAREQ  = 1,
} soilsensor_commands_t;

enum {
    TYPE_NODATA         = 0,
    TYPE_ASK            = 1,
    TYPE_ADDRESS        = 2,
    TYPE_SOIL_SENSOR    = 3,
} data_types_t;

static uint64_t address_uart;

static uint8_t buf_in[BUF_SIZE] = {};     /* buffer for the received data */

static uint8_t num_bytes_received = 0;
static msg_t send_msg;
static kernel_pid_t process_pid;

typedef struct {
    uint32_t magic;
    int moisture_max;
    int moisture_min;
} sensor_settings_t;

static sensor_settings_t sensor_settings = { 0xCAFEBABE, 0, 0 };

typedef struct {
    int moisture_raw;
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
    sensor_data_t data;
    
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
    
    data.moisture_raw = moisture/100;
    
    /* Vref ADC channel */
    adc_init(ADC_VREF);
    int vdda = adc_sample(ADC_VREF, ADC_RES_12BIT);
    printf("VDD: %d mV\n", vdda);

    moisture -= (100*sensor_settings.moisture_min);
    moisture /= (sensor_settings.moisture_max - sensor_settings.moisture_min);
    
    if (moisture > 100) {
        moisture = 100;
    }
    
    if (moisture < 0) {
        moisture = 0;
    }
#if ENABLE_DEBUG
    printf("Moisture: %d %% (%d)\n", moisture, data.moisture_raw);
#else
    printf("Moisture: %d %%\n", moisture);
#endif
    
    temp = (10*vdda*sum2)/(4096 * n); /* mV*10 */
    
    temp -= 5000;
    temp /= 100;
    
    printf("Temperature: %d C\n", temp);
    
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
    
    uint16_t crc = ucrc16_calc_be(buffer, BUF_SIZE - CRC_SIZE, UCRC16_CCITT_POLY_BE, SEED_CRC);
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
                /* puts("No echo detected"); */
                return 1;
            }
        }

        if (last_byte_value != data[i]) {
            printf("Collision: %02x vs %02x", data[i], last_byte_value);
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

#if ENABLE_DEBUG
    printf("Data: 0x");
    for (uint8_t i = 0; i < BUF_SIZE; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
#endif
    
    int i = 0;
    while (collision_detect(data) && (++i < 10000)) { }
}

/* Verifying сhecksum  */
static int verify_crc (uint8_t data [])
{
    uint16_t crc_uint16_t = ucrc16_calc_be(data, BUF_SIZE - CRC_SIZE, UCRC16_CCITT_POLY_BE, SEED_CRC);
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
    
    buf[0] = START_BYTE;
    memcpy(&buf[1], (void *)&address_uart, sizeof(address_uart));
    buf[OFFSET_TYPE] = TYPE_SOIL_SENSOR;
    buf[OFFSET_CMD] = SOILSENSOR_CMD_DATA;
    buf[OFFSET_BYTE_MOISTURE] = data.moisture;
    buf[OFFSET_BYTE_TEMP] = data.temperature;
    
    uint16_t crc = ucrc16_calc_be(buf, BUF_SIZE - CRC_SIZE, UCRC16_CCITT_POLY_BE, SEED_CRC);
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
        
#if ENABLE_DEBUG
        printf("Received: 0x");
        for (int i = 0; i<BUF_SIZE; i++) {
            printf ("%02X", buf_in[i]);
        }
        printf("\n");
#endif
        
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
            
            puts("Address sent");
            
            continue;
        }
        
        /* Verifying if destination address is ours */
        if (memcmp(buf_in, (void *)&address_uart, ADDRESS_SIZE)) {
            puts("Address mismatch, ignoring");
            continue;
        }
        
        if (buf_in[OFFSET_TYPE] == TYPE_ASK) {
            puts("Data requested");
            prepare_data(buf_out);
            send_data(buf_out, false);
            puts("Data sent");
        } else {
            puts("Unknown code, ignoring");
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
/*
static int min_cal(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    sensor_data_t data = readval();
    sensor_settings.moisture_min = data.moisture_raw;
    printf("Min value: %d\n", sensor_settings.moisture_min);
    return 0;
}

static int max_cal(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    sensor_data_t data = readval();
    sensor_settings.moisture_max = data.moisture_raw;
    printf("Max value: %d\n", sensor_settings.moisture_max);
    return 0;
}

static int save(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    sensor_settings.magic = 0xCAFEBABE;
    flashpage_write(cpu_status.flash.pages - 1, (void *)&sensor_settings, sizeof(sensor_settings));
    
    puts("Calibration saved permanently");
    return 0;
}

static int get_data(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    uint8_t buf_out[BUF_SIZE];
    prepare_data(buf_out);
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "get", "get data", get_data },
    { "min", "min value cal", min_cal },
    { "max", "max value cal", max_cal },
    { "save", "save calibration", save },
    { NULL, NULL, NULL }
};
*/
int main(void)
{
    puts("FW v. 1.00 / STM32F030");
    puts("(c) 2019 Unwired Devices LLC");
 
    xtimer_init();

    /* generate 64-bit address based on CPUID */    
    luid_get((void *)&address_uart, sizeof(address_uart));
    
    printf("DevAddr: 0x%16llx\n", address_uart);
    
#if defined(CPU_LINE_STM32F051x8)
    cpu_status.flash.pages = 32;
    cpu_status.flash.pagesize = 1024;
    cpu_status.flash.size = 32768;
    
    /* enable MCO */
    gpio_init_af(GPIO_PIN(PORT_A, 8), GPIO_AF0);
#elif defined(CPU_LINE_STM32F030x8) || defined(CPU_LINE_STM32F070xB)
    cpu_status.flash.pages = 16;
    cpu_status.flash.pagesize = 1024;
    cpu_status.flash.size = 16384;
    
    /* setup TIM14 */
#else
    #error Unsupported CPU model
#endif
    
    bool calibration = false;
    flashpage_read(cpu_status.flash.pages - 1, (void *)&sensor_settings, sizeof(sensor_settings));
    
    if (sensor_settings.magic == 0xCAFEBABE) {
        puts("Sensor already calibrated");
        calibration = true;
    }
    
    puts("*****************************");
    
    if (calibration) {
        uart_init(UART_1, 9600, uart_input, NULL);
        
        static char stack[SOILSENSOR_STACK_SIZE];
        process_pid = thread_create(stack, SOILSENSOR_STACK_SIZE, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, processing_thread, NULL, "data");
        
        puts("Sending data once");
        xtimer_sleep(3);
        uint8_t buf_out[BUF_SIZE];
        prepare_data(buf_out);
        send_data(buf_out, true);
        
        puts("Data sent");
    } else {
        puts("(!!!) UNCALIBRATED SENSOR (!!!)");
    }
  
    while (1) {};
    
    return 0;
}