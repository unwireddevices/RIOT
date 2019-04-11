/*
 * Copyright (C) 2016-2019 Unwired Devices LLC <info@unwds.com>

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
#include "periph/pwm.h"
#include "periph/pm.h"
#include "pm_layered.h"
#include "periph/flashpage.h"
#include "stdio_uart.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define SOILSENSOR_STACK_SIZE       768

#define ADC_MOIST                   0
#define ADC_TEMP                    1
#define ADC_VREF                    2

#define SOILSENSOR_COMM_UART        1
#define SOILSENSOR_PWM_DEV          0

#define SOILSENSOR_COMM_DEF_BAUD    9600
#define SOILSENSOR_COMM_DEF_PERIOD  15

#define SOILSENSOR_SENSOR_WARMUP    3

#define START_BYTE                  0x55 // byte 0
#define ADDRESS_SIZE                8   // bytes 1-8 are for the device address
#define OFFSET_TYPE                 9   // byte 9 is device type
#define OFFSET_CMD                  10  // byte 10 is command code
#define OFFSET_SIZE                 11  // byte 11 is data size
#define OFFSET_BYTE_MOISTURE        12  // byte 12 is for moisture
#define OFFSET_BYTE_TEMP            13  // byte 13 is for temperature

#define CRC_SIZE                    2

#define BUF_SIZE                    50

#define SEED_CRC                    42

enum {
    SOILSENSOR_CMD_DATA     = 0,
    SOILSENSOR_CMD_DATAREQ  = 1,
    SOILSENSOR_CMD_SETTINGS = 2,
} soilsensor_commands_t;

enum {
    TYPE_SOIL_SENSOR    = 1,
} data_types_t;

static uint64_t address_uart;

static uint8_t buf_in[BUF_SIZE] = {};     /* buffer for the received data */

static volatile uint8_t num_bytes_received = 0;
static msg_t send_msg;
static kernel_pid_t process_pid;
static xtimer_t uart_timer;

typedef struct {
    uint32_t magic;
    uint32_t baud;
    uint16_t period;
} sensor_settings_t;

static sensor_settings_t sensor_settings = { 0xCAFEBABE, 0, 0 };

typedef struct {
    int moisture_raw;
    uint8_t moisture;
    uint8_t temperature;
    uint8_t voltage;
} sensor_data_t;

static void cpu_switch_idle(bool idle) {
    /* wait for UART to finish current operation */
    while ((uart_config[SOILSENSOR_COMM_UART].dev->ISR & USART_ISR_BUSY) ||
           (!(uart_config[SOILSENSOR_COMM_UART].dev->ISR & USART_ISR_TC))) {}

    uint32_t irqs = irq_disable();
    uint32_t reg = RCC->CFGR;
    reg &= ~RCC_CFGR_HPRE_Msk;
    
    if (idle) {
        /* switch core to 48/16 = 3 MHz */
        cpu_status.clock.coreclock = 48000000/16;
        reg |= RCC_CFGR_HPRE_DIV16;
    } else {
        /* switch back to 48 MHz */
        cpu_status.clock.coreclock = 48000000;
        reg |= RCC_CFGR_HPRE_DIV1;
    }
    RCC->CFGR = reg;

    /* recalculate XTIMER clock */
    xtimer_init();

    /* recalculate UART baudrate */
    stdio_init();
    uart_set_baudrate(SOILSENSOR_COMM_UART, sensor_settings.baud);
    
    irq_restore(irqs);
}

/* Reading moisture and temperature value  */
static sensor_data_t readval(void)
{
    sensor_data_t data;
    
    int moisture = 0, temp = 0;
    int n = 10;
    
    adc_init(ADC_MOIST);
    for (int i = 0; i < n; i++) {
        moisture += adc_sample(ADC_MOIST, ADC_RES_12BIT);
        xtimer_usleep(1000);
    }
    
    adc_init(ADC_TEMP);
    for (int i = 0; i < n; i++) {
        temp += adc_sample(ADC_TEMP, ADC_RES_12BIT);
        xtimer_usleep(1000);
    }
    
    moisture = (100*moisture)/n;
    
    data.moisture_raw = moisture/100;
    
    /* Vref ADC channel */
    adc_init(ADC_VREF);
    int vdda = 0;
    for (int i = 0; i < n; i++) {
        vdda += adc_sample(ADC_VREF, ADC_RES_12BIT);
        xtimer_usleep(1000);
    }
    vdda /= n;
    printf("VDD: %d mV\n", vdda);

    /*
    moisture -= (100*sensor_settings.moisture_min);
    moisture /= (sensor_settings.moisture_max - sensor_settings.moisture_min);
    */
    
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
    
    temp = (10*vdda*temp)/(4096 * n); /* mV*10 */
    
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
    uint8_t address_request[OFFSET_SIZE + 1] = { 0xFF };
    
    address_request[0] = START_BYTE;
    address_request[OFFSET_TYPE] = 0;
    address_request[OFFSET_CMD] = 0;
    address_request[OFFSET_SIZE] = 0;
    
    return memcmp(buffer, address_request, OFFSET_SIZE + 1);
}

volatile uint8_t byte_received = 0;
volatile uint8_t last_byte_value = 0;
volatile uint8_t ignore_rx_data = 0;

static int collision_detect(uint8_t *data)
{
    ignore_rx_data = 1;
    
    for (uint8_t i = 0; i < OFFSET_SIZE + data[OFFSET_SIZE] + 1 + 2; i++) {
        byte_received = 0;
        uart_write(SOILSENSOR_COMM_UART, &data[i], 1);
        
        int count = 0;
        while (!byte_received) {
            xtimer_usleep(100);
            if (++count > 10) {
                DEBUG("No echo detected\n");
                return 1;
            }
        }

        if (last_byte_value != data[i]) {
            DEBUG("Collision: %02x vs %02x\n", data[i], last_byte_value);
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
static void send_data(uint8_t *data)
{
    while (1) {
        /* 8 symbols timeout */
        uint32_t timeout = 80000000 / sensor_settings.baud;
        xtimer_usleep(timeout);
        
        if (num_bytes_received == 0) {
            break;
        }
    }

#if ENABLE_DEBUG
    printf("Data: 0x");
    for (uint8_t i = 0; i < OFFSET_SIZE + data[OFFSET_SIZE] + 1 + 2; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
#endif
    
    int i = 0;
    while (collision_detect(data) && (++i < 10000)) { }
}

/* Verifying сhecksum  */
static int verify_crc(uint8_t data [])
{
    int offset = OFFSET_SIZE + data[OFFSET_SIZE] + 1;

    uint16_t crc = ucrc16_calc_le(data, offset, UCRC16_CCITT_POLY_LE, SEED_CRC);
    
    if (memcmp(&data[offset], (void *)&crc, 2)) {
        printf("\nError: wrong CRC \n");
        return -1;
    } else {
        return 0;
    }
}

static void prepare_data(uint8_t *buf)
{
    cpu_switch_idle(false);
    pwm_start(SOILSENSOR_PWM_DEV, 0);
    
    xtimer_sleep(SOILSENSOR_SENSOR_WARMUP);
    sensor_data_t data = readval();
    pwm_stop(SOILSENSOR_PWM_DEV, 0);
    
    cpu_switch_idle(true);
    
    buf[0] = START_BYTE;
    memcpy(&buf[1], (void *)&address_uart, sizeof(address_uart));
    buf[OFFSET_TYPE] = TYPE_SOIL_SENSOR;
    buf[OFFSET_CMD] = SOILSENSOR_CMD_DATA;
    buf[OFFSET_SIZE] = 2; /* 2 bytes of data */
    buf[OFFSET_BYTE_MOISTURE] = data.moisture;
    buf[OFFSET_BYTE_TEMP] = data.temperature;

    uint16_t crc = ucrc16_calc_le(buf, OFFSET_SIZE + buf[OFFSET_SIZE] + 1, UCRC16_CCITT_POLY_LE, SEED_CRC);
    memcpy(&buf[OFFSET_SIZE + buf[OFFSET_SIZE] + 1], (void *)&crc, 2);
}

static void save_settings(void)
{
    flashpage_write(cpu_status.flash.pages - 1, (void *)&sensor_settings, sizeof(sensor_settings));
    puts("Config saved to flash");
}

static void *processing_thread(void *arg)
{
    (void)arg;
    
    uint8_t buf_out[BUF_SIZE];
    msg_t msg;
    
    while (1) {
        msg_receive(&msg);
        num_bytes_received = 0;
        
#if ENABLE_DEBUG
        printf("Received: 0x");
        for (int i = 0; i < OFFSET_SIZE + buf_in[OFFSET_SIZE] + 1 + 2; i++) {
            printf ("%02X", buf_in[i]);
        }
        printf("\n");
#endif
        
        if (verify_crc(buf_in) != 0) {
            puts("CRC error");
            continue;
        }
        
        /* Broadcast request for device address */
        if (request_for_address(buf_in) == 0) {
            puts("Broadcast request for device address");
            
            /* replying with data is ok */
            prepare_data(buf_out);
            
            /* random delay 30 to 300 ms */
            xtimer_usleep(1000*random_uint32_range(30, 300));
            send_data(buf_out);
            
            puts("Address sent");
            
            continue;
        }
        
        /* Verifying if destination address is ours */
        if (memcmp(buf_in, (void *)&address_uart, ADDRESS_SIZE)) {
            puts("Address mismatch, ignoring");
            continue;
        }
        
        if (buf_in[OFFSET_TYPE] == TYPE_SOIL_SENSOR) {
            switch (buf_in[OFFSET_TYPE]) {
                case SOILSENSOR_CMD_DATAREQ: {
                    puts("Data requested");
                    prepare_data(buf_out);
                    send_data(buf_out);
                    puts("Data sent");
                    break;
                }
                case SOILSENSOR_CMD_SETTINGS: {
                    sensor_settings.magic = 0xCAFEBABE;
                    
                    uint32_t baud;
                    memcpy((void *)&baud, &buf_in[OFFSET_SIZE + 1], 4);
                    if ((baud == 1200)  || (baud == 2400)  || (baud == 9600) ||
                        (baud == 19200) || (baud == 57600) || (baud == 115200)) {
                        sensor_settings.baud = baud;
                    }
                    memcpy((void *)&sensor_settings.period, &buf_in[OFFSET_SIZE + 1 + 4], 2);
                    save_settings();
                    
                    buf_out[0] = START_BYTE;
                    memcpy(&buf_out[1], (void *)&address_uart, sizeof(address_uart));
                    buf_out[OFFSET_TYPE] = TYPE_SOIL_SENSOR;
                    buf_out[OFFSET_CMD] = SOILSENSOR_CMD_SETTINGS;
                    buf_out[OFFSET_SIZE] = 6; /* 6 bytes of data */

                    memcpy(&buf_out[OFFSET_SIZE + 1], (void *)&sensor_settings.baud, 4);
                    memcpy(&buf_out[OFFSET_SIZE + 1 + 4], (void *)&sensor_settings.period, 2);

                    uint16_t crc = ucrc16_calc_le(buf_out, OFFSET_SIZE + 1 + 6, UCRC16_CCITT_POLY_LE, SEED_CRC);
                    memcpy(&buf_out[OFFSET_SIZE + 1 + 6], (void *)&crc, 2);
                    
                    send_data(buf_out);
                    
                    uart_set_baudrate(SOILSENSOR_COMM_UART, sensor_settings.baud);
                }
            }
        } else {
            puts("Device type mismatch");
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
    
    num_bytes_received++;
    
    /* buffer overflow */
    if (num_bytes_received == BUF_SIZE) {
        num_bytes_received = 0;
        xtimer_remove(&uart_timer);
    } else {    
        buf_in[num_bytes_received] = data;

        /* 5 symbols timeout */
        uint32_t timeout = 50000000 / sensor_settings.baud;
        xtimer_set_msg(&uart_timer, timeout, &send_msg, process_pid);
    }
    
    return;
}

int main(void)
{
    puts("FW v. 1.00 / STM32F070");
    puts("(c) 2019 Unwired Devices LLC");
 
    xtimer_init();

    /* generate 64-bit address based on CPUID */    
    luid_get((void *)&address_uart, sizeof(address_uart));
    
    printf("DevAddr: 0x%08lx%08lx\n", (uint32_t)(address_uart >> 32), (uint32_t)(address_uart & 0xFFFFFFFF));
    
#if defined(CPU_LINE_STM32F051x8)
    /* enable MCO */
    gpio_init_af(GPIO_PIN(PORT_A, 8), GPIO_AF0);
#elif defined(CPU_LINE_STM32F030x8) || defined(CPU_LINE_STM32F070xB)
    /* setup 24 MHz output to feed the sensor */
    pwm_init(SOILSENSOR_PWM_DEV, PWM_RIGHT, 24000000, 2);
    pwm_set(SOILSENSOR_PWM_DEV, 0, 1);
#else
    #error Unsupported CPU model
#endif
    
    bool calibration = false;
    flashpage_read(cpu_status.flash.pages - 1, (void *)&sensor_settings, sizeof(sensor_settings));
    
    if (sensor_settings.magic != 0xCAFEBABE) {
        sensor_settings.magic = 0xCAFEBABE;
        sensor_settings.baud = SOILSENSOR_COMM_DEF_BAUD;
        sensor_settings.period = SOILSENSOR_COMM_DEF_PERIOD;
    }
    
    puts("*****************************");
    
    uint8_t buf_out[BUF_SIZE];
    
    if (!calibration) {
        puts("(!!!) UNCALIBRATED SENSOR (!!!)");
    }
    
    uart_init(SOILSENSOR_COMM_UART, sensor_settings.baud, uart_input, NULL);

    static char stack[SOILSENSOR_STACK_SIZE];
    process_pid = thread_create(stack, SOILSENSOR_STACK_SIZE, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, processing_thread, NULL, "data");
    
    puts("Sending data");
    prepare_data(buf_out);
    send_data(buf_out);
    
    puts("Data sent");
    
    while (1) {
        xtimer_sleep(sensor_settings.period - SOILSENSOR_SENSOR_WARMUP);
        puts("Sending data");
        prepare_data(buf_out);
        send_data(buf_out);
        puts("Data sent");
    };
    
    return 0;
}