/* Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file        onewire.c
 * @brief       1-Wire bus driver
 * @authoh      Mikhail Perkov
 */

#include "periph/gpio.h"
#include "periph/uart.h"
#include "xtimer.h"

#include "onewire_config.h"
#include "onewire.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ONEWIRE_BITBANG_MODE
static xtimer_ticks32_t onewire_delay_a;
static xtimer_ticks32_t onewire_delay_b;
static xtimer_ticks32_t onewire_delay_c;
static xtimer_ticks32_t onewire_delay_d;
static xtimer_ticks32_t onewire_delay_e;
static xtimer_ticks32_t onewire_delay_f;
static xtimer_ticks32_t onewire_delay_g;
static xtimer_ticks32_t onewire_delay_h;
static xtimer_ticks32_t onewire_delay_i;
static xtimer_ticks32_t onewire_delay_j;

static gpio_t onewire_pin;
#endif

/**
 * @brief   Byte received by 1-Wire
 */
static volatile uint8_t rx_byte = 0 ;

/**
 * @brief   Byte to be transmitted
 */
static uint8_t tx_byte = 0;

/**
 * @brief   Using UART as 1-Wire bus device
 */
static onewire_t onewire_device;

/**
 * @brief Delay before checking received byte
 */
static void onewire_delay(void)
{
    for(volatile uint8_t i = 0; i < OW_DELAY; i++);
}

/**
 * @brief 1-Wire command to transmit a single bit
 *
 * @param[in] Bit value (0|1)
 *
 */
static void onewire_sendbit(uint8_t bit)
{
#ifdef ONEWIRE_BITBANG_MODE
    gpio_clear(onewire_pin);
    if (bit) {
        xtimer_spin(onewire_delay_a);
        gpio_set(onewire_pin);
        xtimer_spin(onewire_delay_b);
    } else {
        xtimer_spin(onewire_delay_c);
        gpio_set(onewire_pin);
        xtimer_spin(onewire_delay_d);
    }
#else
    uart_set_baudrate(onewire_device, ONEWIRE_BAUD_115200);
    
    uint8_t data = OW_WRITE_0;
    if(bit) data = OW_WRITE_1;

    tx_byte = data;
    /* Send data */
    uart_write(onewire_device, &tx_byte, 1);
#endif
}

/**
 * @brief 1-Wire command to read a single bit
 *
 * @param[out] Bit value (0|1)
 *
 */
static uint8_t onewire_readbit(void)
{
#ifdef ONEWIRE_BITBANG_MODE
    uint8_t result = 0;
    
    gpio_clear(onewire_pin);
    xtimer_spin(onewire_delay_a);
    gpio_set(onewire_pin);
    
    xtimer_spin(onewire_delay_e);
    gpio_init(onewire_pin, GPIO_IN);
    
    if (gpio_read(onewire_pin)) {
        result = 1;
    }
    
    gpio_init(onewire_pin, GPIO_OD);
    xtimer_spin(onewire_delay_f);
    
    return result;
#else
    uart_set_baudrate(onewire_device, ONEWIRE_BAUD_115200);
    tx_byte = OW_READ;
    
    /* Send data */
    uart_write(onewire_device, &tx_byte, 1);
    onewire_delay();

    if(rx_byte == OW_READ) return 1;
    return 0;
#endif
}

/**
 * @brief UART interrupt handler
 */
#ifndef ONEWIRE_BITBANG_MODE
static void onewire_rx_handler(void *arg, uint8_t data)
{
  (void) arg;

    rx_byte = data;
}
#endif

/**
 * @brief 1-Wire bus driver initialization routine
 *
 * @param[in] USART interface to be used for 1-Wire
 * @param[in] GPIO to be used for 1-Wire
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int onewire_init(onewire_t device)
{
#ifdef ONEWIRE_BITBANG_MODE
    onewire_delay_a = xtimer_ticks_from_usec(6);
    onewire_delay_b = xtimer_ticks_from_usec(64);
    onewire_delay_c = xtimer_ticks_from_usec(60);
    onewire_delay_d = xtimer_ticks_from_usec(10);
    onewire_delay_e = xtimer_ticks_from_usec(9);
    onewire_delay_f = xtimer_ticks_from_usec(55);
    onewire_delay_g = xtimer_ticks_from_usec(0);
    onewire_delay_h = xtimer_ticks_from_usec(480);
    onewire_delay_i = xtimer_ticks_from_usec(70);
    onewire_delay_j = xtimer_ticks_from_usec(410);
    
    onewire_pin = onewire_pin;
    gpio_init(onewire_pin, GPIO_OD);
#else
    onewire_device = UART_DEV(device);
    
    /* Initialize the UART */
    uart_init(onewire_device, ONEWIRE_BAUD_9600, onewire_rx_handler, NULL);
    /* Set pin of TX like as open-drain */
    gpio_init(uart_config[onewire_device].tx_pin, GPIO_OD_PU);
    gpio_init_af(uart_config[onewire_device].tx_pin, uart_config[onewire_device].tx_af);
#endif
    
  return OW_OK;
}

/**
 * @brief 1-Wire reset and device present detection
 *
 * @param[in] None
 *
 * @return 1 if device is present
 * @return 0 if device is not present
 */
uint8_t onewire_detect(void)
{    
#ifdef ONEWIRE_BITBANG_MODE
    uint8_t result = OW_OK;
    
    xtimer_spin(onewire_delay_g);
    gpio_clear(onewire_pin);
    xtimer_spin(onewire_delay_h);
    gpio_set(onewire_pin);
    
    xtimer_spin(onewire_delay_i);
    gpio_init(onewire_pin, GPIO_IN);
    
    if (gpio_read(onewire_pin)) {
        result = OW_NO_DEVICE;
    }
    
    gpio_init(onewire_pin, GPIO_OD);
    xtimer_spin(onewire_delay_j);
    
    return result;
#else
    uart_set_baudrate(onewire_device, ONEWIRE_BAUD_9600);
    
    tx_byte = OW_RESET;
    /* Send data */
    uart_write(onewire_device, &tx_byte, 1);
    onewire_delay();

    if (rx_byte != OW_RESET) return OW_OK;        // Device detected
    return OW_NO_DEVICE;                            // Device not detected
#endif
}

/**
 * @brief 1-Wire to read a byte
 *
 * @param[out] Byte read
 *
 */
uint8_t onewire_readbyte(void)
{
    uint8_t byte = 0;
    
    for(uint8_t i = 0; i < 8; i++) {
        if(onewire_readbit()) _SETBIT(byte, i); 
    }

    return byte;
}

/**
 * @brief 1-Wire command to transmit a byte
 *
 * @param[in] Byte to transimt
 * @param[out] Transmitted byte
 *
 */
uint8_t onewire_sendbyte(uint8_t byte)
{
    /* Send byte */
    for(uint8_t i = 0; i < 8; i++) {
        /* Send bit */
        onewire_sendbit(byte & 0x01);
        /* Next bit in byte */
        byte >>= 1;
        onewire_delay();
        if(rx_byte > 0xFE) byte|= 0x80;
    }
        
    return byte & 0xFF;
}


#ifdef __cplusplus
}
#endif
