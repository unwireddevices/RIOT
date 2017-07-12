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

#include "onewire_config.h"
#include "onewire.h"

#ifdef __cplusplus
extern "C" {
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
static onewire_t ow_device;

/**
 * @brief Delay before checking received byte
 */
static void ow_delay(void)
{
    for(volatile uint8_t i = 0; i < OW_DELAY; i++);
}

/**
 * @brief UART interrupt handler
 */
static void ow_rx_handler(void *arg, uint8_t data)
{
  (void) arg;

    rx_byte = data;
}

/**
 * @brief 1-Wire bus driver initialization routine
 *
 * @param[in] USART interface to be used for 1-Wire
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int ow_init(onewire_t device)
{
    ow_device = UART_DEV(device);
    
      /* Initialize the UART */
    uart_init(ow_device, ONEWIRE_BAUD_9600, ow_rx_handler, NULL);
    /* Set pin of TX like as open-drain */
    gpio_init(uart_config[ow_device].tx_pin, GPIO_OD);
    gpio_init_af(uart_config[ow_device].tx_pin, uart_config[ow_device].tx_af);

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
uint8_t ow_PRESENCE(void)
{    
    uart_set_baudrate(ow_device, ONEWIRE_BAUD_9600);
    
    tx_byte = OW_RESET;
  /* Send data */
    uart_write(ow_device, &tx_byte, 1);
  ow_delay();

     if (rx_byte != OW_RESET) return OW_OK;        // Device detected
 return OW_NO_DEVICE;                            // Device not detected

}

/**
 * @brief 1-Wire command to transmit a single bit
 *
 * @param[in] Bit value (0|1)
 *
 */
void ow_SendBit(uint8_t bit)
{
    uart_set_baudrate(ow_device, ONEWIRE_BAUD_115200);
    
    uint8_t data = OW_WRITE_0;
    if(bit) data = OW_WRITE_1;

    tx_byte = data;
  /* Send data */
  uart_write(ow_device, &tx_byte, 1);
}

/**
 * @brief 1-Wire command to read a single bit
 *
 * @param[out] Bit value (0|1)
 *
 */
uint8_t ow_ReadBit(void)
{
    uart_set_baudrate(ow_device, ONEWIRE_BAUD_115200);
    tx_byte = OW_READ;
  /* Send data */
  uart_write(ow_device, &tx_byte, 1);
    ow_delay();

     if(rx_byte == OW_READ) return 1;
    return 0;
}

/**
 * @brief 1-Wire to read a byte
 *
 * @param[out] Byte read
 *
 */
uint8_t ow_ReadByte(void)
{
    uint8_t byte = 0;
    
    for(uint8_t i = 0; i < 8; i++) {
        if(ow_ReadBit()) _SETBIT(byte, i); 
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
uint8_t ow_SendByte(uint8_t byte)
{
        /* Send byte */
    for(uint8_t i = 0; i < 8; i++) {
        /* Send bit */
        ow_SendBit(byte & 0x01);
        /* Next bit in byte */
        byte >>= 1;
        ow_delay();
        if(rx_byte > 0xFE) byte|= 0x80;
    }
        
    return byte & 0xFF;
}


#ifdef __cplusplus
}
#endif
