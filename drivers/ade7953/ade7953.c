/* Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file        ade7953.c
 * @brief       ADE7953 driver
 * @authoh      Mikhail Perkov
 */
 
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "xtimer.h"
#include "lptimer.h"
#include "board.h"
#include "periph/spi.h"

#include "ade7953.h"
#include "ade7953_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t ade7953_txbuf[ADE7953_MAX_BYTE_BUFF] = { 0x00 };
// static uint8_t ade7953_rxbuf[ADE7953_MAX_BYTE_BUFF] = { 0x00 };

// static uint16_t length_rx = 0;

static uint8_t _ade7953_spi_receive(const ade7953_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff);
static uint8_t _ade7953_spi_send(const ade7953_t * dev, uint8_t * txbuff, uint8_t length_tx);
static int _ade7953_send(const ade7953_t * dev, uint16_t addr, uint8_t * data, uint8_t len);
int _ade7953_receive(const ade7953_t * dev, uint16_t addr, uint8_t * data, uint8_t len);

static int _ade7953_setup(const ade7953_t * dev);

/**
 * @brief   This function send data over SPI bus
 * 
 * @param[in]   dev:            Pointer to ADE7953 device descriptor
 * @param[out]  txbuff:         Pointer to the transmit buffer
 * @param[out]  length_tx:   Size of the transmit buffer
 * 
 * @return  0:  data has been successfully transmitted
 */
static uint8_t _ade7953_spi_send(const ade7953_t * dev, uint8_t * txbuff, uint8_t length_tx)
{
    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ADE7953_SPI_CLK);

    /*Send command*/
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, txbuff, NULL, length_tx);

    spi_release(SPI_DEV(dev->params.spi));
          
    memset(txbuff, 0x00, length_tx);
        
    gpio_irq_enable(dev->params.irq);
       
    return ADE7953_OK;
}

/**
 * @brief   This function receive data over SPI bus
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[out]  size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  data has been successfully received
 * @return  >0: in case of an error
 */
static uint8_t _ade7953_spi_receive(const ade7953_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff)
{      
    if(size_rx_buff > ADE7953_MAX_BYTE_BUFF) {
        size_rx_buff = ADE7953_MAX_BYTE_BUFF;
    }
    
    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ADE7953_SPI_CLK);

    memset(rxbuff, 0x00, size_rx_buff);       
     
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, NULL, rxbuff, size_rx_buff);

    spi_release(SPI_DEV(dev->params.spi));

    return ADE7953_OK;
}

/**
 * @brief   Callback
 * 
 * @return  None
 */
static void _ade7953_spi_rx(void* arg)
{
    (void) arg;
    ade7953_t *dev = arg;
    
    if (dev->cb) {
        dev->cb(dev->arg);
    }
  
    return;
}

/**
 * @brief   This function send data in ADE
 * 
 * @param[in]   dev:            Pointer to ADE7953 device descriptor
 * @param[in]   addr:           Register address
 * @param[out]  txbuff:         Pointer to the transmit buffer
 * @param[out]  length_tx:      Size of the transmit buffer
 * 
 * @return  0:  data has been successfully transmitted
 * @return >0:  in case of an error
 */
static int _ade7953_send(const ade7953_t * dev, uint16_t addr, uint8_t * data, uint8_t len )
{
    uint8_t byte_len = ((addr & 0x300) >> 8)  + 1;

    if( len != byte_len ){
        return ADE7953_ERROR;
    }
    uint8_t length_tx = 0;
    // Send packet
    ade7953_txbuf[length_tx++] = (addr & 0xFF00) >> 8;
    ade7953_txbuf[length_tx++] = addr & 0xFF;
    ade7953_txbuf[length_tx++] = 0x00;    // Write data to ADE
    // for( int8_t i = byte_len; i > 0; ){
    for(uint32_t i = 0; i < byte_len; i++){
        ade7953_txbuf[length_tx++] = *data;
        data--;
    }

    _ade7953_spi_send(dev, ade7953_txbuf, length_tx);
    return ADE7953_OK;
}

/**
 * @brief   This function receive data from ADE
 * 
 * @param[in]   dev:            Pointer to ADE7953 device descriptor
 * @param[in]   addr:           Register address
 * @param[out]  txbuff:         Pointer to the receive buffer
 * @param[out]  length_tx:      Size of the receive buffer
 * 
 * @return  0:  data has been successfully received
 * @return >0:  in case of an error
 */
int _ade7953_receive(const ade7953_t * dev, uint16_t addr, uint8_t * data, uint8_t len)
{
    uint8_t byte_len = ((addr & 0x300) >> 8)  + 1;

    if( len != byte_len ){
        return ADE7953_ERROR;
    }
    uint8_t length_tx = 0;
    // Send packet
    ade7953_txbuf[length_tx++] = (addr & 0xFF00) >> 8;
    ade7953_txbuf[length_tx++] = addr & 0xFF;
    ade7953_txbuf[length_tx++] = 0x80;    // Read data from ADE
    length_tx += len;

    _ade7953_spi_send(dev, ade7953_txbuf, length_tx);
    _ade7953_spi_receive(dev, data, length_tx);
    
    return ADE7953_OK;
}

/**
 * @brief ADE7953 power up setup
 *
 * @param[in]   dev:    Pointer to ADE7953 device descriptor
 * @param[in]   params: Pointer to static ST95 device configuration
 *
 * @return 0:   if initialization succeeded
 * @return >0:  in case of an error
 */
static int _ade7953_setup(const ade7953_t * dev)
{
    uint8_t  data[8] = { 0x00 };
    uint8_t len = 0;
    data[0] = 0xAD;
    len = 1;
    if(_ade7953_send(dev, ADE7953_UNLOCK_REG_8, data, len) != ADE7953_OK) {
        return ADE7953_ERROR;
    }

    data[0] = 0x30;
    data[1] = 0x00;
    len = 2;
    if(_ade7953_send(dev, ADE7953_SETUP_REG_16, data, len) != ADE7953_OK) {
        return ADE7953_ERROR;
    }
    
    // Set gain
  data[0] = 1;         // Gain coeff (1, 2, 4, 8, 16, 22)
  len = 1;
  _ade7953_send(dev, ADE7953_PGA_V_8, data, len);
  _ade7953_send(dev, ADE7953_PGA_IA_8, data, len);
  
    // Set I_max and V_max
    

    // Set IRQ:
    // OIA - I max
    // OV  - V max
    // AEHFA - enable an interrupt when the active energy is half full (Current Channel  A)
  // data.u32d = IRQENA_OIA | IRQENA_OV | IRQENA_AEHFA;
  // _ade7953_send( IRQENA_32, (uint8_t *)&(data.u32d), 4);    
    return ADE7953_OK;
}

/**
 * @brief ADE7953 driver initialization routine
 *
 * @param[in]   dev:    Pointer to ADE7953 device descriptor
 * @param[in]   params: Pointer to static ST95 device configuration
 *
 * @return 0:   if initialization succeeded
 * @return >0:  in case of an error
 */
int ade7953_init(ade7953_t * dev, ade7953_params_t * params)
{      
    // ade7953_t ade7953_dev;
    
    // #define ADE7953_SPI_DEV     2
    // #define ADE7953_SPI_CS      UNWD_GPIO_4     // PD13
    // #define ADE7953_IRQ         UNWD_GPIO_25    // PE7
    // #define ADE7953_RESET       UNWD_GPIO_26    // PD11
    
    // ade7953_params_t ade7953_params = { .spi = ADE7953_SPI_DEV, . cs_spi = ADE7953_SPI_CS, .irq =  ADE7953_IRQ, .reset = ADE7953_RESET };
    // ade7953_init(&ade7953_dev, &ade7953_params);

    dev->params = *params;
    dev->arg = NULL;
    dev->cb = NULL;
    
    /* Init RESET pin */
    gpio_init(dev->params.reset, GPIO_OD_PU);
    
    /* Initialize SPI */
    spi_init(SPI_DEV(dev->params.spi));
    /* Initialize CS SPI */
    if(spi_init_cs(SPI_DEV(dev->params.spi), dev->params.cs_spi) != SPI_OK) {
        DEBUG("[ST95]: Error init SPI interface\n");
        return ADE7953_ERROR;
    }

    /* Init Interrupt */
    gpio_init_int(dev->params.irq, GPIO_IN_PU, GPIO_FALLING, _ade7953_spi_rx, dev);
    gpio_irq_disable(dev->params.irq);
    
    if(_ade7953_setup(dev) != ADE7953_OK) {
        return ADE7953_ERROR;
    }
    
    return ADE7953_OK;
}

#ifdef __cplusplus
}
#endif