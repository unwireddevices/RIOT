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
static uint8_t ade7953_rxbuf[ADE7953_MAX_BYTE_BUFF] = { 0x00 };

// static uint16_t length_rx = 0;

static uint8_t _ade7953_spi_receive(const ade7953_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff);
static uint8_t _ade7953_spi_send(const ade7953_t * dev, uint8_t * txbuff, uint8_t length_tx);


/**
 * @brief   This function send data over SPI bus
 * 
 * @param[in]   dev:            Pointer to ADE7953 device descriptor
 * @param[out]  txbuff:         Pointer to the transmit buffer
 * @param[out]  length_tx:   Size of the transmit buffer
 * 
 * @return  0:  data has been successfully transmitted
 */
uint8_t _ade7953_spi_send(const ade7953_t * dev, uint8_t * txbuff, uint8_t length_tx)
{
    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ADE7953_SPI_CLK);

    /*Send command*/
    // spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &tx_spi, NULL, 1);
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
     
    // spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &rx_spi, NULL, 1);
    // spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, NULL, rxbuff, size_rx_buff);

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
    
    _ade7953_spi_receive(dev, ade7953_rxbuf, 10);
    _ade7953_spi_send(dev, ade7953_txbuf, 10);
    
    return ADE7953_OK;
}

#ifdef __cplusplus
}
#endif