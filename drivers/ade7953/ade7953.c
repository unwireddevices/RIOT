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

#include "ade7953.h"
#include "ade7953_params.h"
#include "ade7953_regs.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

static volatile uint8_t state = ADE7953_STATE_STOP;

static uint8_t data_tx[ADE7953_MAX_BYTE_BUFF] = { 0x00 };
static uint8_t data_rx[ADE7953_MAX_BYTE_BUFF] = { 0x00 };

static uint8_t _ade7953_spi_send_recv(const ade7953_t * dev, uint8_t * txbuff, uint8_t * rxbuff, uint8_t length, bool cond);
static uint32_t _ade7953_power_up_setup(const ade7953_t * dev);
static int32_t _ade7953_to_twos_compl(int32_t value);
static uint32_t _ade7953_write(const ade7953_t * dev, uint16_t reg, uint32_t data);
static uint32_t _ade7953_read(const ade7953_t * dev, uint16_t reg);
static uint32_t _ade7953_wait_reset(const ade7953_t * dev);


static int32_t _ade7953_to_twos_compl(int32_t value)
{
    const int modul = 1 << 24;
    const int max_value = (1 << 23) - 1;

    if (value > max_value) {
        value -= modul;
    }
    
    return value;    
}

static uint32_t _ade7953_wait_reset(const ade7953_t * dev)
{
    uint32_t time_begin = lptimer_now_msec();
    uint32_t time_end = 0;
    uint32_t time_delta = 0;
    uint8_t pin_irq = ADE7953_GPIO_PIN_SET;
    
    pin_irq = gpio_read(dev->params.reset);
    while(pin_irq != ADE7953_GPIO_PIN_RESET) {
        pin_irq = gpio_read(dev->params.reset);
        time_end = lptimer_now_msec();
		time_delta = time_end - time_begin;
        if(time_delta > ADE7953_NO_RESPONSE_TIME_MS) {
            return ADE7953_ERROR;
        }
        xtimer_spin(xtimer_ticks_from_usec(ADE7953_NO_RESPONSE_TIME_MIN_USEC));
    }
          
    return ADE7953_OK;
}

/**
 * @brief   This function write data to ADE
 * 
 * @param[in]  dev:         Pointer to ADE7953 device descriptor
 * @param[in]  reg:         Register address
 * @param[in]  data:        Data for writing
 * 
 * @return  0:  data has been successfully wrote
 */
static uint32_t _ade7953_write(const ade7953_t * dev, uint16_t reg, uint32_t data)
{
    uint8_t num_bytes = ((reg >> 8) & 0x3) + 1;
    uint8_t len = 0;
    memset(data_tx, 0x00, ADE7953_MAX_BYTE_BUFF);
    data_tx[len++] = (reg & 0xFF00) >> 8;
    data_tx[len++] = reg & 0xFF;
    data_tx[len++] = ADE7953_CMD_WRITE;    // Write data to ADE

    while(num_bytes--) {
        data_tx[len] = (data >> (8 * num_bytes)) & 0xFF;
        len++;
    }

    _ade7953_spi_send_recv(dev, data_tx, NULL, len, false);     
   
    _ade7953_wait_reset(dev); 	
    
    return ADE7953_OK;
}


/**
 * @brief   This function read data from ADE
 * 
 * @param[in]   dev:            Pointer to ADE7953 device descriptor
 * @param[in]   reg:           Register address
 * 
 * @return  data:  data from ADE
 */
static uint32_t _ade7953_read(const ade7953_t * dev, uint16_t reg)
{
    uint8_t num_bytes = ((reg >> 8) & 0x3) + 1;
    uint32_t data = 0;
    uint8_t len = 0;
    // uint8_t pin_irq = 1;
    memset(data_tx, 0x00, ADE7953_MAX_BYTE_BUFF);
    memset(data_rx, 0x00, ADE7953_MAX_BYTE_BUFF);
    data_tx[len++] = (reg & 0xFF00) >> 8;
    data_tx[len++] = reg & 0xFF;
    data_tx[len++] = ADE7953_CMD_READ;    // Read data from ADE
    len += num_bytes;
    
    _ade7953_spi_send_recv(dev, data_tx, data_rx, len, false);  

    _ade7953_wait_reset(dev); 		

    if(num_bytes == 1) {
        data = data_rx[3];
    } else if(num_bytes == 2) {
        data = (data_rx[3] << 8) | data_rx[4];
    }
    else if(num_bytes == 3) {
        data = (data_rx[3] << 16) | (data_rx[4] << 8) | data_rx[5];
    }
    else if(num_bytes == 4) {
        data = (data_rx[3] << 24) | (data_rx[4] << 16) | (data_rx[5] << 8) | data_rx[6];			
    }		
    return data;
}


/**
 * @brief   This function send/receive data over SPI bus
 * 
 * @param[in]  dev:      Pointer to ADE7953 device descriptor
 * @param[in]  txbuff:   Pointer to the transmit buffer
 * @param[out] rxbuff:   Pointer to the receive buffer 
 * @param[in]  length:   Size of the buffer
 * @param[in]  cond:     Condition of SPI release and trigger CS after sending data
 * 
 * @return  0:  data has been successfully transmitted
 */
static uint8_t _ade7953_spi_send_recv(const ade7953_t * dev, uint8_t * txbuff, uint8_t * rxbuff, uint8_t length, bool cond)
{
    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ADE7953_SPI_CLK);

    /*Send command*/
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, cond, txbuff, rxbuff, length);

    if(cond == false) {
        spi_release(SPI_DEV(dev->params.spi));
    }
       
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

  uint32_t irq_state = 0;
  // Read IRQ states
  _ade7953_read(dev, ADE7953_IRQSTATA_24);
  
  // IRQ handling
    if(irq_state & ADE7953_IRQENA_RESET) {
        state = ADE7953_STATE_READY;
    }
    else if(irq_state & ADE7953_IRQENA_AEHFA) {
        
    }

  // Clear IRQ states
  // _ade7953_read(dev, ADE7953_RSTIRQSTATA_24);    
  
    return;
}

uint32_t ade7953_get_volt(const ade7953_t * dev)
{
    uint32_t volt_tmp = 0;
    volt_tmp = _ade7953_read(dev, ADE7953_V_24);
    return volt_tmp;
}

uint32_t ade7953_get_curr(const ade7953_t * dev)
{
    uint32_t curr_tmp = 0;
    curr_tmp = _ade7953_read(dev, ADE7953_IA_24);
    return curr_tmp;
}

int32_t ade7953_get_awatt(const ade7953_t * dev)
{
    int32_t awatt_tmp = 0;
    awatt_tmp = _ade7953_read(dev, ADE7953_AWATT_24);
    awatt_tmp = _ade7953_to_twos_compl(awatt_tmp);
    awatt_tmp = (awatt_tmp * 1000) / ADE7953_COEFF_ENERGY;
    
    return awatt_tmp;
}

uint32_t ade7953_get_irms(const ade7953_t * dev)
{
    uint32_t irms_tmp = 0;
    irms_tmp = _ade7953_read(dev, ADE7953_IRMSA_24);
    irms_tmp = (irms_tmp * 1000) / ADE7953_COEFF_IRMS;
    return irms_tmp;
}

uint32_t ade7953_get_vrms(const ade7953_t * dev)
{
    uint32_t vrms_tmp = 0;
    vrms_tmp = _ade7953_read(dev, ADE7953_VRMS_24);
    vrms_tmp = (vrms_tmp * 1000) / ADE7953_COEFF_VRMS;
    return vrms_tmp;
}

/**
 * @brief ADE7953 power-up procedure
 *
 * @param[in]   dev:    Pointer to ADE7953 device descriptor
 *
 * @return 0:   if power-up procedure succeeded
 * @return >0:  in case of an error
 */
static uint32_t _ade7953_power_up_setup(const ade7953_t * dev)
{
    uint8_t len = 0;	
    memset(data_tx, 0x00, ADE7953_MAX_BYTE_BUFF);	

    data_tx[len++] = (ADE7953_UNLOCK_REG_8 & 0xFF00) >> 8;
    data_tx[len++] = ADE7953_UNLOCK_REG_8 & 0xFF;
    data_tx[len++] = ADE7953_CMD_WRITE;    // Write data to ADE
    data_tx[len++] = 0x00;
    data_tx[len++] = 0xAD;
    
    _ade7953_spi_send_recv(dev, data_tx, NULL, len, true);  
	
    memset(data_tx, 0x00, ADE7953_MAX_BYTE_BUFF);	
    len = 0;		
    data_tx[len++] = (ADE7953_SETUP_REG_16 & 0xFF00) >> 8;
    data_tx[len++] = ADE7953_SETUP_REG_16 & 0xFF;
    data_tx[len++] = ADE7953_CMD_WRITE;    // Write data to ADE		
    data_tx[len++] = 0x00;
    data_tx[len++] = 0x30;

    _ade7953_spi_send_recv(dev, data_tx, NULL, len, false);  

    _ade7953_wait_reset(dev);
    
    lptimer_sleep(ADE7953_POWER_ON_TIMEOUT_MS);
    
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
    // #define ADE7953_SPI_CLK                    SPI_CLK_1MHZ
    
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
        DEBUG("[ADE7953]: Error init SPI interface\n");
        return ADE7953_ERROR;
    }

    /* Init Interrupt */
    gpio_init_int(dev->params.irq, GPIO_IN_PU, GPIO_FALLING, _ade7953_spi_rx, dev);
    gpio_irq_disable(dev->params.irq); 
    
        /* Software reset */
    _ade7953_write(dev, ADE7953_CONFIG_16, 0x0080);
    
    if(_ade7953_power_up_setup(dev) != ADE7953_OK) {
        return ADE7953_ERROR;
    }
     
    	// uint16_t config = ade7953_read(ADE7953_CONFIG_16);
	_ade7953_write(dev, ADE7953_CONFIG_16, 0x0004);	
    
        /* Set gain amplifier */
    _ade7953_write(dev, ADE7953_PGA_V_8, 0);
    _ade7953_write(dev, ADE7953_PGA_IA_8, 0);
    _ade7953_write(dev, ADE7953_PGA_IB_8, 3);
    
    return ADE7953_OK;
}

#ifdef __cplusplus
}
#endif