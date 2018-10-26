/* Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file        st95.c
 * @brief       ST95 driver
 * @authoh      Mikhail Perkov
 */
 
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "xtimer.h"
#include "rtctimers-millis.h"
#include "board.h"

#include "st95.h"
#include "st95_params.h"
#include "iso14443a.h"


#ifdef __cplusplus
extern "C" {
#endif

static st95_params_t st95_device;

static uint8_t st95_txbuf[255] = { 0x00 };
static uint8_t st95_rxbuf[255] = { 0x00 };

static uint8_t dac_data_h = 0;
static uint8_t dac_data_l = 0;

static volatile bool data_rx = 0;
static volatile bool timeout = 0;

static volatile uint8_t state = 0;
static volatile uint8_t is_wake_up = 0;
static int wake_up(void);
static void _wait_ready_data(void);

 // static void _printbuff(uint8_t *buff, unsigned len)
    // {
        // while (len) {
            // len--;
            // printf("%02X ", *buff++);
        // }
        // printf("\n");
    // }
    
static void st95_send_irqin_negative_pulse(void)
{
    gpio_set(st95_device.irq_in);
    xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    gpio_clear(st95_device.irq_in);
    xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    gpio_set(st95_device.irq_in);
}

// static void st95_reset_spi(void)
// {
    // spi_acquire(SPI_DEV(st95_params.spi), st95_params.cs_spi, SPI_MODE_0, SPI_CLK_1MHZ);
        /*Send Reset*/
    // tx_spi = 0x01;
    // spi_transfer_bytes(SPI_DEV(st95_params.spi), st95_params.cs_spi, false, &tx_spi, &rx_reset, 1);

    // spi_release(SPI_DEV(st95_params.spi));

    // rtctimers_millis_sleep(20);
    // st95_send_irqin_negative_pulse();
// }

// static uint8_t st95_select_field_off(void)
// {
    // txbuf[0] = ST95_CMD_PROTOCOL;
    // txbuf[1] = 2;
    // txbuf[2] = FIELD_OFF;
    // txbuf[3] = 0x00;

    // send_pack(4);

    // return 1;
// }

// static uint8_t st95_select_iso18092(void)
// {
    // uint8_t length  = 0;

    // txbuf[0] = ST95_CMD_PROTOCOL;
    // txbuf[1] = 2;    // Data Length

    // txbuf[2] = ISO_18092;

    // txbuf[3] = 0x00 | (ST95_TX_RATE_14443A << 6) | (ST95_RX_RATE_14443A << 4);

    // txbuf[3] = 0x51;
    // length = 4;

    // txbuf[4] = 0x00;    // PP (Optioanal)                                                             // 00
    // txbuf[5] = 0x00;    // MM (Optioanal)                                                            // 01
    // txbuf[6] = 0x00;    // DD (Optioanal)                                                            // 80
    // txbuf[7] = 0x00;    // ST Reserved (Optioanal)
    // txbuf[8] = 0x00;    // ST Reserved (Optioanal)

    // send_pack(length);

    // return 4;

    // return 0;
// }

// static uint8_t st95_select_iso15693(void)
// {
    // txbuf[0] = 0x00;

    // txbuf[1] = ST95_CMD_PROTOCOL;
    // txbuf[2] = 2;// Length

    // txbuf[3] = ISO_15693;
    // txbuf[4] = 0x05;

    // return 5;
// }


// static uint8_t st95_select_iso14443b(void)
// {
    // txbuf[0] = 0x00;

    // txbuf[1] = ST95_CMD_PROTOCOL;
    // txbuf[2] = 2;// Length

    // txbuf[3] = ISO_14443B;
   // txbuf[4] = 0x00 | (ST95_TX_RATE_106 << 6) | (ST95_RX_RATE_106 << 4);    // (TX_RATE << 6) | (RX_RATE << 4) // 02
    // txbuf[4] = 0x01;


    // txbuf[5] = 0x00;    // PP (Optioanal)                                                             // 00
    // txbuf[6] = 0x00;    // MM (Optioanal)                                                            // 01
    // txbuf[7] = 0x00;    // DD (Optioanal)                                                            // 80
    // txbuf[8] = 0x00;    // ST Reserved (Optioanal)
    // txbuf[9] = 0x00;    // ST Reserved (Optioanal)

    // return 5;
// }

uint8_t st95_send(uint8_t length_tx)
{
    uint8_t tx_spi = 0x00;

    spi_acquire(SPI_DEV(st95_device.spi), st95_device.cs_spi, SPI_MODE_0, ST95_SPI_CLK);

    /*Send command*/
    spi_transfer_bytes(SPI_DEV(st95_device.spi), st95_device.cs_spi, true, &tx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(st95_device.spi), st95_device.cs_spi, false, st95_txbuf, NULL, length_tx);

         // reset the CR95HF data status 
    data_rx = false;
    
    gpio_irq_enable(st95_device.irq_out);
    
    return ST95_OK;
}

uint8_t st95_receive(uint8_t * rxbuff)
{ 
    uint8_t rx_spi = 0x02;
    uint16_t length_rx = 0;
    
    // _wait_ready_data();     
    
    gpio_irq_disable(st95_device.irq_out);
    
    if(timeout == true) {		
			// Disable CR95HF EXTI 
        // gpio_irq_disable(st95_params->irq_out);
        return ST95_NO_DEVICE;
	}
      
    spi_transfer_bytes(SPI_DEV(st95_device.spi), st95_device.cs_spi, true, &rx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(st95_device.spi), st95_device.cs_spi, false, NULL, rxbuff, sizeof(st95_rxbuf));

    spi_release(SPI_DEV(st95_device.spi));

    if(rxbuff[0] == ST95_CMD_ECHO) {
        length_rx = 1;
    }
    else {
        length_rx = rxbuff[1] + 2;
        
        if((length_rx - 2) != rxbuff[1]) {
            return ST95_ERROR;
        }
    }     
    return ST95_OK;
}

static void _wait_ready_data(void)
{
    uint32_t time_begin = rtctimers_millis_now();
    uint32_t time_end = 0;
    uint32_t time_delta = 0;
    timeout = false;
    
	while((data_rx == false) && (timeout == false)) {
		time_end = rtctimers_millis_now();
		time_delta = time_end - time_begin;
		if(time_delta > ST95_NO_RESPONSE_TIME_MS) {
            timeout = true;
		}
	}
    return;
}

static void st95_spi_rx(void* arg)
{
    (void) arg;
    
    data_rx = true;
    if(state == 1) {
        wake_up();
    }
    
    return;
}

int st95_echo(void)
{   
    st95_txbuf[0] = ST95_CMD_ECHO;
    
    st95_send(1);
        _wait_ready_data();  
	if(st95_receive(st95_rxbuf) == ST95_OK) {
        if(st95_rxbuf[0] == ST95_CMD_ECHO) {
            return ST95_OK;
        }
        else {
            return ST95_NO_DEVICE;
        }
    }
    return ST95_ERROR;
}

int st95_idn(uint8_t * idn, uint8_t * length)
{ 
    
    st95_txbuf[0] = ST95_CMD_IDN;
    st95_txbuf[1] = 0x00;
    
    st95_send(2);
        _wait_ready_data();  
    if(st95_receive(st95_rxbuf) == ST95_OK) {
        memcpy(idn, (st95_rxbuf + 2), st95_rxbuf[1]);
        *length = (st95_rxbuf[1] + 2);       
        return ST95_OK;      
    }   	
    return ST95_ERROR;
}

uint8_t st95_calibration(void)
{   
    st95_txbuf[0] = ST95_CMD_IDLE;    // Command
    st95_txbuf[1] = 14;                // Data Length
    /* Idle params */
        /* Wake Up Source */
    st95_txbuf[2] = 0x03;            // Tag Detection + Time out
        /* Enter Control (the resources when entering WFE mode)*/
    st95_txbuf[3] = 0xA1;            // Tag Detector Calibration
    st95_txbuf[4] = 0x00;            //
        /* Wake Up Control (the wake-up resources) */
    st95_txbuf[5] = 0xF8;            // Tag Detector Calibration
    st95_txbuf[6] = 0x01;            //
        /* Leave Control (the resources when returning to Ready state)*/
    st95_txbuf[7] = 0x18;            // Tag Detection
    st95_txbuf[8] = 0x00;
        /* Wake Up Period (the time allowed between two tag detections) */
    st95_txbuf[9] = 0x02;            //
        /* Osc Start (the delay for HFO stabilization) */
    st95_txbuf[10] = 0x60;            // Recommendeded value 0x60
        /* DAC Start (the delay for DAC stabilization) */
    st95_txbuf[11] = 0x60;            // Recommendeded value 0x60

        /* DAC Data */
    st95_txbuf[12] = 0x00;            // DacDataL
    st95_txbuf[13] = 0x00;            // DacDataH

        /* Swing Count */
    st95_txbuf[14] = 0x3F;            // Recommendeded value 0x3F
        /* Max Sleep */
    st95_txbuf[15] = 0x01;            // This value must be set to 0x01 during tag detection calibration

    st95_send(16);
        _wait_ready_data();  
    if(st95_receive(st95_rxbuf) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            st95_txbuf[13] = 0xFC;
            st95_send(16);
        }
        else {
            return ST95_ERROR;
        }
    }
    
    while(st95_txbuf[13] > 0x02) {
            _wait_ready_data();  
        if(st95_receive(st95_rxbuf) == ST95_OK) {
            if(st95_rxbuf[2] == 0x02) {
                dac_data_l = st95_txbuf[13];
                dac_data_h = 0xFC;
                // puts(" Calibration done");
                return ST95_OK;
            }
            else if(st95_rxbuf[2] == 0x01){
                st95_txbuf[13] -= 0x04;
                st95_send(16);
            }
            else {
                return ST95_ERROR;
            }
        }
        else {
             return ST95_ERROR;
        }        
    }   	
    
    return  ST95_ERROR;
}

uint8_t st95_idle(void)
{    
    st95_txbuf[0] = ST95_CMD_IDLE;    // Command
    st95_txbuf[1] = 14;                // Data Length
    /* Idle params */
        /* Wake Up Source */
    st95_txbuf[2] = 0x02;            // Tag Detection
        /* Enter Control (the resources when entering WFE mode)*/
    st95_txbuf[3] = 0x21;            // Tag Detection
    st95_txbuf[4] = 0x00;            //
        /* Wake Up Control (the wake-up resources) */
    st95_txbuf[5] = 0x79;            // Tag Detection
    st95_txbuf[6] = 0x01;            //
        /* Leave Control (the resources when returning to Ready state)*/
    st95_txbuf[7] = 0x18;            // Tag Detection
    st95_txbuf[8] = 0x00;            //
        /* Wake Up Period (the time allowed between two tag detections) */
    st95_txbuf[9] = 0x20;            // Typical value 0x20
        /* Osc Start (the delay for HFO stabilization) */
    st95_txbuf[10] = 0x60;            // Recommendeded value 0x60
        /* DAC Start (the delay for DAC stabilization) */
    st95_txbuf[11] = 0x60;            // Recommendeded value 0x60
        /* DAC Data */
    st95_txbuf[12] = dac_data_l;     //0x42;            // DacDataL
    st95_txbuf[13] = dac_data_h;     //0xFC;            // DacDataH
        /* Swing Count */
    st95_txbuf[14] = 0x3F;            // Recommendeded value 0x3F
        /* Max Sleep */
    st95_txbuf[15] = 0x08;            // Typical value 0x28
    
    st95_send(16);
    
    // while(data_rx == false){
        // rtctimers_millis_sleep(100);
    // }
    state = 1;
    
   
    return ST95_OK;
}

static int wake_up(void)
{
    state = 0;
    is_wake_up = 0;
    if(st95_receive(st95_rxbuf) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            is_wake_up = 1;
            return ST95_OK;
        }
    }
    return ST95_ERROR;
}

int st95_select_iso14443a(void)
{
    st95_txbuf[0] = ST95_CMD_PROTOCOL;
    st95_txbuf[1] = 2;    // Data Length
    st95_txbuf[2] = ISO_14443A;
    st95_txbuf[3] = 0x00 | (ST95_TX_RATE_14443A << 6) | (ST95_RX_RATE_14443A << 4);

    st95_txbuf[4] = 0x00;    // PP (Optioanal)                                                             // 00
    st95_txbuf[5] = 0x00;    // MM (Optioanal)                                                            // 01
    st95_txbuf[6] = 0x00;    // DD (Optioanal)                                                            // 80
    st95_txbuf[7] = 0x00;    // ST Reserved (Optioanal)
    st95_txbuf[8] = 0x00;    // ST Reserved (Optioanal)
    
    st95_send(4);
        _wait_ready_data();  
    if(st95_receive(st95_rxbuf) == ST95_OK) {       
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00))
            return ST95_OK;
    }

    return ST95_ERROR;
}

void st95_send_receive(uint8_t *data, uint8_t size, uint8_t topaz, uint8_t split_frame, uint8_t crc, uint8_t sign_bits) 
{
	uint8_t length = 0;

    st95_txbuf[0] = ST95_CMD_SEND_RECV;
    st95_txbuf[1] = size + 1;
	
	memcpy(st95_txbuf + 2, data, size);
	length = size + 2;
	
	st95_txbuf[length] = (topaz << 7) | (split_frame << 6) | (crc << 5) | sign_bits;
	length++;
    	
	st95_send(length);
}

int st95_get_uid(uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{
    if(st95_idle() == ST95_OK){ 
        if(is_wake_up == 0) {
            return ST95_ERROR;
        }       
        else if(is_wake_up == 1) {
            if(st95_select_iso14443a() == ST95_ERROR)
                return ST95_ERROR;
        
            if(get_uid_14443a(length_uid, uid, sak) == ST95_OK)
                return ST95_OK;
        }
    }
    
    return ST95_ERROR;
}

/**
 * @brief ST95 driver initialization routine
 *
 * @param[in] SPI interface to be used for 1-Wire
 * @param[in] GPIO to be used for 1-Wire
 *
 * @return 1 if initialization succeeded
 * @return <0 in case of an error
 */
int st95_init(st95_params_t *device)
{   
    st95_device.spi = device->spi;
    st95_device.cs_spi = device->cs_spi;
    st95_device.irq_in = device->irq_in;
    st95_device.irq_out = device->irq_out;
    st95_device.ssi_0 = device->ssi_0;
    st95_device.ssi_1 = device->ssi_1;
    
    gpio_init(st95_device.ssi_0, GPIO_OUT);
    gpio_init(st95_device.irq_in, GPIO_OUT);
    gpio_clear(st95_device.irq_in);
    
    // rtctimers_millis_sleep(ST95_RAMP_UP_TIME_MS);
    /* Select SPI iface */
    gpio_set(st95_device.ssi_0);
    
    // TODO: Set "power on" ST95HF
    st95_send_irqin_negative_pulse();
    
        /* Initialize SPI */
    spi_init(SPI_DEV(st95_device.spi));
        /* Initialize CS SPI */
    if(spi_init_cs(SPI_DEV(st95_device.spi), st95_device.cs_spi) != SPI_OK) {
        // puts("Error init SPI interface");
        return ST95_ERROR;
    }

    gpio_init_int(st95_device.irq_out, GPIO_IN_PU, GPIO_FALLING, st95_spi_rx, NULL);
    // gpio_irq_disable(st95_device.irq_out);
    
    rtctimers_millis_sleep(ST95_HFO_SETUP_TIME_MS);
    
    if(st95_echo() != ST95_OK){
        return ST95_ERROR;
    }

    if(st95_calibration() != ST95_OK) {
        return ST95_ERROR;
    }
    
    return ST95_OK;
}

#ifdef __cplusplus
}
#endif