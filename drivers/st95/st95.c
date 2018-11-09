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

#define ENABLE_DEBUG (0)
#include "debug.h"

#define ST95_ENABLE_GAIN (1)

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t st95_txbuf[ST95_MAX_BYTE_BUFF] = { 0x00 };
static uint8_t st95_rxbuf[ST95_MAX_BYTE_BUFF] = { 0x00 };

static uint8_t dac_data_h = 0;
static uint8_t dac_data_l = 0;

static volatile bool st95_data_rx = 0;
static volatile bool st95_timeout = 0;

static volatile uint8_t state = ST95_READY_STATE;

static void _st95_wait_ready_data(void);

static int _st95_cmd_echo(const st95_t * dev);
static uint8_t _st95_calibration(const st95_t * dev);
static uint8_t _st95_cmd_idle(const st95_t * dev);

#if ENABLE_GAIN
static uint8_t _st95_cmd_write_reg(const st95_t * dev, uint8_t size_tx, uint8_t addr, uint8_t flag, uint8_t * data_tx);
static uint8_t _st95_read_reg(const st95_t * dev, uint8_t * rxbuff);
static uint8_t _st95_modify_modulation_gain(const st95_t * dev, uint8_t modul, uint8_t gain);
static uint8_t _st95_read_modulation_gain(const st95_t * dev, uint8_t * modul, uint8_t * gain);
#endif

 // static void _printbuff(uint8_t *buff, unsigned len)
    // {
        // while (len) {
            // len--;
            // printf("%02X ", *buff++);
        // }
        // printf("\n");
    // }
    
    
// static void st95_send_irqin_negative_pulse(void)
// {
    // gpio_set(st95_device.irq_in);
    // xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    // gpio_clear(st95_device.irq_in);
    // xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    // gpio_set(st95_device.irq_in);
// }



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

// static uint8_t _st95_select_iso14443b(const st95_t * dev)
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
    
    // _st95_spi_send(dev, 4);
    // _st95_wait_ready_data();  

    // if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {       
        // if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00))
            // return ST95_OK;
    // }

    // return ST95_ERROR;
// }

uint8_t _st95_spi_send(const st95_t * dev, uint8_t length_tx)
{
    uint8_t tx_spi = ST95_CTRT_SPI_SEND;

    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ST95_SPI_CLK);

    /*Send command*/
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &tx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, st95_txbuf, NULL, length_tx);

    // reset the ST95HF data status 
    st95_data_rx = false;
    st95_timeout = false;
        
    gpio_irq_enable(dev->params.irq_out);
    
    return ST95_OK;
}

uint8_t _st95_spi_receive(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff)
{ 
    uint8_t rx_spi = ST95_CTRT_SPI_READ;
    uint16_t length_rx = 0;
             
    gpio_irq_disable(dev->params.irq_out);
    
    if(st95_timeout == true) {
        return ST95_NO_DEVICE;
	}
      
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &rx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, NULL, rxbuff, size_rx_buff);

    spi_release(SPI_DEV(dev->params.spi));

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

static void _st95_wait_ready_data(void)
{
    uint32_t time_begin = rtctimers_millis_now();
    uint32_t time_end = 0;
    uint32_t time_delta = 0;
    st95_timeout = false;
        
    xtimer_spin(xtimer_ticks_from_usec(ST95_NO_RESPONSE_TIME_MIN_MS));
    
	while((st95_data_rx == false) && (st95_timeout == false)) {
		time_end = rtctimers_millis_now();
		time_delta = time_end - time_begin;
		if(time_delta > ST95_NO_RESPONSE_TIME_MS) {
            st95_timeout = true;
		}
        xtimer_spin(xtimer_ticks_from_usec(ST95_NO_RESPONSE_TIME_MIN_MS));
	}
    return;
}

static void _st95_spi_rx(void* arg)
{
    st95_t *dev = arg;
    
    st95_data_rx = true;    

    if(state == ST95_IDLE_STATE) {
        if (dev->cb) {
            dev->cb(NULL);
        }
    }
         
    return;
}

static int _st95_cmd_echo(const st95_t * dev)
{   
    st95_txbuf[0] = ST95_CMD_ECHO;
    
    _st95_spi_send(dev, 1);

    _st95_wait_ready_data();

    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        if(st95_rxbuf[0] == ST95_CMD_ECHO) {
            return ST95_OK;
        }
        else {
            return ST95_NO_DEVICE;
        }
    }

    return ST95_ERROR;
}

#if ST95_ENABLE_GAIN
static uint8_t _st95_read_reg(const st95_t * dev, uint8_t * rxbuff)
{    
    st95_txbuf[0] = ST95_CMD_READ_REG;
    st95_txbuf[1] = 3;
    st95_txbuf[2] = ST95_READ_ADDR_1;
    st95_txbuf[3] = ST95_REG_SIZE;
    st95_txbuf[4] = ST95_ST_RESERVED;
    
    _st95_spi_send(dev, 5);
    _st95_wait_ready_data();

    if(_st95_spi_receive(dev, rxbuff, ST95_MAX_BYTE_BUFF) == ST95_OK) {     
        return ST95_OK;
    }
    
    return ST95_ERROR;  
}

static uint8_t _st95_cmd_write_reg(const st95_t * dev, uint8_t size_tx, uint8_t addr, uint8_t flag, uint8_t * data_tx)
{   
    if((size_tx < 3) || (size_tx > 4)) {
        return ST95_ERROR;
    }
    
    st95_txbuf[0] = ST95_CMD_WRITE_REG;
    st95_txbuf[1] = size_tx;
    st95_txbuf[2] = addr;
    st95_txbuf[3] = flag;
    
    st95_txbuf[4] = *data_tx;
    
    if(size_tx == 4) {
        st95_txbuf[5] = *(data_tx + 1);
    }
        
    _st95_spi_send(dev, size_tx + 2);
    _st95_wait_ready_data();  

    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {   
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }
    
    return ST95_ERROR;
}

static uint8_t _st95_modify_modulation_gain(const st95_t * dev, uint8_t modul, uint8_t gain)
{  
    uint8_t data[2] = { 0 };
    data[0] = ST95_WR_PTR_MODUL_GAIN;
    data[1] = (modul << 4) | gain;
        
    if(_st95_cmd_write_reg(dev, 4, ST95_WR_ARC_ADDR, ST95_WR_FLAG_INC, data) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}

static uint8_t _st95_read_modulation_gain(const st95_t * dev, uint8_t * modul, uint8_t * gain)
{
    uint8_t data = 0x01;
    if(_st95_cmd_write_reg(dev, 3, ST95_WR_ARC_ADDR, ST95_WR_FLAG_NOT_INC, &data) == ST95_OK) {
        if(_st95_read_reg(dev, st95_rxbuf) == ST95_OK)
        {
            *modul = (st95_rxbuf[2] >> 4) & 0x0F;;
            *gain = st95_rxbuf[2] & 0x0F;
            DEBUG("Modulation: 0x%02X  Gain: 0x%02X\n", *modul, *gain);
            return ST95_OK;
        }
    }

    return ST95_ERROR; 
}
#endif


int st95_cmd_idn(const st95_t * dev, uint8_t * idn, uint8_t * length)
{   
    st95_txbuf[0] = ST95_CMD_IDN;
    st95_txbuf[1] = 0x00;
    
    _st95_spi_send(dev, 2);
        _st95_wait_ready_data();  
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        memcpy(idn, (st95_rxbuf + 2), st95_rxbuf[1]);
        *length = (st95_rxbuf[1] + 2);       
        return ST95_OK;      
    }   	
    return ST95_ERROR;
}

static uint8_t _st95_calibration(const st95_t * dev)
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

    _st95_spi_send(dev, 16);
    
    _st95_wait_ready_data();  
        
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            st95_txbuf[13] = 0xFC;
            _st95_spi_send(dev, 16);
        }
        else {
            return ST95_ERROR;
        }
    }
    
    while(st95_txbuf[13] > 0x02) {
        _st95_wait_ready_data();  
        if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {
            if(st95_rxbuf[2] == 0x02) {
                dac_data_l = st95_txbuf[13];
                dac_data_h = 0xFC;
                return ST95_OK;
            }
            else if(st95_rxbuf[2] == 0x01){
                st95_txbuf[13] -= 0x04;
                _st95_spi_send(dev, 16);
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

static uint8_t _st95_cmd_idle(const st95_t * dev)
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
    
    _st95_spi_send(dev, 16);
    
    state = ST95_IDLE_STATE;
       
    return ST95_OK;
}

/**
 * @brief This function check wake-up state
 * 
 * @param[in]   dev Pointer to ST95 device descriptor
 * 
 * @return 0 if device wake-up
 * @return >0 in case of an error
 */
int st95_is_wake_up(const st95_t * dev)
{
    state = ST95_READY_STATE;

    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            return ST95_WAKE_UP;
        }
    }
    
    return ST95_ERROR;
}

/**
 * @brief This function set st95 in sleep mode
 * 
 * @param[in]   dev Pointer to ST95 device descriptor
 * 
 * @return None
 */
void st95_sleep(st95_t * dev)
{
    _st95_cmd_idle(dev);
}

/**
 * @brief This function select ISO 14443A protocol
 * 
 * @param[in]   dev Pointer to ST95 device descriptor
 * 
 * @return 0 if selecting success
 * @return >0 in case of an error
 */
int _st95_select_iso14443a(const st95_t * dev)
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
    
    _st95_spi_send(dev, 4);
    _st95_wait_ready_data();  
    
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF) == ST95_OK) {       
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }

    return ST95_ERROR;
}

int _st95_cmd_send_receive(const st95_t * dev, uint8_t *data_tx, uint8_t size_tx, uint8_t params, uint8_t * rxbuff, uint16_t size_rx_buff) 
{
	uint8_t length = 0;

    st95_txbuf[0] = ST95_CMD_SEND_RECV;
    st95_txbuf[1] = size_tx + 1;
	
	memcpy(st95_txbuf + 2, data_tx, size_tx);
	length = size_tx + 2;
    
	st95_txbuf[length] = params;
	length++;
    	
	_st95_spi_send(dev, length);
    
      _st95_wait_ready_data();
    
    if(_st95_spi_receive(dev, rxbuff, size_rx_buff) == ST95_OK) {       
        if(rxbuff[0] == ST95_RESULT_CODE_OK) {
            return ST95_OK;
        }       
        else if(rxbuff[0] == ST95_RESULT_CODE_ACK) {
            if((rxbuff[2] == ST95_BYTE_ACK) || (rxbuff[2] == ST95_BYTE_NACK)) {
               if(rxbuff[3] == ST95_RESULT_BYTE) {
                   return ST95_OK;
               }
            }                              
        }        
    }

    return ST95_ERROR;
}


/**
 * @brief This function get UID card
 * 
 * @param[in]   dev Pointer to ST95 device descriptor
 * @param[out]  length_uid UID length
 * @param[out]  uid Card UID
 * @param[out]  sak Card SAK(Select ACK)
 * 
 * @return 0 if Valid UID 
 * @return >0 in case of an error
 */
int st95_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{
    if(_st95_select_iso14443a(dev) == ST95_ERROR) {
        return ST95_ERROR;
    }
#if ST95_ENABLE_GAIN    

    uint8_t modul = 0;
    uint8_t gain = 0;
    
    if(_st95_read_modulation_gain(dev, &modul, &gain) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    if(_st95_modify_modulation_gain(dev, ST95_WR_MODULATION_95, ST95_WR_GAIN_32_DB) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
#endif
                
    if(iso14443a_get_uid(dev, length_uid, uid, sak) == ST95_OK) {
        return ST95_OK;       
    }
 
    return ST95_ERROR;
}

/**
 * @brief ST95 driver initialization routine
 *
 * @param[in]   dev Pointer to ST95 device descriptor
 * @param[in]   params Pointer to static ST95 device configuration
 *
 * @return 0 if initialization succeeded
 * @return >0 in case of an error
 */
int st95_init(st95_t * dev, st95_params_t * params)
{      
    dev->params = *params;
        /* Init SSI_0 pin */
    gpio_init(dev->params.ssi_0, GPIO_OD_PU);
        /* Init IRQ_IN pin */
    gpio_init(dev->params.irq_in, GPIO_OD_PU);
        /* Init VCC_ENABLE pin -> after init st95 is power on! */
    gpio_init(dev->params.vcc, GPIO_OUT);
        /* Number of initializations */
    uint8_t cnt_init = 0;
    
    do {      
            /* ST95 Power Off */
        gpio_set(dev->params.vcc);

            /* Select SPI iface */
        gpio_set(dev->params.ssi_0);
        gpio_set(dev->params.irq_in);

        rtctimers_millis_sleep(ST95_DELAY_POWER_ON_MS);
            
            /* ST95 Power On */
        gpio_clear(dev->params.vcc);
            /* Ramp-up time from 0V to Vps */
        rtctimers_millis_sleep(ST95_RAMP_UP_TIME_MS);
            
         /* Send negative pulse IRQ_IN*/
        gpio_set(dev->params.irq_in);      
        xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
        gpio_clear(dev->params.irq_in);  
        xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
        gpio_set(dev->params.irq_in);
           
            /* Initialize SPI */
        spi_init(SPI_DEV(dev->params.spi));
            /* Initialize CS SPI */
        if(spi_init_cs(SPI_DEV(dev->params.spi), dev->params.cs_spi) != SPI_OK) {
            DEBUG("[ST95]: Error init SPI interface\n");
            return ST95_ERROR;
        }

            /* Init Interrupt */
        gpio_init_int(dev->params.irq_out, GPIO_IN_PU, GPIO_FALLING, _st95_spi_rx, dev);
        gpio_irq_disable(dev->params.irq_out);

            /* HFO setup time */
        rtctimers_millis_sleep(ST95_HFO_SETUP_TIME_MS);

        /* Increase number of init */
        cnt_init++;
        DEBUG("[ST95]: Init: [%d / %d]\n", cnt_init, ST95_NUMB_TRY_INIT); 
        /* Send ECHO cmd (check response from st95) */
    } while((_st95_cmd_echo(dev) != ST95_OK) && (cnt_init < ST95_NUMB_TRY_INIT));
                 
    if(cnt_init >= ST95_NUMB_TRY_INIT) {
            DEBUG("[ST95]: No ECHO\n");
            return ST95_ERROR;
    }
    
        /* Calibration process */
    if(_st95_calibration(dev) != ST95_OK) {
        DEBUG("[ST95]: Calibration error\n");
        return ST95_ERROR;
    }
    
    return ST95_OK;
}

#ifdef __cplusplus
}
#endif