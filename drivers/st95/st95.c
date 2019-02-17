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
#include "periph/uart.h"

#include "st95.h"
#include "st95_params.h"
#include "iso14443a.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define ENABLE_DEBUG_ST95 (0)

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t (*st95_iface_send)(const st95_t *, uint8_t *, uint8_t, bool);
static uint8_t (*st95_iface_receive)(const st95_t *, uint8_t *, uint16_t, bool);

static uint8_t st95_txbuf[ST95_MAX_BYTE_BUFF] = { 0x00 };
static uint8_t st95_rxbuf[ST95_MAX_BYTE_BUFF] = { 0x00 };

static uint16_t length_rx = 0;

static st95_state_t st95_state  = { .data_rx = false, .timeout = false, .mode = ST95_READY_MODE };

static uint8_t _st95_spi_receive(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff, bool cond);
static uint8_t _st95_spi_send(const st95_t * dev, uint8_t * txbuff, uint8_t length_tx, bool cond);

static void _st95_wait_ready_data(void);
static void _st95_send_irqin_low_pulse(const st95_t * dev);

static int _st95_cmd_echo(const st95_t * dev);
static uint8_t _st95_cmd_idle(const st95_t * dev, uint8_t dac_l, uint8_t dac_h);
static uint8_t _st95_cmd_calibration(const st95_t * dev, uint8_t dac_l, uint8_t dac_h);
static uint8_t _st95_calibration(st95_t * dev);

uint8_t _st95_select_field_off(const st95_t * dev);

uint8_t _st95_read_reg(const st95_t * dev, uint8_t * rxbuff);
uint8_t _st95_read_modulation_gain(const st95_t * dev, uint8_t * modul, uint8_t * gain);


/**
 * @brief   This function send low pulse on IRQ_IN pin
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  None
 */
static void _st95_send_irqin_low_pulse(const st95_t * dev)
{    
    gpio_set(dev->params.irq_in);      
    xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    gpio_clear(dev->params.irq_in);  
    xtimer_spin(xtimer_ticks_from_usec(ST95_PULSE_NEGATIVE_USEC));
    gpio_set(dev->params.irq_in);
}

#if ENABLE_DEBUG_ST95
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        for(uint32_t i = 0; i < len; i++){
            printf("%02X ", buff[i]);
            if(i < 2) printf("  ");
        }
        printf("\n");
    }
    
    #define PRINTSTR _printstr
    static void _printstr(char * str)
    {
        printf("%s ", str);
    }
#else
    #define PRINTBUFF(...)
    #define PRINTSTR(...)
#endif
    
    
int st95_read_data(const st95_t * dev, uint8_t * data, uint16_t length)
{
    if(_st95_select_iso14443a(dev, NULL, 1) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    if(_st95_modify_modulation_gain(dev, ST95_WR_MODULATION_95, ST95_WR_GAIN_32_DB) == ST95_ERROR) {
        return ST95_ERROR;
    }

    if(iso14443a_read_tag(dev, data, length, st95_rxbuf) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}    

int st95_write_data(const st95_t * dev, uint8_t * data, uint16_t length)
{
    if(_st95_select_iso14443a(dev, NULL, 1) == ST95_ERROR) {
        return ST95_ERROR;
    }

    if(_st95_modify_modulation_gain(dev, ST95_WR_MODULATION_95, ST95_WR_GAIN_32_DB) == ST95_ERROR) {
        return ST95_ERROR;
    }

    if(iso14443a_write_tag(dev, data, length, st95_rxbuf) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}    


/**
 * @brief   Send a reset sequence over SPI bus.
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 *
 * @return  None
 */
void st95_spi_reset(const st95_t * dev)
{
    uint8_t tx_spi = ST95_CTRL_SPI_RESET;
        // reset the ST95HF data status 
    st95_state.data_rx = false;
    st95_state.timeout = false;
    
    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ST95_SPI_CLK);
         /*Send Reset*/
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, &tx_spi, NULL, 1);
    
    spi_release(SPI_DEV(dev->params.spi));

    xtimer_spin(xtimer_ticks_from_usec(3000));
    // rtctimers_millis_sleep(ST95_DELAY_POWER_ON_MS); 
    
    /* send a low pulse on IRQ_in to wake-up ST95HF device */
    _st95_send_irqin_low_pulse(dev);
        /* HFO setup time (delay before issuing a new command)*/
    rtctimers_millis_sleep(ST95_HFO_SETUP_TIME_MS);  
}

/**
 * @brief   This function switch off RF
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return 0:   if selecting success
 * @return  >0: in case of an error
 */
uint8_t _st95_select_field_off(const st95_t * dev)
{
    st95_txbuf[0] = ST95_CMD_PROTOCOL;
    st95_txbuf[1] = 2;                  // Data Length
    st95_txbuf[2] = FIELD_OFF;
    st95_txbuf[3] = 0x00;

    _st95_spi_send(dev, st95_txbuf, 4, false);
    _st95_wait_ready_data();  
    
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }
    return ST95_ERROR;
}

/**
 * @brief   This function send data over SPI bus
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  txbuff:         Pointer to the transmit buffer
 * @param[out]  size_rx_buff:   Size of the transmit buffer
 * @param[in]   cond:           Condition of SPI release after sending data
 * 
 * @return  0:  data has been successfully transmitted
 */
uint8_t _st95_spi_send(const st95_t * dev, uint8_t * txbuff, uint8_t length_tx, bool cond)
{
    uint8_t tx_spi = ST95_CTRL_SPI_SEND;

    spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ST95_SPI_CLK);

    /*Send command*/
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &tx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, txbuff, NULL, length_tx);

    if(cond) {
        spi_release(SPI_DEV(dev->params.spi));
    }
        
    // reset the ST95HF data status 
    st95_state.data_rx = false;
    st95_state.timeout = false;
    
    memset(txbuff, 0x00, length_tx);
        
    gpio_irq_enable(dev->params.irq_out);
       
    return ST95_OK;
}

/**
 * @brief   This function send data over UART bus
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  txbuff:         Pointer to the transmit buffer
 * @param[out]  size_rx_buff:   Size of the transmit buffer
 * @param[in]   cond:           Condition of SPI release after sending data
 * 
 * @return  0:  data has been successfully transmitted
 */
uint8_t _st95_uart_send(const st95_t * dev, uint8_t * txbuff, uint8_t length_tx, bool cond)
{
    (void) cond;
    
    uart_write(UART_DEV(dev->params.uart), txbuff, length_tx);
        
    // reset the ST95HF data status 
    st95_state.data_rx = false;
    st95_state.timeout = false;
    
    memset(txbuff, 0x00, length_tx);
    memset(st95_rxbuf, 0x00, sizeof(st95_rxbuf));
    length_rx = 0;
       
    return ST95_OK;
}

uint8_t _st95_send_pack(const st95_t * dev, uint8_t * txbuff, uint8_t length_tx, bool cond)
{	
	return ((*st95_iface_send)(dev, txbuff, length_tx, cond));
}


/**
 * @brief   This function receive data over SPI bus
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[out]  size_rx_buff:   Size of the receive buffer
 * @param[in]   cond:           Condition of SPI acquire after sending data
 * 
 * @return  0:  data has been successfully received
 * @return  >0: in case of an error
 */
static uint8_t _st95_spi_receive(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff, bool cond)
{ 
    uint8_t rx_spi = ST95_CTRL_SPI_READ;
    length_rx = 0;
             
    gpio_irq_disable(dev->params.irq_out);
    
    if(size_rx_buff > ST95_MAX_BYTE_BUFF) {
        size_rx_buff = ST95_MAX_BYTE_BUFF;
    }
    
    if(cond) {
        spi_acquire(SPI_DEV(dev->params.spi), dev->params.cs_spi, SPI_MODE_0, ST95_SPI_CLK);
    }
    
    memset(rxbuff, 0x00, size_rx_buff);
         
    if(st95_state.timeout == true) {
        return ST95_NO_DEVICE;
	}
     
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, true, &rx_spi, NULL, 1);
    spi_transfer_bytes(SPI_DEV(dev->params.spi), dev->params.cs_spi, false, NULL, rxbuff, size_rx_buff);

    spi_release(SPI_DEV(dev->params.spi));   

    if(rxbuff[0] == ST95_CMD_ECHO) {
        if((rxbuff[1] == 0x85) && ((rxbuff[2] == 0x00))) {
            length_rx = 3;
        }
        else {
            length_rx = 1;
        }
    }
    else {
        length_rx = rxbuff[1] + 2;
        
        if((length_rx - 2) != rxbuff[1]) {
            return ST95_ERROR;
        }
    }     
    
    return ST95_OK;
}

/**
 * @brief   This function receive data over UART bus
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[out]  size_rx_buff:   Size of the receive buffer
 * @param[in]   cond:           Condition of SPI acquire after sending data
 * 
 * @return  0:  data has been successfully received
 * @return  >0: in case of an error
 */
uint8_t _st95_uart_receive(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff, bool cond)
{    
    (void) cond;
    (void) dev;   
    
    if(size_rx_buff > ST95_MAX_BYTE_BUFF) {
        size_rx_buff = ST95_MAX_BYTE_BUFF;
    }
    
    uint16_t length_rx_tmp = 0;
         
    if(st95_state.timeout == true) {
        return ST95_NO_DEVICE;
	}

    if(rxbuff[0] == ST95_CMD_ECHO) {
        if((rxbuff[1] == 0x85) && ((rxbuff[2] == 0x00))) {
            length_rx_tmp = 3;
        }
        else {
            length_rx_tmp = 1;
        }
    }
    else {
        length_rx_tmp = rxbuff[1] + 2;
        
        if((length_rx_tmp - 2) != rxbuff[1]) {
            return ST95_ERROR;
        }
    }     
    
    return ST95_OK;
}

uint8_t _st95_receive_pack(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff, bool cond)
{	
	return ((*st95_iface_receive)(dev, rxbuff, size_rx_buff, cond));
}

/**
 * @brief   This function wait the data from device
 * 
 * @return  None
 */
static void _st95_wait_ready_data(void)
{
    uint32_t time_begin = rtctimers_millis_now();
    uint32_t time_end = 0;
    uint32_t time_delta = 0;
    st95_state.timeout = false;
        
    xtimer_spin(xtimer_ticks_from_usec(ST95_NO_RESPONSE_TIME_MIN_USEC));
    
	while((st95_state.data_rx == false) && (st95_state.timeout == false)) {
		time_end = rtctimers_millis_now();
		time_delta = time_end - time_begin;
		if(time_delta > ST95_NO_RESPONSE_TIME_MS) {
            st95_state.timeout = true;
		}
        xtimer_spin(xtimer_ticks_from_usec(ST95_NO_RESPONSE_TIME_MIN_USEC));
	}
}

/**
 * @brief   Callback SPI
 * 
 * @return  None
 */
static void _st95_spi_rx(void* arg)
{
    st95_t *dev = arg;
    
    st95_state.data_rx = true;    

    if(st95_state.mode == ST95_SLEEP_MODE) {
        if (dev->cb) {
            dev->cb(dev->arg);
        }
    }
         
    return;
}

/**
 * @brief   Callback UART
 * 
 * @return  None
 */
static void _st95_uart_rx(void *arg, uint8_t data)
{
    st95_t *dev = arg;

    st95_rxbuf[length_rx++] = data;
    
    if(st95_rxbuf[0] == ST95_CMD_ECHO) {
        st95_state.data_rx = true;
		return;
	}
	
	if(length_rx >= 2) {
		if((length_rx - 2) == st95_rxbuf[1]) {
            st95_state.data_rx = true;
            
            if(st95_state.mode == ST95_SLEEP_MODE) {
                if (dev->cb) {
                    dev->cb(dev->arg);
                }
            }
		}
	}
         
    return;
}

/**
 * @brief   This function verifies the response from device
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  0:  if is response
 * @return  >0: in case of an error
 */
static int _st95_cmd_echo(const st95_t * dev)
{   
    st95_txbuf[0] = ST95_CMD_ECHO;

    _st95_send_pack(dev, st95_txbuf, 1, false);
    _st95_wait_ready_data();

    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if(st95_rxbuf[0] == ST95_CMD_ECHO) {
            return ST95_OK;
        }
        else {
            return ST95_NO_DEVICE;
        }
    }

    return ST95_ERROR;
}

/**
 * @brief   This function allows exiting Listening mode
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  0:  if is response
 * @return  >0: in case of an error
 */
int _st95_exit_listen(const st95_t * dev)
{   
    st95_txbuf[0] = ST95_CMD_ECHO;

    _st95_send_pack(dev, st95_txbuf, 1, true);

    _st95_wait_ready_data();

    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, true) == ST95_OK) {
        if((st95_rxbuf[0] == ST95_CMD_ECHO) && (st95_rxbuf[1] == 0x85) && (st95_rxbuf[2] == 0x00)) {
            return ST95_OK;
        }
    }
    
    return ST95_ERROR;
}

uint8_t _st95_read_reg(const st95_t * dev, uint8_t * rxbuff)
{    
    st95_txbuf[0] = ST95_CMD_READ_REG;
    st95_txbuf[1] = 3;
    st95_txbuf[2] = ST95_READ_ADDR_1;
    st95_txbuf[3] = ST95_REG_SIZE;
    st95_txbuf[4] = ST95_ST_RESERVED;
    
    _st95_send_pack(dev, st95_txbuf, 5, false);
    _st95_wait_ready_data();

    if(_st95_receive_pack(dev, rxbuff, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {     
        return ST95_OK;
    }
    
    return ST95_ERROR;  
}

uint8_t _st95_cmd_write_reg(const st95_t * dev, uint8_t size_tx, uint8_t addr, uint8_t flag, uint8_t * data_tx)
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
        
    _st95_send_pack(dev, st95_txbuf, size_tx + 2, false);
    _st95_wait_ready_data();  

    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {     
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }

    return ST95_ERROR;
}

uint8_t _st95_modify_modulation_gain(const st95_t * dev, uint8_t modul, uint8_t gain)
{  
    uint8_t data[2] = { 0 };
    data[0] = ST95_WR_PTR_MODUL_GAIN;
    data[1] = (modul << 4) | gain;
        
    if(_st95_cmd_write_reg(dev, 4, ST95_WR_ARC_ADDR, ST95_WR_FLAG_INC, data) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}

uint8_t _st95_set_timer_window(const st95_t * dev, uint8_t timer_w)
{  
    uint8_t data[2] = { 0 };
    data[0] = timer_w;
    data[1] = ST95_WR_TIMER_WINDOW_CONFIRM;
        
    if(_st95_cmd_write_reg(dev, 4, ST95_WR_TIMER_WINDOW, ST95_WR_FLAG_NOT_INC, data) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;
}

uint8_t _st95_read_modulation_gain(const st95_t * dev, uint8_t * modul, uint8_t * gain)
{
    uint8_t data = 0x01;
    if(_st95_cmd_write_reg(dev, 3, ST95_WR_ARC_ADDR, ST95_WR_FLAG_NOT_INC, &data) == ST95_OK) {
        if(_st95_read_reg(dev, st95_rxbuf) == ST95_OK)
        {
            if(modul != NULL) {
                *modul = (st95_rxbuf[2] >> 4) & 0x0F;
                DEBUG("Modulation: 0x%02X\t", *modul);
            }
            if(gain != NULL) {
                *gain = st95_rxbuf[2] & 0x0F;
                DEBUG("GAIN: 0x%02X", *gain);
            }
            DEBUG("\n");
            return ST95_OK;
        }
    }

    return ST95_ERROR; 
}

/**
 * @brief   This function gives brief information about the ST95HF and its revision
 * 
 * @param[in]   dev:        Pointer to ST95 device descriptor
 * @param[out]  idn:        Pointer to the receive buffer of the info
 * @param[out]  length:     Pointer to length of the info
 * 
 * @return  0:  Valid information
 * @return  1:  in case of an error
 */
int st95_cmd_idn(const st95_t * dev, uint8_t * idn, uint8_t * length)
{   
    st95_txbuf[0] = ST95_CMD_IDN;
    st95_txbuf[1] = 0x00;
    
    _st95_send_pack(dev, st95_txbuf, 2, false);
    _st95_wait_ready_data(); 
    
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        memcpy(idn, (st95_rxbuf + 2), st95_rxbuf[1]);
        *length = (st95_rxbuf[1] + 2);       
        return ST95_OK;      
    }   	
    return ST95_ERROR;
}

/**
 * @brief   This function send calibration command
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * @param[in]   dac_l:  DacDataL value (Lower compare value for tag detection)
 * @param[in]   dac_h:  DacDataH value (Higher compare value for tag detection)
 * 
 * @return  0:  the command has been successfully executed
 */
static uint8_t _st95_cmd_calibration(const st95_t * dev, uint8_t dac_l, uint8_t dac_h)
{
    st95_txbuf[0] = ST95_CMD_IDLE;    // Command
    st95_txbuf[1] = 14;               // Data Length
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
    st95_txbuf[12] = dac_l;            // DacDataL
    st95_txbuf[13] = dac_h;            // DacDataH

        /* Swing Count */
    st95_txbuf[14] = 0x3F;            // Recommendeded value 0x3F
        /* Max Sleep */
    st95_txbuf[15] = 0x01;            // This value must be set to 0x01 during tag detection calibration

    _st95_send_pack(dev, st95_txbuf, 16, false);
    
    return ST95_OK;
}

/**
 * @brief   This function execute the calibration process
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  0:  calibration done
 * @return  1:  in case of an error
 */
static uint8_t _st95_calibration(st95_t * dev)
{      
    uint8_t dac_l = 0;
    uint8_t dac_h = 0;
    
    _st95_cmd_calibration(dev, dac_l, dac_h);
    _st95_wait_ready_data();  
        
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            dac_l = 0x00;
            dac_h = 0xFC;
            _st95_cmd_calibration(dev, dac_l, dac_h);
        }
        else {
            return ST95_ERROR;
        }
    }
    
    while(dac_h > 0x02) {
        _st95_wait_ready_data();  
        if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
            if(st95_rxbuf[2] == 0x02) {              
                dev->params.dac_l = dac_h;
                dev->params.dac_h = 0xFC;
                return ST95_OK;
            }
            else if(st95_rxbuf[2] == 0x01){
                dac_h -= 0x04;
                _st95_cmd_calibration(dev, dac_l, dac_h);
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

/**
 * @brief   This function send POLLFIELD command
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  0:  the command has been successfully executed
 */
uint8_t _st95_cmd_poll_field(const st95_t * dev)
{    
    st95_txbuf[0] = ST95_CMD_POLL_FIELD;    // Command
    st95_txbuf[1] = 0x00;                // Data Length
    
    _st95_send_pack(dev, st95_txbuf, 2, false);
    
    _st95_wait_ready_data();  
    
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if(st95_rxbuf[0] == 0x00) {
            if(st95_rxbuf[1] == 0x00) {
                return ST95_ERROR;
            }
            else if ((st95_rxbuf[1] == 0x01) && (st95_rxbuf[2] == 0x00)) {
                return ST95_ERROR;
            }
            else if ((st95_rxbuf[1] == 0x01) && (st95_rxbuf[2] == 0x01)) {
                return ST95_OK;
            }
            else {
                return ST95_ERROR;
            }
        }
        else {
            return ST95_ERROR;
        }
    }
    
    return ST95_ERROR;
}

/**
 * @brief   This function send LISTEN command
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return  0:  the command has been successfully executed
 */
uint8_t _st95_cmd_listen(const st95_t * dev)
{    
    st95_txbuf[0] = ST95_CMD_LISTEN;    // Command
    st95_txbuf[1] = 00;                // Data Length
    
    _st95_send_pack(dev, st95_txbuf, 2, false);
    
    _st95_wait_ready_data();  
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;    
        }
        return ST95_ERROR;
    }
           
    return ST95_ERROR;
}

/**
 * @brief   This function send IDLE command
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * @param[in]   dac_l:  DacDataL value (Lower compare value for tag detection)
 * @param[in]   dac_h:  DacDataH value (Higher compare value for tag detection)
 * 
 * @return  0:  the command has been successfully executed
 */
static uint8_t _st95_cmd_idle(const st95_t * dev, uint8_t dac_l, uint8_t dac_h)
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
    st95_txbuf[12] = dac_l;     //0x42;            // DacDataL
    st95_txbuf[13] = dac_h;     //0xFC;            // DacDataH
        /* Swing Count */
    st95_txbuf[14] = 0x3F;            // Recommendeded value 0x3F
        /* Max Sleep */
    st95_txbuf[15] = 0x08;            // Typical value 0x28
    
    _st95_send_pack(dev, st95_txbuf, 16, true);
    
    st95_state.mode = ST95_SLEEP_MODE;
           
    return ST95_OK;
}

/**
 * @brief This function check wake-up state
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return 0:   if device wake-up
 * @return 1:   in case of an error
 */
int st95_is_wake_up(const st95_t * dev)
{
    st95_state.mode = ST95_READY_MODE;
    
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, true) == ST95_OK) {
        if(st95_rxbuf[2] == 0x02) {
            return ST95_WAKE_UP;
        }
    }
    
    return ST95_ERROR;
}

/**
 * @brief This function set st95 in sleep mode
 * 
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * 
 * @return None
 */
void st95_sleep(st95_t * dev)
{
    if(st95_state.mode == ST95_SLEEP_MODE) {
        return;
    }

    _st95_cmd_idle(dev, dev->params.dac_l, dev->params.dac_h);
}

/**
 * @brief This function select ISO 14443A protocol
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[in]   params:         Pointer to protocol parameters
 * @param[in]   length_params:  Parameters length (valid value is 1 or 4)
 * 
 * @return 0:   if selecting success
 * @return 1:   in case of an error
 */
int _st95_select_iso14443a(const st95_t * dev, uint8_t * params, uint8_t length_params)
{
    uint8_t length = 0;
    
    st95_txbuf[0] = ST95_CMD_PROTOCOL;
    st95_txbuf[1] = length;    // Data Length
    st95_txbuf[2] = ISO_14443A;
    if(params == NULL) {
        st95_txbuf[3] = 0x00 | (ST95_TX_RATE_14443A << 6) | (ST95_RX_RATE_14443A << 4);
        length = 2;
    }
    else if((length_params == 4) && (params != NULL)){
        st95_txbuf[3] = *(params);    // TX/RX rate
        st95_txbuf[4] = *(params + 1);    // PP (Optioanal)
        st95_txbuf[5] = *(params + 2);    // MM (Optioanal)
        st95_txbuf[6] = *(params + 3);    // DD (Optioanal) 
        length = 5;        
    }
    else {
        DEBUG("[st95]: Invalid parameters length\n");
        return ST95_ERROR;       
    }

    /* TODO: RFU*/
    /* st95_txbuf[7] = 0x00;    // ST Reserved (Optioanal)
    st95_txbuf[8] = 0x00;    // ST Reserved (Optioanal) */
    
    st95_txbuf[1] = length;
       
    _st95_send_pack(dev, st95_txbuf, length + 2, false);
    _st95_wait_ready_data();  
    
    if(_st95_receive_pack(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }
    return ST95_ERROR;
}

/**
 * @brief This function select ISO 14443A protocol Card emulation
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * 
 * @return 0:   if selecting success
 * @return 1:   in case of an error
 */
int _st95_select_iso14443a_card(const st95_t * dev)
{    
    st95_txbuf[0] = ST95_CMD_PROTOCOL;
    st95_txbuf[1] = 2;    // Data Length
    st95_txbuf[2] = CARD_ISO14443A;
    
    st95_txbuf[3] = 0x00 | (ST95_14443A_CARD_WAIT_RF << 3) | (ST95_14443A_CARD_HFO << 1);
    st95_txbuf[3] |= (ST95_TX_RATE_14443A << 6) | (ST95_RX_RATE_14443A << 4);

    _st95_spi_send(dev, st95_txbuf, 4, false);
    _st95_wait_ready_data();  
    
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }

    return ST95_ERROR;
}

/**
 * @brief send AC FILTER command
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * 
 * @return 0:   if selecting success
 * @return 1:   in case of an error
 */
int _st95_cmd_ac_filter(const st95_t * dev)
{    
    st95_txbuf[0] = ST95_CMD_ANTICOL_FILTER;
    st95_txbuf[1] = 7;    // Data Length
    
    st95_txbuf[2] = 0x04; // ATQA
    st95_txbuf[3] = 0x00; // ATQA
    
    st95_txbuf[4] = 0x08; // SAK

    st95_txbuf[5] = 0xAA; // UID
    st95_txbuf[6] = 0xBB; // UID
    st95_txbuf[7] = 0xCC; // UID
    st95_txbuf[8] = 0xDD; // UID

    _st95_spi_send(dev, st95_txbuf, 9, false);
    _st95_wait_ready_data();  
    
    if(_st95_spi_receive(dev, st95_rxbuf, ST95_MAX_BYTE_BUFF, false) == ST95_OK) {
        if((st95_rxbuf[0] == 0x00) && (st95_rxbuf[1] == 0x00)) {
            return ST95_OK;
        }
    }

    return ST95_ERROR;
}


/**
 * @brief   This function send SEND_RECEIVE command
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[in]   data_tx:        Pointer to the transmit buffer
 * @param[in]   size_tx:        Size of the transmit buffer
 * @param[in]   params:         Command parameters 
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[out]  size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
int _st95_cmd_send_receive(const st95_t * dev, uint8_t *data_tx, uint8_t size_tx, uint8_t params, uint8_t * rxbuff, uint16_t size_rx_buff) 
{
	uint8_t length = 0;
    
    st95_txbuf[0] = ST95_CMD_SEND_RECV;
    st95_txbuf[1] = size_tx + 1;
	
	memcpy(st95_txbuf + 2, data_tx, size_tx);
	length = size_tx + 2;

	st95_txbuf[length] = params;
	length++;
      
    _st95_send_pack(dev, st95_txbuf, length, false);
    
    _st95_wait_ready_data();
    
    if(_st95_receive_pack(dev, rxbuff, size_rx_buff, false) == ST95_OK) {
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
        else {
            if(data_tx[0] != ISO14443A_CMD_HLTA) {
                DEBUG("[st95]: Error 0x%02X\n", rxbuff[0]);
            }
            return ST95_ERROR;
        }
    }

    return ST95_ERROR;
}


/**
 * @brief   This function get UID card
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  length_uid:     Pointer to the length UID
 * @param[out]  uid:            Pointer to the card UID
 * @param[out]  sak:            Pointer to the card SAK(Select ACK) byte
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
int st95_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{   
    st95_state.mode = ST95_READY_MODE;

    if(_st95_select_iso14443a(dev, NULL, 1) == ST95_ERROR) {
        return ST95_ERROR;
    }

    if(_st95_modify_modulation_gain(dev, ST95_WR_MODULATION_95, ST95_WR_GAIN_32_DB) == ST95_ERROR) {
        return ST95_ERROR;
    }

    if(iso14443a_get_uid(dev, st95_rxbuf, length_uid, uid, sak) == ST95_OK) {
        return ST95_OK;       
    }
    
    return ST95_ERROR;   
}

/**
 * @brief   This function set UID card
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  length_uid:     Pointer to the length UID
 * @param[out]  uid:            Pointer to the card UID
 * @param[out]  sak:            Pointer to the card SAK(Select ACK) byte
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
int st95_set_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{   
(void) length_uid;
(void) uid;
(void) sak;

     if(st95_state.mode == ST95_LISTEN_MODE) {
        if(_st95_exit_listen(dev) != ST95_OK) {
            return ST95_ERROR;
        }
     }
     st95_state.mode = ST95_READY_MODE;

    if(_st95_select_iso14443a_card(dev) != ST95_OK) {
        return ST95_ERROR;
    }
 
    if(_st95_cmd_ac_filter(dev) != ST95_OK) {
        return ST95_ERROR;
    }

    if(_st95_cmd_poll_field(dev) != ST95_OK) {
        return ST95_ERROR;
    }

    if(_st95_cmd_listen(dev) != ST95_OK) {
        return ST95_ERROR;
    }
    st95_state.mode = ST95_LISTEN_MODE;
    return ST95_OK;   
}

/**
 * @brief ST95 driver initialization routine SPI
 *
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * @param[in]   params: Pointer to static ST95 device configuration
 *
 * @return 0:   if initialization succeeded
 * @return >0:  in case of an error
 */
int _st95_init_spi(st95_t * dev, st95_params_t * params)
{      
    dev->params = *params;
    dev->arg = NULL;
    dev->params.dac_l = 0x00;
    dev->params.dac_h = 0x00;
    
    st95_iface_send = _st95_spi_send;
    st95_iface_receive = _st95_spi_receive; 
    
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
        _st95_send_irqin_low_pulse(dev);
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

            /* HFO setup time (delay before issuing a new command)*/
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

/**
 * @brief ST95 driver initialization routine UART
 *
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * @param[in]   params: Pointer to static ST95 device configuration
 *
 * @return 0:   if initialization succeeded
 * @return >0:  in case of an error
 */
int _st95_init_uart(st95_t * dev, st95_params_t * params)
{      
    dev->params = *params;
    dev->arg = NULL;
    dev->params.dac_l = 0x00;
    dev->params.dac_h = 0x00;
    
    st95_iface_send = _st95_uart_send;
    st95_iface_receive = _st95_uart_receive; 

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

            /* Select UART iface */
        gpio_clear(dev->params.ssi_0);
        gpio_set(dev->params.irq_in);

        rtctimers_millis_sleep(ST95_DELAY_POWER_ON_MS);
            
            /* ST95 Power On */
        gpio_clear(dev->params.vcc);
            /* Ramp-up time from 0V to Vps */
        rtctimers_millis_sleep(ST95_RAMP_UP_TIME_MS);
            
         /* Send negative pulse IRQ_IN*/
        _st95_send_irqin_low_pulse(dev);

        gpio_init_af(dev->params.irq_in, GPIO_AF7);

        /* Initialize UART */
        if (uart_init(UART_DEV(dev->params.uart), dev->params.baudrate, _st95_uart_rx, dev) != UART_OK) {
            DEBUG("[ST95]: Error initializing UART interface\n");
            return ST95_ERROR;
        }
           
            /* HFO setup time (delay before issuing a new command)*/
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

/**
 * @brief ST95 driver initialization routine
 *
 * @param[in]   dev:    Pointer to ST95 device descriptor
 * @param[in]   params: Pointer to static ST95 device configuration
 *
 * @return 0:   if initialization succeeded
 * @return >0:  in case of an error
 */
int st95_init(st95_t * dev, st95_params_t * params)
{      
    if (params->iface == ST95_IFACE_SPI) {
        return _st95_init_spi(dev, params);
    }
    else if (params->iface == ST95_IFACE_UART) {
        return _st95_init_uart(dev, params);
    }
    else {
         return ST95_ERROR;
    }
     
    return ST95_ERROR;
}

#ifdef __cplusplus
}
#endif