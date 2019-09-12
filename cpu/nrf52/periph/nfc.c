/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

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
 * @author      Mikhail Perkov
 */
 
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>

#include "board.h"
#include "periph/gpio.h"
#include "periph/nfc.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

static volatile uint8_t rx_buffer[NRF_NFC_RXTX_BUFFER_SIZE] = { 0x00 };
static volatile uint8_t tx_buffer[NRF_NFC_RXTX_BUFFER_SIZE] = { 0x00 };
static uint8_t cc_file[NRF_NFC_CC_FILE_SIZE] = {};

static inline void nrf_nfc_enable_int(uint32_t interrupt);
static inline void nrf_nfc_disable_int(uint32_t interrupt);
static inline void nrf_nfc_clear_event(volatile uint32_t * event);
static inline void nrf_nfc_clear_errors(void);
static inline void nrf_nfc_clear_rx_status(void);
static inline void nrf_nfc_task(volatile uint32_t * task);

static void nfc_set_ats(uint8_t param_byte);

static void nfc_create_cc_file(void);
static void nfc_cc_file(uint8_t pcb, uint8_t p1, uint8_t p2, uint8_t length);

static void nfc_ndef_file(uint8_t pcb, uint8_t p1, uint8_t p2, uint8_t length);

static nrf_nfc_tag_t current_tag = { .ndef_data = NULL, .tx_length = 0, .ndef_size = 0, 
                                     .mode = NRF_NFC_MODE_UID_TAG, .state = NRF_NFC_STATE_NONE };

static void nfc_ndef_file(uint8_t pcb, uint8_t p1, uint8_t p2, uint8_t length)
{
    tx_buffer[0] = pcb;
    uint16_t offset = (p1 << 8) | p2;

    uint8_t * ndef_data_tmp = current_tag.ndef_data + offset;

    for(uint32_t i = 0; i < length; i++) {
        tx_buffer[i + 1] = *ndef_data_tmp;
        ndef_data_tmp++;
    }
    current_tag.tx_length = length + 1;
}

static void nfc_cc_file(uint8_t pcb, uint8_t p1, uint8_t p2, uint8_t length)
{
    uint16_t offset = (p1 << 8) | p2;
    tx_buffer[0] = pcb;

    for(uint32_t i = offset; i < length; i++) {
        tx_buffer[i + 1] = cc_file[i];
    }
    current_tag.tx_length = length + 1;
    
    if(length == NRF_NFC_CC_FILE_SIZE) {
        current_tag.state = NRF_NFC_STATE_NDEF_FILE;
    }
}

static void nfc_create_cc_file(void)
{
    cc_file[0] = NRF_NFC_CC_FILE_SIZE >> 8;
    cc_file[1] = NRF_NFC_CC_FILE_SIZE & 0xFF;                /**< 15 bytes */
    /* Mapping version */
    cc_file[2] = 0x20;
    /* Mle bytes (max data size that can be read) */
    cc_file[3] = 0x00;
    cc_file[4] = 0xF6;
    /* Mlc bytes (max data size that can be write)*/
    cc_file[5] = 0x00;
    cc_file[6] = 0xF6;
    /* TLV block */
    cc_file[7] = 0x04; /**< Tag field value: 0x04 */
    cc_file[8] = 0x06; /**< Tag length value: 0x06 bytes */
    cc_file[9] = 0x00; /**< NDEF ID (first byte) */
    cc_file[10] = 0x01; /**< NDEF ID (second byte) */
 
    cc_file[11] = NRF_NFC_NDEF_SIZE_MAX >> 8;     /**< Max NDEF file size (first byte) */
    cc_file[12] = NRF_NFC_NDEF_SIZE_MAX & 0xFF;   /**< Max NDEF file size (second byte) */
    
    cc_file[13] = 0x00; /**< NDEF file read access condition */
    cc_file[14] = 0x00; /**< NDEF file write access condition */
}


static void nfc_set_ats(uint8_t param_byte)
{  
    uint8_t tl = 0; // length
    uint8_t t0 = 0; // format byte
    uint8_t ta1 = 0; // iface byte 1
    uint8_t tb1 = 0; // iface byte 2
    uint8_t tc1 = 0; // iface byte 3
    uint8_t fsci = (param_byte >> 4) & 0x0F; // Frame Size for proximity Card Integer

    tl = 5;
    t0 = ( (NRF_NFC_ATS_IS_TC1 << 6) | (NRF_NFC_ATS_IS_TB1 << 5) | (NRF_NFC_ATS_IS_TA1 << 4) | (fsci << 0)) & 0x7F;
    
    if(NRF_NFC_ATS_IS_TA1) {
        ta1 = 0x80; // 0x80 -> (1 000 0 000)b
    }
    if(NRF_NFC_ATS_IS_TB1) {
        tb1 = (NRF_NFC_ATS_FWI << 4) | (NRF_NFC_ATS_SFGI << 0);
    }
    if(NRF_NFC_ATS_IS_TC1) {
        tc1 = ((NRF_NFC_ATS_IS_CID << 1) | (NRF_NFC_ATS_IS_NAD << 0)) & 0x03;
    }
    tx_buffer[0] = tl; 
    tx_buffer[1] = t0;
    tx_buffer[2] = ta1;
    tx_buffer[3] = tb1;
    tx_buffer[4] = tc1;
}


static inline void nrf_nfc_enable_int(uint32_t interrupt)
{   
    NRF_NFCT->INTENSET = interrupt;
}

static inline void nrf_nfc_disable_int(uint32_t interrupt)
{   
    NRF_NFCT->INTENCLR = interrupt;
}

static inline void nrf_nfc_clear_event(volatile uint32_t * event)
{   
    *event = 0x0UL;
    __DSB();
}

static inline void clear_buffers(void)
{
    for(uint32_t i = 0; i < 10; i++) {
        rx_buffer[i] = 0;
        tx_buffer[i] = 0;
    }
}

static inline void nrf_nfc_clear_errors(void) 
{  
    if(current_tag.mode == NRF_NFC_MODE_DATA_RXTX) {  
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_NFCFIELDTOOWEAK_Msk) {
            puts("\t[ERROR]: Field level is too LOW at MIN load resistance");
        }
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_NFCFIELDTOOSTRONG_Msk) {
            puts("\t[ERROR]: Field level is too HIGH at MAX load resistance");
        }
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_FRAMEDELAYTIMEOUT_Msk) {
            puts("\t[ERROR]: No STARTTX task triggered before expiration of the time set in FRAMEDELAYMAX");
        }
    }
    
    NRF_NFCT->ERRORSTATUS = NRF_NFC_ALL_ERRORS;
}

static inline void nrf_nfc_clear_rx_status(void) 
{   
    if(current_tag.mode == NRF_NFC_MODE_DATA_RXTX) {
        if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_OVERRUN_Msk) {
            puts("\t[RX STATUS]: Overrun");
        }
    }
    if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_PARITYSTATUS_Msk) {
        puts("\t[RX STATUS]: Parity Error");
    }
    if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_CRCERROR_Msk) {
        puts("\t[RX STATUS]: CRC Error");
    }
    
    NRF_NFCT->FRAMESTATUS.RX = NRF_NFC_ALL_RX_STATUS;
}

static inline void nrf_nfc_task(volatile uint32_t * task)
{
    *task = 0x1UL;
}

void isr_nfct(void)
{
    if(NRF_NFCT->EVENTS_FIELDDETECTED && (NRF_NFCT->INTEN & NFCT_INTEN_FIELDDETECTED_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDDETECTED);
    }
    
    if(NRF_NFCT->EVENTS_FIELDLOST && (NRF_NFCT->INTEN & NFCT_INTEN_FIELDLOST_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDLOST);
    }
    
    if(NRF_NFCT->EVENTS_RXFRAMEEND && (NRF_NFCT->INTEN & NFCT_INTEN_RXFRAMEEND_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXFRAMEEND);
        
        /* Take into account only number of whole bytes */
        // uint32_t rx_data_size = ((NRF_NFCT->RXD.AMOUNT & NFCT_RXD_AMOUNT_RXDATABYTES_Msk) >> NFCT_RXD_AMOUNT_RXDATABYTES_Pos) - NFC_CRC_SIZE;
        // printf("\tRX size: %ld -> ", rx_data_size);
        
        if (rx_buffer[0] == NRF_NFC_ISO14443A_CMD_RATS) {
            nfc_set_ats(rx_buffer[1]);
            current_tag.tx_length = 5;
            NRF_NFCT->PACKETPTR          = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN             = NRF_NFC_RXTX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (current_tag.tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;

            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);
            nrf_nfc_enable_int(NFCT_INTEN_TXFRAMEEND_Msk);
            current_tag.state = NRF_NFC_STATE_CC_FILE;
        }
        else  if ((rx_buffer[0] == NRF_NFC_ISO7816_IBLOCK_02) || (rx_buffer[0] == NRF_NFC_ISO7816_IBLOCK_03)) {
            if(rx_buffer[2] == NRF_NFC_ISO7816_READ_BINARY) {        
                if(current_tag.state == NRF_NFC_STATE_CC_FILE) {
                /* CC file */                       
                    nfc_cc_file(rx_buffer[0], rx_buffer[3], rx_buffer[4], rx_buffer[5]);
                }
                else if(current_tag.state == NRF_NFC_STATE_NDEF_FILE) {
                /* NDEF file */                   
                    nfc_ndef_file(rx_buffer[0], rx_buffer[3], rx_buffer[4], rx_buffer[5]);
                }
            }
            
            tx_buffer[current_tag.tx_length] = NRF_NFC_NDEF_OK_SW_1;
            current_tag.tx_length++;
            tx_buffer[current_tag.tx_length] = NRF_NFC_NDEF_OK_SW_2;
            current_tag.tx_length++;

            NRF_NFCT->PACKETPTR          = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN             = NRF_NFC_RXTX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (current_tag.tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;
            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);
            
            nrf_nfc_enable_int(NFCT_INTEN_TXFRAMEEND_Msk);
        }
        else  if (rx_buffer[0] == NRF_NFC_ISO14443A_CMD_HLTA) {
            nrf_nfc_disable_int(NFCT_INTEN_RXFRAMEEND_Msk);
            nrf_nfc_disable_int(NFCT_INTEN_RXERROR_Msk);
            nrf_nfc_clear_errors();
            nrf_nfc_clear_rx_status();
            current_tag.mode = NRF_NFC_MODE_UID_TAG;
        }
        else  if (rx_buffer[0] == NRF_NFC_ISO14443A_CMD_DESELECT) {
            tx_buffer[0] = NRF_NFC_ISO14443A_CMD_DESELECT;
            current_tag.tx_length = 1;
             
            NRF_NFCT->PACKETPTR  = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN     = NRF_NFC_RXTX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (current_tag.tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;
            
            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);
            nrf_nfc_enable_int(NFCT_INTEN_TXFRAMEEND_Msk);
        }
    }
    
    if(NRF_NFCT->EVENTS_TXFRAMEEND && (NRF_NFCT->INTEN & NFCT_INTEN_TXFRAMEEND_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMEEND);
        
        nrf_nfc_clear_errors();
        nrf_nfc_clear_rx_status();
        clear_buffers();
        // /* Set up registers for EasyDMA and start receiving packets */
        NRF_NFCT->PACKETPTR = (uint32_t) rx_buffer;
        NRF_NFCT->MAXLEN = NRF_NFC_RXTX_BUFFER_SIZE;
        nrf_nfc_task(&NRF_NFCT->TASKS_ENABLERXDATA);
        nrf_nfc_enable_int(NFCT_INTEN_RXFRAMEEND_Msk);
    }
    
    if(NRF_NFCT->EVENTS_RXERROR && (NRF_NFCT->INTEN & NFCT_INTEN_RXERROR_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXERROR);
        nrf_nfc_clear_rx_status();
    }
    
    if(NRF_NFCT->EVENTS_ERROR && (NRF_NFCT->INTEN & NFCT_INTEN_ERROR_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_ERROR);
        nrf_nfc_clear_errors();
    }
    
    if(NRF_NFCT->EVENTS_SELECTED && (NRF_NFCT->INTEN & NFCT_INTEN_SELECTED_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_SELECTED);

        current_tag.state = NRF_NFC_STATE_RATS;
        nrf_nfc_clear_errors();
        nrf_nfc_clear_rx_status();
        current_tag.mode = NRF_NFC_MODE_DATA_RXTX;

        /* Set up registers for EasyDMA and start receiving packets */
        NRF_NFCT->PACKETPTR          = (uint32_t) rx_buffer;
        NRF_NFCT->MAXLEN             = 15;
        nrf_nfc_enable_int(NFCT_INTEN_RXFRAMEEND_Msk);
        nrf_nfc_task(&NRF_NFCT->TASKS_ENABLERXDATA);
        
        nrf_nfc_enable_int(NFCT_INTEN_RXERROR_Msk);
    }      
    
    if(NRF_NFCT->EVENTS_TXFRAMESTART && (NRF_NFCT->INTEN & NFCT_INTEN_TXFRAMESTART_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMESTART);
    }
    
    cortexm_isr_end();
}

uint8_t nfc_send_data(uint8_t * uid, nrf_nfc_id_size_t size, nrf_nfc_type_tag_t tag_type, uint8_t * data, uint16_t length)
{        
    if(length > NRF_NFC_NDEF_SIZE_MAX) {
        puts("[Error]: Data length over allowed max size");
        return NRF_NFC_ERROR;
    }
    
    clear_buffers();
    current_tag.tx_length = 0;
    current_tag.ndef_data = data;
    current_tag.mode = NRF_NFC_MODE_UID_TAG;
    current_tag.state = NRF_NFC_STATE_NONE;
    current_tag.ndef_size = length;
    
    nfc_create_cc_file();
  
    if(nfc_set_uid(uid, size, tag_type) == NRF_NFC_ERROR) {
        return NRF_NFC_ERROR;      
    }
  
        /* Enable NFC field detect interrupt*/   
    nrf_nfc_enable_int(NFCT_INTEN_SELECTED_Msk);
	
	return NRF_NFC_OK;
        
}

uint8_t nfc_set_uid(uint8_t * uid, nrf_nfc_id_size_t size, nrf_nfc_type_tag_t tag_type)
{        
    current_tag.mode = NRF_NFC_MODE_UID_TAG;
    current_tag.state = NRF_NFC_STATE_NONE;
        
    /*  Disable NFC peripheral */
    nrf_nfc_task(&NRF_NFCT->TASKS_DISABLE);    
    /* Disable Shortcut between FIELDDETECTED event and ACTIVATE task */
    NRF_NFCT->SHORTS = NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Disabled << NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Pos;
    /* Disable Shortcut between FIELDLOST event and SENSE task */
    NRF_NFCT->SHORTS |= NFCT_SHORTS_FIELDLOST_SENSE_Disabled << NFCT_SHORTS_FIELDLOST_SENSE_Pos;
    
    /* Set ID size and SDD (Single Device Detection) */
    NRF_NFCT->SENSRES = NFCT_SENSRES_BITFRAMESDD_SDD00001 | (size << NFCT_SENSRES_NFCIDSIZE_Pos);
    
    /* Set type tag */
    NRF_NFCT->SELRES = tag_type << NFCT_SELRES_PROTOCOL_Pos;
    
	if(size == NRF_NFC_UID_4_BYTES) {
		NRF_NFCT->NFCID1_LAST = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
	}
	else if(size == NRF_NFC_UID_7_BYTES) {
		NRF_NFCT->NFCID1_2ND_LAST =	(uid[0] << 16) | (uid[1] << 8) | uid[2];
		NRF_NFCT->NFCID1_LAST = (uid[3] << 24) | (uid[4] << 16) | (uid[5] << 8) | uid[6];
	}
	else if(size == NRF_NFC_UID_10_BYTES) {
		NRF_NFCT->NFCID1_3RD_LAST = (uid[0] << 16) | (uid[1] << 8) | uid[2];
		NRF_NFCT->NFCID1_2ND_LAST =	(uid[3] << 16) | (uid[4] << 8) | uid[5];
		NRF_NFCT->NFCID1_LAST = (uid[6] << 24) | (uid[7] << 16) | (uid[8] << 8) | uid[9];
	}
	else {
		return NRF_NFC_ERROR;
	}
    
    /* Enable Shortcut between FIELDDETECTED event and ACTIVATE task */
    NRF_NFCT->SHORTS = NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Enabled << NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Pos;
    /* Enable  Shortcut between FIELDLOST event and SENSE task */
    NRF_NFCT->SHORTS |= NFCT_SHORTS_FIELDLOST_SENSE_Enabled << NFCT_SHORTS_FIELDLOST_SENSE_Pos;
	/*  Enable NFC sense field mode, change state to sense mode */
    nrf_nfc_task(&NRF_NFCT->TASKS_SENSE);
	
	return NRF_NFC_OK;
}

void nfc_init(void)
{
    /*  Disable NFC peripheral */
	nrf_nfc_task(&NRF_NFCT->TASKS_DISABLE);
  
        /* Minimum frame delay */
    NRF_NFCT->FRAMEDELAYMIN = (NRF_NFC_FRAMEDELAYMIN) & (0x000FFFF);
        /* Maximum frame delay */
    NRF_NFCT->FRAMEDELAYMAX = (NRF_NFC_FRAMEDELAYMAX) & (0x000FFFF);
        /*  Frame is transmitted between FRAMEDELAYMIN and FRAMEDELAYMAX */
    NRF_NFCT->FRAMEDELAYMODE = NFCT_FRAMEDELAYMODE_FRAMEDELAYMODE_WindowGrid;
     
    /* Disable all NFC interrupts */
    nrf_nfc_disable_int(NRF_NFC_ALL_INTERRUPTS);
    
    /* Clear error status */ 
    nrf_nfc_clear_errors();
    /* Clear RX status */   
    nrf_nfc_clear_rx_status();    
    
    /* Enable NFC error interrupt*/
    nrf_nfc_enable_int(NFCT_INTEN_ERROR_Msk);
            
	/* Enable interrupts */
    NVIC_EnableIRQ(NFCT_IRQn); 
}