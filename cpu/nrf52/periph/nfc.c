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

#define NFCT_ALL_INTERRUPTS 	0x001D5CFF

#define NFCT_ALL_ERRORS 	0xDUL
#define NFCT_ALL_RX_STATUS 	0xDUL

#define MODE_UID_TAG 1
#define MODE_DATA_RXTX 0

#define NFC_CRC_SIZE 2
#define NFC_RX_BUFFER_SIZE          64      /**< NFC Rx data buffer size */

#define NFC_T4T_RATS_CMD 0xE0

#define NFC_FRAMEDELAYMAX 0x0000FFFF    /**< 4832 usec */
#define NFC_FRAMEDELAYMIN 0x00000486  /**< FDT = 85 usec */
// #define NFC_FRAMEDELAYMIN 0x000004DF  /**< FDT = 91 usec */

#define NFC_FWI 4
/**
 * @brief   */
typedef enum {
    RATS = 0x00,
    CC_LENGTH      = 0x01,
    CC_READ     = 0x02,
    NDEF_LENGTH      = 0x03,
    NDEF_READ      = 0x04,
} ndef_state_t;

static ndef_state_t state = RATS;

static volatile uint8_t rx_buffer[NFC_RX_BUFFER_SIZE] = { 0x00 };
static volatile uint8_t tx_buffer[NFC_RX_BUFFER_SIZE] = { 0x00 };
static volatile uint8_t tx_length = 0;
static uint16_t ndef_size = 0;
static uint8_t * ndef_data = NULL;


static volatile uint8_t nfc_mode_operation = MODE_UID_TAG;

static inline void nrf_nfc_enable_int(uint32_t interrupt);
static inline void nrf_nfc_disable_int(uint32_t interrupt);
static inline void nrf_nfc_clear_event(volatile uint32_t * event);
static inline void nrf_nfc_clear_errors(void);
static inline void nrf_nfc_clear_rx_status(void);
static inline void nrf_nfc_task(volatile uint32_t * task);

static void nfc_set_ats(uint8_t param_byte);
static void nfc_create_cc_file(uint8_t pcb);
static void nfc_length_cc_file(uint8_t pcb);

static void nfc_length_ndef_file(uint8_t pcb);
static void nfc_ndef_file(uint8_t pcb);

static void nfc_ndef_file(uint8_t pcb)
{
    tx_buffer[0] = pcb;
    
    for(uint32_t i = 0; i < ndef_size ; i++) {
        tx_buffer[i + 1] = *ndef_data;
        ndef_data++;
    }
    // tx_buffer[1] = 0xAA;
    // tx_buffer[2] = 0xBB;
    // tx_buffer[3] = 0xCC;
    // tx_buffer[4] = 0xDD;
    // tx_buffer[5] = 0xFF;
    // tx_buffer[6] = 0x01;
    // tx_buffer[7] = 0x02;
    // tx_buffer[8] = 0x03;
    // tx_buffer[9] = 0x04;
    // tx_buffer[10] = 0x05;
    
    tx_length = ndef_size + 1;
}

static void nfc_length_ndef_file(uint8_t pcb)
{
    tx_buffer[0] = pcb;
    tx_buffer[1] = ndef_size >> 8;
    tx_buffer[2] = ndef_size & 0xFF;                /**< 15 bytes */
    tx_length = 3;
}

static void nfc_create_cc_file(uint8_t pcb)
{
    if(rx_buffer[5] == 0x0F) {
        tx_buffer[0] = pcb;
        tx_buffer[1] = 0x00;
        tx_buffer[2] = 0x0F;                /**< 15 bytes */
        /* Mapping version */
        tx_buffer[3] = 0x20;
        /* Mle bytes (max data size that can be read) */
        tx_buffer[4] = 0x00;
        tx_buffer[5] = 0xF6;
        /* Mlc bytes (max data size that can be write)*/
        tx_buffer[6] = 0x00;
        tx_buffer[7] = 0xF6;
        /* TLV block */
        tx_buffer[8] = 0x04; /**< Tag field value: 0x04 */
        tx_buffer[9] = 0x06; /**< Tag length value: 0x06 bytes */
        tx_buffer[10] = 0x00; /**< NDEF ID (first byte) */
        tx_buffer[11] = 0x01; /**< NDEF ID (second byte) */
        
        tx_buffer[12] = ndef_size >> 8;     /**< Max NDEF file size (first byte) */
        tx_buffer[13] = ndef_size & 0xFF;   /**< Max NDEF file size (second byte) */
        
        tx_buffer[14] = 0x00; /**< NDEF file read access condition */
        tx_buffer[15] = 0x00; /**< NDEF file write access condition */                
        tx_length = 16;   
    }
    else {
        tx_length = 0;   
    }
}

static void nfc_length_cc_file(uint8_t pcb)
{
    tx_buffer[0] = pcb;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x0F;                /**< 15 bytes */
    tx_length = 3;
}

static void nfc_set_ats(uint8_t param_byte)
{
    #define IS_TA1 1
    #define IS_TB1 1
    #define IS_TC1 1
    #define IS_NAD 0
    #define IS_CID 1
    #define SFGI   0
    
    uint8_t tl = 0; // length
    uint8_t t0 = 0; // format byte
    uint8_t ta1 = 0; // iface byte 1
    uint8_t tb1 = 0; // iface byte 2
    uint8_t tc1 = 0; // iface byte 3
    uint8_t fsci = (param_byte >> 4) & 0x0F; // Frame Size for proximity Card Integer

    tl = 5;
    t0 = ( (IS_TC1 << 6) | (IS_TB1 << 5) | (IS_TA1 << 4) | (fsci << 0)) & 0x7F;
    
    if(IS_TA1) {
        ta1 = 0x80; // 0x80 -> (1 000 0 000)b
    }
    if(IS_TB1) {
        tb1 = (NFC_FWI << 4) | (SFGI << 0);
    }
    if(IS_TC1) {
        tc1 = ((IS_CID << 1) | (IS_NAD << 0)) & 0x03;
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
    if(nfc_mode_operation == MODE_DATA_RXTX) {  
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
    
    NRF_NFCT->ERRORSTATUS = NFCT_ALL_ERRORS;
}

static inline void nrf_nfc_clear_rx_status(void) 
{   
    if(nfc_mode_operation == MODE_DATA_RXTX) {  
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
    
    NRF_NFCT->FRAMESTATUS.RX = NFCT_ALL_RX_STATUS;                       
}

static inline void nrf_nfc_task(volatile uint32_t * task)
{
    *task = 0x1UL;
}

void isr_nfct(void)
{
    if(NRF_NFCT->EVENTS_FIELDDETECTED && (NRF_NFCT->INTEN & NFCT_INTEN_FIELDDETECTED_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDDETECTED);
        puts("\t\tEVENTS_FIELDDETECTED");
    }
    
    if(NRF_NFCT->EVENTS_FIELDLOST && (NRF_NFCT->INTEN & NFCT_INTEN_FIELDLOST_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDLOST);
        puts("\t\tEVENTS_FIELDLOST\t>>>>>>>>>>>>>>");
    }
    
    if(NRF_NFCT->EVENTS_RXFRAMEEND && (NRF_NFCT->INTEN & NFCT_INTEN_RXFRAMEEND_Msk)) {       
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXFRAMEEND);
        // puts("EVENTS_RXFRAMEEND"); 
        
        /* Take into account only number of whole bytes */
        // uint32_t rx_data_size = ((NRF_NFCT->RXD.AMOUNT & NFCT_RXD_AMOUNT_RXDATABYTES_Msk) >> NFCT_RXD_AMOUNT_RXDATABYTES_Pos) - NFC_CRC_SIZE;
        // printf("\tRX size: %ld -> ", rx_data_size);
        
        if (rx_buffer[0] == NFC_T4T_RATS_CMD) {            
            nfc_set_ats(rx_buffer[1]);
            tx_length = 5;
            NRF_NFCT->PACKETPTR          = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN             = NFC_RX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;

            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);
            nrf_nfc_enable_int(NFCT_INTENSET_TXFRAMEEND_Msk);
            state = CC_LENGTH;
            puts("\t[RATS]");
        }
        else  if ((rx_buffer[0] == 0x02) || (rx_buffer[0] == 0x03)) {       
            tx_buffer[0] = rx_buffer[0];
            tx_length = 1;
            if(rx_buffer[2] == 0xB0) {
                /* CC file */           
                /* CC file length */
                if(state == CC_LENGTH) {
                    nfc_length_cc_file(rx_buffer[0]); 
                    state = CC_READ;
                }                
                else if(state == CC_READ) {
                    nfc_create_cc_file(rx_buffer[0]);
                    state = NDEF_LENGTH;
                }
                else if(state == NDEF_LENGTH) {
                    nfc_length_ndef_file(rx_buffer[0]);
                    state = NDEF_READ;
                }
                else if(state == NDEF_READ) {
                    nfc_ndef_file(rx_buffer[0]);                    
                }
                
            }
            
            tx_buffer[tx_length] = 0x90;
            tx_length++;
            tx_buffer[tx_length] = 0x00;
            tx_length++;

            NRF_NFCT->PACKETPTR          = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN             = NFC_RX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;
            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);
            
            nrf_nfc_enable_int(NFCT_INTENSET_TXFRAMEEND_Msk);
            printf("\tIBLOCK %02X\n", rx_buffer[0]);            
        }
        else  if (rx_buffer[0] == 0x50) {
            nrf_nfc_disable_int(NFCT_INTENCLR_RXFRAMEEND_Msk);
            nrf_nfc_disable_int(NFCT_INTENCLR_RXERROR_Msk);
            puts("\t\t[HALT]");
        }
        else  if (rx_buffer[0] == 0xC2) {
            tx_buffer[0] = 0xC2;
            tx_length = 1;
             
            NRF_NFCT->PACKETPTR          = (uint32_t) tx_buffer;
            NRF_NFCT->MAXLEN             = NFC_RX_BUFFER_SIZE;
            NRF_NFCT->TXD.AMOUNT = (tx_length << NFCT_TXD_AMOUNT_TXDATABYTES_Pos) & NFCT_TXD_AMOUNT_TXDATABYTES_Msk;
            
            nrf_nfc_task(&NRF_NFCT->TASKS_STARTTX);           
            nrf_nfc_enable_int(NFCT_INTENSET_TXFRAMEEND_Msk);
            
             // nrf_nfc_disable_int(NFCT_INTENCLR_RXFRAMEEND_Msk);  
             // nrf_nfc_disable_int(NFCT_INTENCLR_RXERROR_Msk);
             
            puts("\t\t[DESELECT]");
        }
        else {
            puts("\tOTHER");
        }
        // nrf_nfc_disable_int(NFCT_INTENCLR_RXFRAMEEND_Msk);
    }
    
    if(NRF_NFCT->EVENTS_TXFRAMEEND && (NRF_NFCT->INTEN & NFCT_INTEN_TXFRAMEEND_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMEEND);
        nrf_nfc_disable_int(NFCT_INTENCLR_TXFRAMEEND_Msk);
        
        nrf_nfc_clear_errors();
        nrf_nfc_clear_rx_status();
        clear_buffers();
        // /* Set up registers for EasyDMA and start receiving packets */
        NRF_NFCT->PACKETPTR          = (uint32_t) rx_buffer;
        NRF_NFCT->MAXLEN             = NFC_RX_BUFFER_SIZE;
        nrf_nfc_task(&NRF_NFCT->TASKS_ENABLERXDATA);
        nrf_nfc_enable_int(NFCT_INTENSET_RXFRAMEEND_Msk);    
        // puts("EVENTS_TXFRAMEEND");
    }
    
    if(NRF_NFCT->EVENTS_RXERROR && (NRF_NFCT->INTEN & NFCT_INTEN_RXERROR_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXERROR);
        puts("EVENTS_RX_ERROR");  
        nrf_nfc_clear_rx_status();
    }
    
    if(NRF_NFCT->EVENTS_ERROR && (NRF_NFCT->INTEN & NFCT_INTEN_ERROR_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_ERROR);
        puts("EVENTS_ERROR");
        nrf_nfc_clear_errors();
    }
    
    if(NRF_NFCT->EVENTS_SELECTED && (NRF_NFCT->INTEN & NFCT_INTEN_SELECTED_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_SELECTED);
        nfc_mode_operation = MODE_DATA_RXTX;
        // nrf_nfc_disable_int(NFCT_INTENCLR_TXFRAMEEND_Msk);
        // nrf_nfc_disable_int(NFCT_INTENCLR_RXFRAMEEND_Msk);
        state = RATS;
        nrf_nfc_clear_errors();
        nrf_nfc_clear_rx_status();
        // clear_buffers();
        
        /* Set up registers for EasyDMA and start receiving packets */
        NRF_NFCT->PACKETPTR          = (uint32_t) rx_buffer;
        NRF_NFCT->MAXLEN             = 15;
        nrf_nfc_enable_int(NFCT_INTENSET_RXFRAMEEND_Msk);
        nrf_nfc_task(&NRF_NFCT->TASKS_ENABLERXDATA);
        
        nrf_nfc_enable_int(NFCT_INTENSET_RXERROR_Msk);
        // puts("\n\n\t>>> EVENTS_SELECTED <<<");
    }      
    
    if(NRF_NFCT->EVENTS_TXFRAMESTART && (NRF_NFCT->INTEN & NFCT_INTEN_TXFRAMESTART_Msk)) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMESTART);
        puts("EVENTS_TXFRAMESTART");
    }

     // puts(">>> [IRQ END] <<<\n");
    
    cortexm_isr_end();
}

uint8_t nfc_send_data(uint8_t * uid, nfc_id_size_t size, nfc_type_tag_t tag_type, uint8_t * data, uint8_t length)
{        
    ndef_data = data;
    nfc_mode_operation = MODE_UID_TAG;
    clear_buffers();
    state = RATS;
    ndef_size = length;
        
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
    
	if(size == NFC_UID_4_BYTES) {								
		NRF_NFCT->NFCID1_LAST = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];	
	}
	else if(size == NFC_UID_7_BYTES) {								
		NRF_NFCT->NFCID1_2ND_LAST =	(uid[0] << 16) | (uid[1] << 8) | uid[2];		
		NRF_NFCT->NFCID1_LAST = (uid[3] << 24) | (uid[4] << 16) | (uid[5] << 8) | uid[6];
	}
	else if(size == NFC_UID_10_BYTES) {							
		NRF_NFCT->NFCID1_3RD_LAST = (uid[0] << 16) | (uid[1] << 8) | uid[2];
		NRF_NFCT->NFCID1_2ND_LAST =	(uid[3] << 16) | (uid[4] << 8) | uid[5];		
		NRF_NFCT->NFCID1_LAST = (uid[6] << 24) | (uid[7] << 16) | (uid[8] << 8) | uid[9];
	}
	else {
		return NRF_NFC_ERROR;
	}

        /* Enable NFC field detect interrupt*/   
    nrf_nfc_enable_int(NFCT_INTENSET_SELECTED_Msk);
    
    /* Enable Shortcut between FIELDDETECTED event and ACTIVATE task */
    NRF_NFCT->SHORTS = NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Enabled << NFCT_SHORTS_FIELDDETECTED_ACTIVATE_Pos;
    /* Enable  Shortcut between FIELDLOST event and SENSE task */
    NRF_NFCT->SHORTS |= NFCT_SHORTS_FIELDLOST_SENSE_Enabled << NFCT_SHORTS_FIELDLOST_SENSE_Pos;
	/*  Enable NFC sense field mode, change state to sense mode */
    nrf_nfc_task(&NRF_NFCT->TASKS_SENSE);
	
	return NRF_NFC_OK;
}

uint8_t nfc_set_uid(uint8_t * uid, nfc_id_size_t size, nfc_type_tag_t tag_type)
{        
    nfc_mode_operation = MODE_UID_TAG;
        
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
    
	if(size == NFC_UID_4_BYTES) {								
		NRF_NFCT->NFCID1_LAST = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];	
	}
	else if(size == NFC_UID_7_BYTES) {								
		NRF_NFCT->NFCID1_2ND_LAST =	(uid[0] << 16) | (uid[1] << 8) | uid[2];		
		NRF_NFCT->NFCID1_LAST = (uid[3] << 24) | (uid[4] << 16) | (uid[5] << 8) | uid[6];
	}
	else if(size == NFC_UID_10_BYTES) {							
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
     /* Checking setting of pins dedicated to NFC functionality */
    if((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != UICR_NFCPINS_PROTECT_NFC) {
         /* Setting of pins dedicated to NFC functionality */
        NRF_UICR->NFCPINS |= (0x1UL << UICR_NFCPINS_PROTECT_Pos);
    }
    
    /*  Disable NFC peripheral */
	nrf_nfc_task(&NRF_NFCT->TASKS_DISABLE);
  
        /* Minimum frame delay */
    NRF_NFCT->FRAMEDELAYMIN = (NFC_FRAMEDELAYMIN) & (0x000FFFF);
        /* Maximum frame delay */
    NRF_NFCT->FRAMEDELAYMAX = (NFC_FRAMEDELAYMAX) & (0x000FFFF);
        /*  Frame is transmitted between FRAMEDELAYMIN and FRAMEDELAYMAX */
    NRF_NFCT->FRAMEDELAYMODE = NFCT_FRAMEDELAYMODE_FRAMEDELAYMODE_WindowGrid;
    // NRF_NFCT->FRAMEDELAYMODE = NFCT_FRAMEDELAYMODE_FRAMEDELAYMODE_ExactVal;
    uint32_t delay_max = (uint32_t)(((NRF_NFCT->FRAMEDELAYMAX) * 1000) / 13560);
    uint32_t delay_min = (uint32_t)(((NRF_NFCT->FRAMEDELAYMIN) * 1000) / 13560);
    printf("\t[FRAMEDELAYMIN]: 0x%08lX -> %ld [usec]\n", NRF_NFCT->FRAMEDELAYMIN, delay_min);
    printf("\t[FRAMEDELAYMAX]: 0x%08lX -> %ld [usec]\n", NRF_NFCT->FRAMEDELAYMAX, delay_max);
    
    
    uint8_t pp = NFC_FWI; 
    uint8_t mm = 1; 
    uint8_t dd = 0;
    uint32_t fwt_st = (uint32_t)((1 << pp)*(mm+1)*(dd + 128)*32*100/1356);
    printf("FWT st95: %ld [usec]\n", fwt_st);
    
    uint32_t fwt_nrf = (uint32_t)((409600/1356) * (1 << NFC_FWI));
    printf("FWT nRF: %ld [usec]\n", fwt_nrf);
    
    
    /* Disable all NFC interrupts */
    nrf_nfc_disable_int(NFCT_ALL_INTERRUPTS);
    
    /* Clear error status */ 
    nrf_nfc_clear_errors();
    /* Clear RX status */   
    nrf_nfc_clear_rx_status();    
    
    /* Enable NFC error interrupt*/
    nrf_nfc_enable_int(NFCT_INTENSET_ERROR_Msk);
            
	/* Enable interrupts */
    NVIC_EnableIRQ(NFCT_IRQn);
    
    
    /* TODO:  */
    // nrf_nfc_task(&NRF_NFCT->TASKS_GOIDLE);
    // nrf_nfc_task(&NRF_NFCT->TASKS_GOSLEEP);    
}