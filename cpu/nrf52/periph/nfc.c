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
#define NFCT_ENABLE_ALL_INT 	NFCT_ALL_INTERRUPTS
#define NFCT_DISABLE_ALL_INT 	NFCT_ALL_INTERRUPTS

#define NFCT_ALL_ERRORS 	0xDUL
#define NFCT_ALL_RX_STATUS 	0xDUL

#define MODE_UID_TAG 1
#define MODE_DATA_RXTX 0


static uint8_t nfc_mode_operation = MODE_UID_TAG;

static inline void nrf_nfc_enable_int(uint32_t interrupt);
static inline void nrf_nfc_disable_int(uint32_t interrupt);
static inline void nrf_nfc_clear_event(volatile uint32_t * event);
static inline void nrf_nfc_clear_errors(void);
static inline void nrf_nfc_clear_rx_status(void);
static inline bool nrf_nfc_get_event(volatile uint32_t * event);
static inline void nrf_nfc_task(volatile uint32_t * task);

static inline void nrf_nfc_enable_int(uint32_t interrupt)
{   
    if(interrupt == NFCT_ENABLE_ALL_INT) {
        NRF_NFCT->INTENSET =    //(NFCT_INTENSET_READY_Set << NFCT_INTENSET_READY_Pos) |
                                // (NFCT_INTENSET_FIELDDETECTED_Set << NFCT_INTENSET_FIELDDETECTED_Pos) |
                                // (NFCT_INTENSET_FIELDLOST_Set << NFCT_INTENSET_FIELDLOST_Pos);// |
                                // (NFCT_INTENSET_TXFRAMESTART_Set << NFCT_INTENSET_TXFRAMESTART_Pos) |
                                // (NFCT_INTENSET_TXFRAMEEND_Set << NFCT_INTENSET_TXFRAMEEND_Pos) |
                                // (NFCT_INTENSET_RXFRAMESTART_Set << NFCT_INTENSET_RXFRAMESTART_Pos) |
                                // (NFCT_INTENSET_RXFRAMEEND_Set << NFCT_INTENSET_RXFRAMEEND_Pos) |
                                (NFCT_INTENSET_ERROR_Set << NFCT_INTENSET_ERROR_Pos) |
                                (NFCT_INTENSET_RXERROR_Set << NFCT_INTENSET_RXERROR_Pos);// |
                                // (NFCT_INTENSET_ENDRX_Set << NFCT_INTENSET_ENDRX_Pos) |
                                // (NFCT_INTENSET_ENDTX_Set << NFCT_INTENSET_ENDTX_Pos) |
                                // (NFCT_INTENSET_AUTOCOLRESSTARTED_Set << NFCT_INTENSET_AUTOCOLRESSTARTED_Pos) |
                                // (NFCT_INTENSET_COLLISION_Set << NFCT_INTENSET_COLLISION_Pos) |
                                // (NFCT_INTENSET_SELECTED_Set << NFCT_INTENSET_SELECTED_Pos) |
                                // (NFCT_INTENSET_STARTED_Set << NFCT_INTENSET_STARTED_Pos);
    }
    else {
        NRF_NFCT->INTENSET = interrupt;
    }
}

static inline void nrf_nfc_disable_int(uint32_t interrupt)
{   
    if(interrupt == NFCT_ENABLE_ALL_INT) {
        NRF_NFCT->INTENCLR =    //(NFCT_INTENCLR_READY_Clear << NFCT_INTENCLR_READY_Pos) |
                                (NFCT_INTENCLR_FIELDDETECTED_Clear << NFCT_INTENCLR_FIELDDETECTED_Pos) |
                                (NFCT_INTENCLR_FIELDLOST_Clear << NFCT_INTENCLR_FIELDLOST_Pos);// |
                                // (NFCT_INTENCLR_TXFRAMESTART_Clear << NFCT_INTENCLR_TXFRAMESTART_Pos) |
                                // (NFCT_INTENCLR_TXFRAMEEND_Clear << NFCT_INTENCLR_TXFRAMEEND_Pos) |
                                // (NFCT_INTENCLR_RXFRAMESTART_Clear<< NFCT_INTENCLR_RXFRAMESTART_Pos) |
                                // (NFCT_INTENCLR_RXFRAMEEND_Clear << NFCT_INTENCLR_RXFRAMEEND_Pos) |
                                // (NFCT_INTENCLR_ERROR_Clear << NFCT_INTENCLR_ERROR_Pos);// |
                                // (NFCT_INTENCLR_RXERROR_Clear << NFCT_INTENCLR_RXERROR_Pos) |
                                // (NFCT_INTENCLR_ENDRX_Clear << NFCT_INTENCLR_ENDRX_Pos) |
                                // (NFCT_INTENCLR_ENDTX_Clear << NFCT_INTENCLR_ENDTX_Pos) |
                                // (NFCT_INTENCLR_AUTOCOLRESSTARTED_Clear << NFCT_INTENCLR_AUTOCOLRESSTARTED_Pos) |
                                // (NFCT_INTENCLR_COLLISION_Clear << NFCT_INTENCLR_COLLISION_Pos) |
                                // (NFCT_INTENCLR_SELECTED_Clear << NFCT_INTENCLR_SELECTED_Pos) |
                                // (NFCT_INTENCLR_STARTED_Clear << NFCT_INTENCLR_STARTED_Pos);
    }
    else {
        NRF_NFCT->INTENCLR = interrupt;
    }
}

static inline void nrf_nfc_clear_event(volatile uint32_t * event)
{   
    *event = 0x0UL;
    __DSB();
}

static inline bool nrf_nfc_get_event(volatile uint32_t * event)
{
    return (bool)(* event);
}

static inline void nrf_nfc_clear_errors(void) 
{  
    if(nfc_mode_operation == MODE_DATA_RXTX) {  
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_NFCFIELDTOOWEAK_Msk) {
            puts("[ERROR]: Field level is too LOW at MIN load resistance");
        }
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_NFCFIELDTOOSTRONG_Msk) {
            puts("[ERROR]: Field level is too HIGH at MAX load resistance");
        }
        if(NRF_NFCT->ERRORSTATUS & NFCT_ERRORSTATUS_FRAMEDELAYTIMEOUT_Msk) {
            puts("[ERROR]: No STARTTX task triggered before expiration of the time set in FRAMEDELAYMAX");
        }
    }
    
    NRF_NFCT->ERRORSTATUS = NFCT_ALL_ERRORS;
}

static inline void nrf_nfc_clear_rx_status(void) 
{   
    if(nfc_mode_operation == MODE_DATA_RXTX) {  
        if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_OVERRUN_Msk) {
            puts("[RX STATUS]: Overrun");
        }
    }
    if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_PARITYSTATUS_Msk) {
        puts("[RX STATUS]: Parity Error");
    }
    if(NRF_NFCT->FRAMESTATUS.RX & NFCT_FRAMESTATUS_RX_CRCERROR_Msk) {
        puts("[RX STATUS]: CRC Error");
    }
    
    NRF_NFCT->FRAMESTATUS.RX = NFCT_ALL_RX_STATUS;                       
}

static inline void nrf_nfc_task(volatile uint32_t * task)
{
    *task = 0x1UL;
}

void isr_nfct(void)
{
    // if(NRF_NFCT->EVENTS_FIELDDETECTED) {
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDDETECTED);
        // puts("\t\tEVENTS_FIELDDETECTED");
    // }
    
    // if(NRF_NFCT->EVENTS_READY) {
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_READY);
         // puts("EVENTS_READY");       
    // }

    // if(NRF_NFCT->EVENTS_AUTOCOLRESSTARTED) {
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_AUTOCOLRESSTARTED);
        // puts("EVENTS_AUTOCOLRESSTARTED"); 
    // }
    // if(NRF_NFCT->EVENTS_COLLISION) {       
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_COLLISION);
        // puts("EVENTS_COLLISION"); 
    // }
    // if(NRF_NFCT->EVENTS_SELECTED) {
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_SELECTED);
        // nrf_nfc_clear_errors();
        // puts("EVENTS_SELECTED");
    // }
    
    // if(NRF_NFCT->EVENTS_FIELDLOST) {
        // nrf_nfc_clear_event(&NRF_NFCT->EVENTS_FIELDLOST);
        // puts("\t\tEVENTS_FIELDLOST\t\t>>>>>>>\t>>>>>>>");
    // }
    
    if(NRF_NFCT->EVENTS_ERROR) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_ERROR);
        puts("EVENTS_ERROR");
        nrf_nfc_clear_errors();
        // nrf_nfc_task(&NRF_NFCT->TASKS_SENSE);
    }
    if(NRF_NFCT->EVENTS_RXERROR) {
        nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXERROR);
        puts("EVENTS_RXERROR");  
        nrf_nfc_clear_rx_status();
        // nrf_nfc_task(&NRF_NFCT->TASKS_SENSE);
    }
        
    if(nfc_mode_operation == MODE_DATA_RXTX) {  
    
        if(NRF_NFCT->EVENTS_TXFRAMESTART) {
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMESTART);
            puts("EVENTS_TXFRAMESTART");
        }
        if(NRF_NFCT->EVENTS_TXFRAMEEND) {
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_TXFRAMEEND);
            puts("EVENTS_TXFRAMEEND");
        }
        if(NRF_NFCT->EVENTS_RXFRAMESTART) {
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXFRAMESTART);
            puts("EVENTS_RXFRAMESTART");
        }
        if(NRF_NFCT->EVENTS_RXFRAMEEND) {       
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_RXFRAMEEND);
            puts("EVENTS_RXFRAMEEND"); 
        }
        
        if(NRF_NFCT->EVENTS_ENDRX) {        
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_ENDRX);
            puts("EVENTS_ENDRX");
        }
        if(NRF_NFCT->EVENTS_ENDTX) {      
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_ENDTX);    
            puts("EVENTS_ENDTX");
        }
        if(NRF_NFCT->EVENTS_STARTED) {       
            nrf_nfc_clear_event(&NRF_NFCT->EVENTS_STARTED);
            puts("EVENTS_STARTED"); 
        }
    }

     // puts(">>> [IRQ END] <<<\n");
    
    cortexm_isr_end();
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
    // NRF_NFCT->SENSRES = NFCT_SENSRES_BITFRAMESDD_SDD00001 | (0x0UL << NFCT_SENSRES_RFU5_Pos) |
                        // (size << NFCT_SENSRES_NFCIDSIZE_Pos) | (0x0UL << NFCT_SENSRES_RFU74_Pos);
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
  
    /* TODO: Frame delay mode */
        /* Minimum frame delay */
    // NRF_NFCT->FRAMEDELAYMIN = 0x00000001;
        /* Maximum frame delay */
    NRF_NFCT->FRAMEDELAYMAX = 0x00007FFF;
        /*  Frame is transmitted between FRAMEDELAYMIN and FRAMEDELAYMAX */
    NRF_NFCT->FRAMEDELAYMODE = NFCT_FRAMEDELAYMODE_FRAMEDELAYMODE_Window;

    /* Disable all NFC interrupts */
    nrf_nfc_disable_int(NFCT_ALL_INTERRUPTS);
    
    /* Clear error status */ 
    nrf_nfc_clear_errors();
    /* Clear RX status */   
    nrf_nfc_clear_rx_status();    
    
    /* Enable NFCT interrupts */    
    /* Enable NFC error interrupt*/
    nrf_nfc_enable_int(NFCT_INTENSET_ERROR_Msk);
    /* Enable NFC RX frame error interrupt*/
    nrf_nfc_enable_int(NFCT_INTENSET_RXERROR_Msk);
        
	/* Enable interrupts */
    NVIC_EnableIRQ(NFCT_IRQn);
    
	/*  Enable NFC sense field mode, change state to sense mode */
    // nrf_nfc_task(&NRF_NFCT->TASKS_SENSE);
    
    
    /* TODO:  */
    // nrf_nfc_task(&NRF_NFCT->TASKS_GOIDLE);
    // nrf_nfc_task(&NRF_NFCT->TASKS_GOSLEEP);    
}