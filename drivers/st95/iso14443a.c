/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file		iso14443a.c
 * @brief       protocol ISO14443A for ST95
 * @author      Mikhail Perkov
 */
 
#include "st95.h"
#include "iso14443a.h"

extern uint8_t st95_txbuf[255];
extern uint8_t st95_rxbuf[255];

bool _check_bcc(uint8_t length, uint8_t * data, uint8_t bcc)
{
    uint8_t bcc_tmp = 0;

	for (uint8_t i = 0; i < length; i++) {
        bcc_tmp	^= data[i];
    }
	
	if((bcc_tmp ^ bcc) == 0) {
		return true;
    }
	else {
		return false;
    }
}

uint8_t _get_uidsize(uint8_t uid_byte)
{
	uint8_t size = 0;
	
	size = (uid_byte >> 6) & 0x03;
	return size;
}

static uint8_t _is_uid_complete(uint8_t sak_byte)
{
	if ((sak_byte & 0x04) == 0x04)
		return 0;
	else 
		return 1; 
}

void _14443a_reqa(const st95_t * dev)
{
	uint8_t data = 0x26;
    
	/* 1 byte data, Not used topaz format, not SplitFrame, Not aapend CRC, 7 significant bits in last byte */
	st95_send_receive(dev, &data, 1, 0, 0, 0, 7);
}

void _anticollision_1(const st95_t * dev)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL1, 0x20 };
       
	/* 2 byte data, Not used topaz format, not SplitFrame, Not aapend CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, 2, 0, 0, 0, 8);
}

void _anticollision_2(const st95_t * dev)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL2, 0x20 };
    
	/* 2 byte data, Not used topaz format, not SplitFrame, Not aapend CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, 2, 0, 0, 0, 8);
}

void _anticollision_3(const st95_t * dev)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL3, 0x20 };
    
	/* 2 byte data, Not used topaz format, not SplitFrame, Not aapend CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, 2, 0, 0, 0, 8);
}

void _select_1(const st95_t * dev, uint8_t num, uint8_t * uid_sel)
{
	uint8_t data[16] = { 0 };
    	
	data[0] = ISO14443A_SELECT_LVL1;
	data[1] = 0x70;
	memcpy(data + 2, uid_sel, num);
	
	/* 2 byte data, Not used topaz format, not SplitFrame, Append CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, num + 2, 0, 0, 1, 8);
}

void _select_2(const st95_t * dev, uint8_t num, uint8_t * uid_sel)
{
	uint8_t data[16] = { 0 };
	
	data[0] = ISO14443A_SELECT_LVL2;
	data[1] = 0x70;
	memcpy(data + 2, uid_sel, num);
	
	/* 2 byte data, Not used topaz format, not SplitFrame, Append CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, num + 2, 0, 0, 1, 8);
}

void _select_3(const st95_t * dev, uint8_t num, uint8_t * uid_sel)
{
	uint8_t data[16] = { 0 };
    	
	data[0] = ISO14443A_SELECT_LVL3;
	data[1] = 0x70;
	memcpy(data + 2, uid_sel, num);
	
	/* 2 byte data, Not used topaz format, not SplitFrame, Append CRC, 8 significant bits in last byte */
	st95_send_receive(dev, data, num + 2, 0, 0, 1, 8);
}

int get_uid_14443a(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{
    // uint8_t rxbuf[255] = { 0 };
    
    _14443a_reqa(dev);
      
    // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }
    
    // UIDsize : (2 bits) value: 0 for single, 1 for double,  2 for triple and 3 for RFU
    uint8_t uid_size = _get_uidsize(st95_rxbuf[2]);
    
    // === Select cascade level 1 ===
    _anticollision_1(dev);
        // _wait_ready_data();  
     // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }
    
    //  Check BCC
   if(!_check_bcc(4, st95_rxbuf + 2, st95_rxbuf[2 + 4])) {
       return ST95_ERROR;
   }
    // copy UID from CR95Hf response
    if (uid_size == ISO14443A_ATQA_SINGLE){
        memcpy(uid,&st95_rxbuf[2],ISO14443A_UID_SINGLE );
    }
    else {
        memcpy(uid,&st95_rxbuf[2 + 1],ISO14443A_UID_SINGLE - 1 );
    }
    _select_1(dev, 5, &st95_rxbuf[2]);   
        // _wait_ready_data();  
    // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }

    if(_is_uid_complete(st95_rxbuf[2]) == 1) {
        *sak = st95_rxbuf[2];
        *length_uid = ISO14443A_UID_SINGLE;
        return ST95_OK;
    }
    
         // === Select cascade level 2 ===
        _anticollision_2(dev);
            // _wait_ready_data();  
        // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }
 //  Check BCC
       if(!_check_bcc(4, st95_rxbuf + 2, st95_rxbuf[2 + 4])) {
           return ST95_ERROR;
       }

        // copy UID from CR95Hf response
        if (uid_size == ISO14443A_ATQA_DOUBLE)
            memcpy(&uid[ISO14443A_UID_SINGLE - 1], &st95_rxbuf[2], ISO14443A_UID_SINGLE );
        else 
            memcpy(&uid[ISO14443A_UID_SINGLE - 1], &st95_rxbuf[2], ISO14443A_UID_SINGLE - 1);
        
        //Send Select command	
        _select_2(dev, 5, &st95_rxbuf[2]);  
            // _wait_ready_data();  
        // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }

    if(_is_uid_complete(st95_rxbuf[2]) == 1) {
        *sak = st95_rxbuf[2];
        *length_uid = ISO14443A_UID_DOUBLE;
        return ST95_OK;
    }

    // === Select cascade level 2 ===
    _anticollision_3(dev);
        // _wait_ready_data();  
    // if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }                     
 //  Check BCC
   if(!_check_bcc(4, st95_rxbuf + 2, st95_rxbuf[2 + 4])) {
       return ST95_ERROR;
   }

    // copy UID from CR95Hf response
    if (uid_size == ISO14443A_ATQA_TRIPLE)
        memcpy(&uid[ISO14443A_UID_DOUBLE - 1], &st95_rxbuf[2], ISO14443A_UID_DOUBLE );
    
    //Send Select command	
    _select_3(dev, 5, &st95_rxbuf[2]);  
        // _wait_ready_data();  
// if(st95_receive(rxbuf) != ST95_OK) {
        // return ST95_ERROR;
    // }
    if(_is_uid_complete(st95_rxbuf[2]) == 1) {
        *sak = st95_rxbuf[2];
        *length_uid = ISO14443A_UID_TRIPLE;
        return ST95_OK;
    }
    
    return ST95_ERROR;
}