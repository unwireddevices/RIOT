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

static uint8_t iso_rxbuf[255];

static bool _iso14443a_check_bcc(uint8_t length, uint8_t * data, uint8_t bcc)
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

static uint8_t _iso14443a_get_uidsize(uint8_t uid_byte)
{
	uint8_t size = 0;
	
	size = (uid_byte >> ISO14443A_OFFSET_UID_SIZE) & ISO14443A_MASK_UID_SIZE;
	return size;
}

static uint8_t _iso14443a_is_uid_complete(uint8_t sak_byte)
{
	if ((sak_byte & ISO14443A_MASK_SAK_UID_NOT_COMPLETE) == ISO14443A_SAK_UID_NOT_COMPLETE)
		return ST95_ERROR;
	else 
		return ST95_OK; 
}

static uint8_t _iso14443a_reqa(const st95_t * dev, uint8_t * rxbuff)
{
	uint8_t data = ISO14443A_CMD_REQA;
    
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 7 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_7;
    
    if(_st95_cmd_send_receive(dev, &data, sizeof(data), ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;
}

static uint8_t _iso14443a_anticollision_1(const st95_t * dev, uint8_t * rxbuff)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL1, ISO14443A_NVB_20 };
       
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8;
    
    if(_st95_cmd_send_receive(dev, data, sizeof(data), ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;
    
}

static uint8_t _iso14443a_anticollision_2(const st95_t * dev, uint8_t * rxbuff)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL2, ISO14443A_NVB_20 };
    
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8;
    
    if(_st95_cmd_send_receive(dev, data, sizeof(data), ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;
}

static uint8_t _iso14443a_anticollision_3(const st95_t * dev, uint8_t * rxbuff)
{
	uint8_t data[2] = { ISO14443A_SELECT_LVL3, ISO14443A_NVB_20 };
    
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8;
    
   if(_st95_cmd_send_receive(dev, data, sizeof(data), ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;
}

static uint8_t _iso14443a_select_1(const st95_t * dev, uint8_t num, uint8_t * uid_sel, uint8_t * rxbuff)
{
	uint8_t data[ISO14443A_CMD_MAX_BYTE] = { 0 };
    	
	data[0] = ISO14443A_SELECT_LVL1;
	data[1] = ISO14443A_NVB_70;
	memcpy(data + 2, uid_sel, num);
    
    /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    if(_st95_cmd_send_receive(dev, data, num + 2, ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;   
}

static uint8_t _iso14443a_select_2(const st95_t * dev, uint8_t num, uint8_t * uid_sel, uint8_t * rxbuff)
{
	uint8_t data[ISO14443A_CMD_MAX_BYTE] = { 0 };
	
	data[0] = ISO14443A_SELECT_LVL2;
	data[1] = ISO14443A_NVB_70;
	memcpy(data + 2, uid_sel, num);
	
	/* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
   if(_st95_cmd_send_receive(dev, data, num + 2, ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;   
}

static uint8_t _iso14443a_select_3(const st95_t * dev, uint8_t num, uint8_t * uid_sel, uint8_t * rxbuff)
{
	uint8_t data[ISO14443A_CMD_MAX_BYTE] = { 0 };
    	
	data[0] = ISO14443A_SELECT_LVL3;
	data[1] = ISO14443A_NVB_70;
	memcpy(data + 2, uid_sel, num);
	
	/* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    if(_st95_cmd_send_receive(dev, data, num + 2, ctrl_byte, rxbuff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;   
}

int iso14443a_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{  
    if(_iso14443a_reqa(dev, iso_rxbuf) != ST95_OK)
        return ST95_ERROR;
    
    // UIDsize : (2 bits) value: 0 for single, 1 for double,  2 for triple and 3 for RFU
    uint8_t uid_size = _iso14443a_get_uidsize(iso_rxbuf[ISO14443A_OFFSET_ATQA_FIRST_BYTE]);
    
    // === Select cascade level 1 ===
    if(_iso14443a_anticollision_1(dev, iso_rxbuf) != ST95_OK)
        return ST95_ERROR;
    
    //  Check BCC
   if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
       return ST95_ERROR;
   }
    // copy UID from CR95Hf response
    if (uid_size == ISO14443A_ATQA_SINGLE){
        memcpy(uid, &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE );
    }
    else {
        memcpy(uid, &iso_rxbuf[ST95_DATA_OFFSET + 1], ISO14443A_UID_SINGLE - 1 );
    }
    if(_iso14443a_select_1(dev, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf) != ST95_OK)
        return ST95_ERROR;

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        *sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        *length_uid = ISO14443A_UID_SINGLE;
        return ST95_OK;
    }
    
         // === Select cascade level 2 ===
        if(_iso14443a_anticollision_2(dev, iso_rxbuf) != ST95_OK)
            return ST95_ERROR;

 //  Check BCC
       if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
           return ST95_ERROR;
       }

        // copy UID from CR95Hf response
        if (uid_size == ISO14443A_ATQA_DOUBLE)
            memcpy(&uid[ISO14443A_UID_SINGLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE );
        else 
            memcpy(&uid[ISO14443A_UID_SINGLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE - 1);
        
        //Send Select command	
        if(_iso14443a_select_2(dev, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf) != ST95_OK)
            return ST95_ERROR;

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        *sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        *length_uid = ISO14443A_UID_DOUBLE;
        return ST95_OK;
    }

    // === Select cascade level 2 ===
    if(_iso14443a_anticollision_3(dev, iso_rxbuf) != ST95_OK)
        return ST95_ERROR;
                    
 //  Check BCC
   if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
       return ST95_ERROR;
   }

    // copy UID from CR95Hf response
    if (uid_size == ISO14443A_ATQA_TRIPLE)
        memcpy(&uid[ISO14443A_UID_DOUBLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_DOUBLE );
    
    //Send Select command	
    if(_iso14443a_select_3(dev, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf) != ST95_OK)
        return ST95_ERROR;

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        *sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        *length_uid = ISO14443A_UID_TRIPLE;
        return ST95_OK;
    }
    
    return ST95_ERROR;
}