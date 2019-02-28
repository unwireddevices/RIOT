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
#define ENABLE_DEBUG (1)
#include "debug.h"

#define ENABLE_DEBUG_ST95 (0)

#if ENABLE_DEBUG_ST95
    // #define PRINTBUFF _printbuff
    // static void _printbuff(uint8_t *buff, unsigned len)
    // {
        // for(uint32_t i = 0; i < len; i++){
            // printf("%02X ", buff[i]);
            // if(i < 2) printf("  ");
        // }
        // printf("\n");
    // }
    
    #define PRINTSTR _printstr
    static void _printstr(char * str)
    {
        printf("%s ", str);
    }
#else
    // #define PRINTBUFF(...)
    #define PRINTSTR(...)
#endif

extern uint8_t cmd_allow;

static uint8_t iblock = ISO7816_IBLOCK_02;

static int _iso14443a_anticoll_algorithm(const st95_t * dev, uint8_t * iso_rxbuf);
uint8_t _iso14443a_get_ats(const st95_t * dev, uint8_t * rxbuff);

static iso14443a_picc_t picc;

static int _is_iblock (uint8_t * rxbuff) {
    if ((rxbuff[NDEF_OFFSET_PCB] & NDEF_MASK_BLOCK) == NDEF_MASK_IBLOCK) {
        return ST95_OK;
    } else {
        return ST95_ERROR;
    }
}

static int _is_rblock (uint8_t * rxbuff) {
    if ((rxbuff[NDEF_OFFSET_PCB] & NDEF_MASK_BLOCK) == NDEF_MASK_RBLOCK) {
        return ST95_OK;
    } else {
        return ST95_ERROR;
    }
}

static int _is_sblock (uint8_t * rxbuff) {
    if ((rxbuff[NDEF_OFFSET_PCB] & NDEF_MASK_BLOCK) == NDEF_MASK_SBLOCK) {
        return ST95_OK;
    } else {
        return ST95_ERROR;
    }
}

static uint8_t _check_ndef(const st95_t * dev, uint8_t * rxbuff)
{
    uint8_t length = rxbuff[ST95_LENGTH_OFFSET];
    
    if(length < 3) {
        DEBUG("Error: invalid RX length\n");
        return ST95_ERROR;
    }
        
    if(_is_iblock(rxbuff) == ST95_OK) { 
        uint8_t sw1 = rxbuff[length - ISO14443A_CRC16_LENGTH - ISO14443A_SERVICE_DATA_LENGTH];
        uint8_t sw2 = rxbuff[length - ISO14443A_CRC16_LENGTH - ISO14443A_SERVICE_DATA_LENGTH + 1];
        
        if((sw1 == NDEF_OK_SW_1) && (sw2 == NDEF_OK_SW_2)) {
            /* Toggle iblock*/
            iblock ^= 0x01;
            return ST95_OK;
        }
        DEBUG("Error status: %02X%02X\n", sw1, sw2);
        return ST95_ERROR;
    }
    else if(_is_rblock(rxbuff) == ST95_OK) { 
        // TODO
        return ST95_OK;
    }
    else if(_is_sblock(rxbuff) == ST95_OK) { 
        uint8_t s_buff[2] = { 0x00 };
        s_buff[0] = rxbuff[NDEF_OFFSET_PCB];
        s_buff[1] = rxbuff[NDEF_OFFSET_PCB + 1];
        uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
        if(_st95_cmd_send_receive(dev, s_buff, sizeof(s_buff), ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
            if(_check_ndef(dev, rxbuff) == ST95_OK) {
                PRINTSTR("\t\t >>> S BLOCK OK\n");
                return ST95_OK;  
            }
            if(_st95_cmd_send_receive(dev, s_buff, sizeof(s_buff), ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
                if(_check_ndef(dev, rxbuff) == ST95_OK) {
                    PRINTSTR("\t\t >>> S BLOCK OK\n");
                    return ST95_OK;  
                }
            }  
        }
        PRINTSTR("\t\t >>> S BLOCK ERROR\n");
        return ST95_ERROR;
    }
    
    return ST95_ERROR;
}    

static uint8_t _select_app(const st95_t * dev, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[16] = { 0x00 };
    uint8_t data[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;

    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_SELECT_FILE;
    iso_txbuff[len++] = 0x04;
    iso_txbuff[len++] = 0x00;

    iso_txbuff[len++] = sizeof(data);

    memcpy(iso_txbuff + len, data, sizeof(data));
    len += sizeof(data);

    iso_txbuff[len++] = 0x00;
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_OK) {
            return ST95_OK;  
        }
    }

    return ST95_ERROR;
}

static uint8_t _select_cc_file(const st95_t * dev, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[16] = { 0x00 };
    uint8_t data[] = { 0xE1, 0x03 };
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;

    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_SELECT_FILE;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x0C;

    iso_txbuff[len++] = sizeof(data);

    memcpy(iso_txbuff + len, data, sizeof(data));
    len += sizeof(data);
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_OK) {
            return ST95_OK;  
        }
    }

    return ST95_ERROR;      
}

static uint8_t _read_cc_file(const st95_t * dev, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[16] = { 0x00 };
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;

    iso_txbuff[len++] = 0x02;
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
            return ST95_ERROR;  
        }    
    }
    else {
        return ST95_ERROR;
    }
    picc.cc_size = (rxbuff[3] << 8) | rxbuff[4];
    
    len = 0;
    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;
    
    iso_txbuff[len++] = (uint8_t)(picc.cc_size & 0xFF);
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {     
        if(_check_ndef(dev, rxbuff) == ST95_OK) {           
            picc.ndef_id = (rxbuff[12] << 8) | rxbuff[13];
            picc.ndef_read_max = (rxbuff[6] << 8) | rxbuff[7];
            picc.ndef_write_max = (rxbuff[8] << 8) | rxbuff[9];
            picc.ndef_size = (rxbuff[14] << 8) | rxbuff[15];
            return ST95_OK;  
        }     
    }

    return ST95_ERROR;      
}

static uint8_t _select_ndef(const st95_t * dev, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[16] = { 0x00 };
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;

    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_SELECT_FILE;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x0C;
    iso_txbuff[len++] = sizeof(picc.ndef_id);
    iso_txbuff[len++] = (uint8_t)(picc.ndef_id >> 8);
    iso_txbuff[len++] = (uint8_t)(picc.ndef_id & 0xFF);

    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
       if(_check_ndef(dev, rxbuff) == ST95_OK) {
            return ST95_OK;  
        }    
    }
    
    return ST95_ERROR;
}

static uint8_t _read_ndef(const st95_t * dev, uint8_t * data, uint16_t length, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[16] = { 0x00 };
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    if(length > picc.ndef_size) {
        return ST95_ERROR;
    }  

    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;

    iso_txbuff[len++] = 0x02;
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ST95_MAX_BYTE_BUFF) == ST95_OK) {       
        if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
            PRINTSTR("\t\t >>> READ NDEF length: ERROR\n");
            return ST95_ERROR;  
        }       
    }
    else {
        return ST95_ERROR;
    }
    
    picc.ndef_length = (rxbuff[ST95_DATA_OFFSET + 1] << 8) | rxbuff[ST95_DATA_OFFSET + 2];
    
    if(length > picc.ndef_length) {
        PRINTSTR("\t\t >>> READ NDEF length: INVALID\n");
        return ST95_ERROR;
    }
    
    len = 0;
    iso_txbuff[len++] = iblock;
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = NDEF_PACK_OFFSET_DATA;

    if(length <= picc.ndef_read_max) {
        iso_txbuff[len++] = length;
        if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ST95_MAX_BYTE_BUFF) == ST95_OK) {
            if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
                PRINTSTR("\t\t >>> READ NDEF data: ERROR\n");
                return ST95_ERROR;  
            }    
        }
        else {
            return ST95_ERROR;
        }
        PRINTSTR("\t\t >>> READ NDEF data: SUCCESS\n");
        memcpy(data, rxbuff + NDEF_OFFSET_DATA, length);
        return ST95_OK;
    }
        
    len = 0;
    iso_txbuff[len++] = iblock;
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = NDEF_PACK_OFFSET_DATA;

    iso_txbuff[len++] = picc.ndef_read_max & 0xFF;
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
            PRINTSTR("\t\t >>> READ NDEF data 1: ERROR\n");
            return ST95_ERROR;  
        }
    }
    else {
        return ST95_ERROR;
    }
    PRINTSTR("\t\t >>> READ NDEF data 1: SUCCESS\n");
    memcpy(data, rxbuff + NDEF_OFFSET_DATA, picc.ndef_read_max);
    
    len = 0;
    iso_txbuff[len++] = iblock;
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_READ_BINARY;   
    iso_txbuff[len++] = (picc.ndef_read_max >> 8) & 0xFF;
    iso_txbuff[len++] = (picc.ndef_read_max & 0xFF) + 0x02;

    iso_txbuff[len++] = (length - picc.ndef_read_max) & 0xFF;
    
    
     if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ST95_MAX_BYTE_BUFF) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_OK) {           
            memcpy(data + picc.ndef_read_max, rxbuff + NDEF_OFFSET_DATA, (length - picc.ndef_read_max));
            PRINTSTR("\t\t >>> READ NDEF data 2: SUCCESS\n");
            return ST95_OK;  
        }    
    }
    PRINTSTR("\t\t >>> READ NDEF data 2: ERROR\n");
    return ST95_ERROR;
}

static uint8_t _update_ndef(const st95_t * dev, uint8_t * data, uint16_t length, uint8_t * rxbuff)
{
    uint8_t iso_txbuff[255] = { 0x00 };
    uint16_t len = 0;
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;

    if(length > picc.ndef_size) {
        return ST95_ERROR;
    }

        /* First step - erase ndef data length */
    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_UPDATE_BINARY;   
        /* Offset */
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;
    
    iso_txbuff[len++] = 0x02;
        /* Erase length ndef data */
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;
    
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {     
        if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
            PRINTSTR("\t\t >>> ERASE NDEF length: ERROR\n"); 
            return ST95_ERROR;  
        }
    }
    else {
        return ST95_ERROR;
    }
    PRINTSTR("\t\t >>> ERASE NDEF length: SUCCESS\n");  
    /* Second step - update data */
    len = 0;    
    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_UPDATE_BINARY;   
        /* Offset */
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = NDEF_PACK_OFFSET_DATA;
        /* Length */
    if(length <= picc.ndef_write_max) {
        iso_txbuff[len++] = length & 0xFF;

        memcpy(iso_txbuff + len, data, length);
        len += length;
        
        if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {  
            if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
                PRINTSTR("\t\t >>> UPDATE NDEF data: ERROR\n"); 
                return ST95_ERROR;  
            }    
        }
        else {
            return ST95_ERROR;
        }      
        PRINTSTR("\t\t >>> UPDATE NDEF data: SUCCESS \n");          
    }
    else {
        
        iso_txbuff[len++] = picc.ndef_write_max;
        memcpy(iso_txbuff + len, data, picc.ndef_write_max);
        len += picc.ndef_write_max;
        
        if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
            if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
                PRINTSTR("\t\t >>> UPDATE NDEF data 1: ERROR\n"); 
                return ST95_ERROR;  
            }    
        }
        else {
            return ST95_ERROR;
        }        
        PRINTSTR("\t\t >>> UPDATE NDEF data 1: SUCCESS\n"); 
        len = 0;    
        iso_txbuff[len++] = iblock;
        
        iso_txbuff[len++] = ISO7816_CLASS_0X00;
        iso_txbuff[len++] = ISO7816_UPDATE_BINARY;   
            /* Offset */
        iso_txbuff[len++] = (picc.ndef_write_max >> 8) & 0xFF;
        iso_txbuff[len++] = (picc.ndef_write_max & 0xFF) + NDEF_PACK_OFFSET_DATA;
        
        iso_txbuff[len++] = (length - picc.ndef_write_max) & 0xFF;
        
        memcpy(iso_txbuff + len, data + picc.ndef_write_max, length - picc.ndef_write_max);
        
        if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
            if(_check_ndef(dev, rxbuff) == ST95_ERROR) {
                PRINTSTR("\t\t >>> UPDATE NDEF data 2: ERROR\n"); 
                return ST95_ERROR;  
            }    
        }
        else {
            return ST95_ERROR;
        }        
        PRINTSTR("\t\t >>> UPDATE NDEF data 2: SUCCESS\n"); 
    }    
           
    /* Third step - update data length */    
    len = 0;
    iso_txbuff[len++] = iblock;
    
    iso_txbuff[len++] = ISO7816_CLASS_0X00;
    iso_txbuff[len++] = ISO7816_UPDATE_BINARY;   
        /* Offset */
    iso_txbuff[len++] = 0x00;
    iso_txbuff[len++] = 0x00;

    iso_txbuff[len++] = 0x02;
    
    iso_txbuff[len++] = (length >> 8) & 0xFF;
    iso_txbuff[len++] = length & 0xFF;
    
    if(_st95_cmd_send_receive(dev, iso_txbuff, len, ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {
        if(_check_ndef(dev, rxbuff) == ST95_OK) {
            PRINTSTR("\t\t >>> UPDATE NDEF length: SUCCESS\n"); 
            return ST95_OK;  
        }
        else {
            PRINTSTR("\t\t >>> UPDATE NDEF length: ERROR\n"); 
        }
    }
    PRINTSTR("\t\t >>> UPDATE NDEF: ERROR\n"); 
    return ST95_ERROR;
}

uint8_t _deselect_ndef(const st95_t * dev, uint8_t * rxbuff)
{
        /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    uint8_t cmd = ISO14443A_CMD_DESELECT;
    if(_st95_cmd_send_receive(dev, &cmd, sizeof(cmd), ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {       
        if(rxbuff[ST95_DATA_OFFSET] == ISO14443A_CMD_DESELECT) {
            PRINTSTR("\t\t >>> DESELECT NDEF OK\n");
            return ST95_OK;
        }
        if(_st95_cmd_send_receive(dev, &cmd, sizeof(cmd), ctrl_byte, rxbuff, ISO14443A_ANSWER_MAX_BYTE) == ST95_OK) {  
            if(rxbuff[ST95_DATA_OFFSET] == ISO14443A_CMD_DESELECT) {
                PRINTSTR("\t\t >>> DESELECT NDEF OK\n");
                return ST95_OK;
            }
        }
    }
    PRINTSTR("\t\t >>> DESELECT NDEF ERROR\n");
    return ST95_ERROR;
}

/**
 * @brief This function calculate and check BCC
 * 
 * @param[in]   length: Length of data
 * @param[in]   data:   Pointer to the data
 * @param[in]   bcc:    BCC
 * 
 * @return  true:   Valid BCC
 * @return  false:  in case of an error
 */
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

/**
 * @brief   This function send REQA command to a tag
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[in]   size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */ 
uint8_t _iso14443a_reqa(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff)
{
	uint8_t data = ISO14443A_CMD_REQA;
    
    picc.type = 0;
    picc.sak = 0;
    picc.uid_length = 0;
    memset(picc.uid, 0x00, ISO14443A_UID_LENGTH_MAX); 
    picc.is_ats = false;
    picc.cc_size = 0;
    picc.ndef_length = 0;
    picc.ndef_id = 0;    
    picc.ndef_read_max = 0; 
    picc.ndef_write_max = 0; 
    picc.ndef_size = 0;
    
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 7 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_7;

    if(_st95_cmd_send_receive(dev, &data, sizeof(data), ctrl_byte, rxbuff, size_rx_buff) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}

/**
 * @brief   This function returns UID size 
 * 
 * @param[in]   uid_byte: ATQA uid byte
 * 
 * @return  UID size 
 */
static uint8_t _iso14443a_get_uidsize(uint8_t byte_size)
{   
    uint8_t size = 0;
    size = (byte_size >> ISO14443A_OFFSET_UID_SIZE) & ISO14443A_MASK_UID_SIZE;
   
    if(size == ISO14443A_ATQA_SINGLE) { 
        picc.uid_length = ISO14443A_UID_SINGLE;
    }
    else if(size == ISO14443A_ATQA_DOUBLE) { 
        picc.uid_length = ISO14443A_UID_DOUBLE;
    }
    else if(size == ISO14443A_ATQA_TRIPLE) { 
        picc.uid_length = ISO14443A_UID_TRIPLE;
    }
    else {
        return ST95_ERROR;
    }
    
	return ST95_OK;
}

/**
 * @brief   This function check if ID is complete
 * @brief   This check is carry out on SAK byte.
 * 
 * @param[in]   sak_byte: SAK byte
 * 
 * @return  0:  UID is complete
 * @return  1:  in case of an error
 */
static uint8_t _iso14443a_is_uid_complete(uint8_t sak_byte)
{
	if ((sak_byte & ISO14443A_MASK_SAK_UID_NOT_COMPLETE) == ISO14443A_SAK_UID_NOT_COMPLETE) {
		return ST95_ERROR;
    }
	else {
		return ST95_OK;
    }        
}

/**
 * @brief   This function send WUPA command to a tag
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[in]   size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */ 
uint8_t _iso14443a_wupa(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff)
{
	uint8_t data = ISO14443A_CMD_WUPA;
    
    picc.type = 0;
    picc.sak = 0;
    picc.uid_length = 0;
    memset(picc.uid, 0x00, ISO14443A_UID_LENGTH_MAX); 
    picc.is_ats = false;
    picc.cc_size = 0;
    picc.ndef_length = 0;
    picc.ndef_id = 0;    
    picc.ndef_read_max = 0; 
    picc.ndef_write_max = 0; 
    picc.ndef_size = 0;

	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 7 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_7;
    if(_st95_cmd_send_receive(dev, &data, sizeof(data), ctrl_byte, rxbuff, size_rx_buff) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;
}

/**
 * @brief   This function send HLTA command to a tag
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[in]   size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */ 
uint8_t _iso14443a_hlta(const st95_t * dev, uint8_t * rxbuff, uint16_t size_rx_buff)
{    
    uint8_t data[2] = { ISO14443A_CMD_HLTA, 0x00 };
    /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    if(_st95_cmd_send_receive(dev, data, sizeof(data), ctrl_byte, rxbuff, size_rx_buff) != ST95_OK) {
        PRINTSTR("\t\t >>> HLTA OK\n");
        return ST95_OK;
    }
    return ST95_ERROR;
}

/**
 * @brief   This function send Anticollision command
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[in]   level:          Anticollision level value
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[in]   size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
static uint8_t _iso14443a_anticollision(const st95_t * dev, uint8_t level, uint8_t * rxbuff, uint16_t size_rx_buff)
{
	uint8_t data[2] = { level, ISO14443A_NVB_20 };
       
	/* Control byte: Not used topaz format, Not SplitFrame, Not append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8;
    
    if(_st95_cmd_send_receive(dev, data, sizeof(data), ctrl_byte, rxbuff, size_rx_buff) == ST95_OK) {
        return ST95_OK;
    }

    return ST95_ERROR;    
}

/**
 * @brief   This function send Select command to a tag
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[in]   level:          Select Command level value
 * @param[in]   num:            Number of byte of UID in parameters
 * @param[in]   uid_sel:        Pointer on UID append to Select command
 * @param[out]  rxbuff:         Pointer to the receive buffer
 * @param[in]   size_rx_buff:   Size of the receive buffer
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
static uint8_t _iso14443a_select(const st95_t * dev, uint8_t level, uint8_t num, uint8_t * uid_sel, uint8_t * rxbuff, uint16_t size_rx_buff)
{
	uint8_t data[ISO14443A_CMD_MAX_BYTE] = { 0 };
    	
	data[0] = level;
	data[1] = ISO14443A_NVB_70;
	memcpy(data + 2, uid_sel, num);
    
    /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    if(_st95_cmd_send_receive(dev, data, num + 2, ctrl_byte, rxbuff, size_rx_buff) == ST95_OK) {
        return ST95_OK;
    }
    
    return ST95_ERROR;   
}

static void _iso14443a_support_ats(void) 
{
    /* Checks if the RATS command is supported by the card */
	if(picc.sak & ISO14443A_FLAG_ATS_SUPPORTED) {
        picc.is_ats = true;
	}
    else {
        picc.is_ats = false;
    }
}

static uint8_t _iso14443a_rats(const st95_t * dev, uint8_t param, uint8_t * rxbuff, uint16_t size_rx_buff)
{
    uint8_t data[2] = { ISO14443A_CMD_RATS, param };
    
    /* Control byte: Not used topaz format, Not SplitFrame, Append CRC, 8 significant bits in last byte */	
    uint8_t ctrl_byte = ISO14443A_NUM_SIGN_BIT_8 | ISO14443A_APPEND_CRC;
    
    if(_st95_cmd_send_receive(dev, data, 2, ctrl_byte, rxbuff, size_rx_buff) == ST95_OK) {       
        return ST95_OK;
    }
    
    return ST95_ERROR; 
}

uint8_t _iso14443a_type_tag(void)
{
    /* Check the Tag type found */
    if((picc.sak & 0x60) == 0x00) {
        picc.type = ISO14443A_TYPE_2;
        return ST95_OK;
    }
    else if((picc.sak & 0x20) == 0x20) {
        picc.type = ISO14443A_TYPE_4;
        return ST95_OK;
    }
    return ST95_ERROR;
}    

uint8_t _iso14443a_config_fdt(const st95_t * dev)
{
    uint8_t params[4] = { 0x00 };
    
     /* FDT = ((2^PP) * (MM + 1) * (DD + 128) * 32) / 13.56 [usec] */
     /* 9-1-0 => 309 [ms] */
    uint8_t pp = 0x09; /* Min = 4 */
    uint8_t mm = 1; /* Min = 1 */
    uint8_t dd = 0; /* Min = 0 */
    
    // uint32_t fdt = (uint32_t)((1 << pp)*(mm+1)*(dd + 128)*32*100/1356);
    // printf("FDT: %ld [usec]\n", fdt);
    
    params[0] = (ST95_TX_RATE_14443A << 6) | (ST95_RX_RATE_14443A << 4);
    params[1] = pp;
    params[2] = mm;
    params[3] = dd;
    
    if(_st95_select_iso14443a(dev, params, 4) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    if(_st95_modify_modulation_gain(dev, ST95_WR_MODULATION_95, ST95_WR_GAIN_32_DB) == ST95_ERROR) {
        return ST95_ERROR;
    }
        
    if(_st95_set_timer_window(dev, ST95_WR_TIMER_WINDOW_VAL) == ST95_ERROR) {
        return ST95_ERROR;
    }

    return ST95_OK;   
}

int iso14443a_read_tag(const st95_t * dev, uint8_t * data, uint16_t length, uint8_t * rxbuff)
{
    if(_iso14443a_get_ats(dev, rxbuff) == ST95_ERROR) {
        _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_app(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_cc_file(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_read_cc_file(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_ndef(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_read_ndef(dev, data, length, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_deselect_ndef(dev, rxbuff) == ST95_ERROR) {
        _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

     _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
    return ST95_OK;
}

int iso14443a_write_tag(const st95_t * dev, uint8_t * data, uint8_t length, uint8_t * rxbuff)
{
    if(_iso14443a_get_ats(dev, rxbuff) == ST95_ERROR) {
        _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_app(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_cc_file(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_read_cc_file(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_select_ndef(dev, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_update_ndef(dev, data, length, rxbuff) == ST95_ERROR) {
         _deselect_ndef(dev, rxbuff);
         _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }  

    if(_deselect_ndef(dev, rxbuff) == ST95_ERROR) {
        _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

     _iso14443a_hlta(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE);
    return ST95_OK;
}

uint8_t _iso14443a_get_ats(const st95_t * dev, uint8_t * rxbuff)
{         
    if(_iso14443a_wupa(dev, rxbuff, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }
    
    if(_iso14443a_get_uidsize(rxbuff[ISO14443A_OFFSET_ATQA_FIRST_BYTE]) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    if(_iso14443a_anticoll_algorithm(dev, rxbuff) == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    if(_iso14443a_type_tag() == ST95_ERROR) {
        return ST95_ERROR;
    }
    
    _iso14443a_support_ats();
    
    if(picc.is_ats) {
        uint8_t param_rats = (ISO14443A_FSD_256 << 4) | ISO14443A_CID_DEF;
        if(_iso14443a_rats(dev, param_rats, rxbuff, ISO14443A_APDU_CMD_MAX_BYTE) == ST95_ERROR) {
            return ST95_ERROR;
        }       
    }
    else {
        return ST95_ERROR;
    }
    
    if(_iso14443a_config_fdt(dev) == ST95_ERROR) {
        return ST95_ERROR;
    }    

    return ST95_OK;
}

int iso14443a_get_uid(const st95_t * dev, uint8_t * iso_rxbuf, uint8_t * length_uid, uint8_t * uid, uint8_t * sak)
{                    
    if(_iso14443a_wupa(dev, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) == ST95_ERROR) {
        _iso14443a_hlta(dev, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_iso14443a_get_uidsize(iso_rxbuf[ISO14443A_OFFSET_ATQA_FIRST_BYTE]) == ST95_ERROR) {
        _iso14443a_hlta(dev, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE);
        return ST95_ERROR;
    }

    if(_iso14443a_anticoll_algorithm(dev, iso_rxbuf) == ST95_OK) {
        _iso14443a_hlta(dev, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE);
        *length_uid = picc.uid_length;
        *sak = picc.sak;
        for(uint32_t i = 0; i < picc.uid_length; i++) {
            uid[i] = picc.uid[i];
        }
        return ST95_OK;
    }
    
    _iso14443a_hlta(dev, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE);
    return ST95_ERROR;
}


/**
 * @brief   This function get iso14443A UID card
 * 
 * @param[in]   dev:            Pointer to ST95 device descriptor
 * @param[out]  length_uid:     Pointer to the length UID
 * @param[out]  uid:            Pointer to the card UID
 * @param[out]  sak:            Pointer to the card SAK(Select ACK) byte
 * 
 * @return  0:  the command has been successfully executed
 * @return  1:  in case of an error
 */
static int _iso14443a_anticoll_algorithm(const st95_t * dev, uint8_t * iso_rxbuf)
{     
    // Select cascade level 1
    if(_iso14443a_anticollision(dev, ISO14443A_SELECT_LVL1, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    //  Check BCC
    if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
        return ST95_ERROR;
    }
    // copy UID from ST95 response
    if (picc.uid_length == ISO14443A_UID_SINGLE){
        memcpy(picc.uid, &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE );
    }
    else {
        memcpy(picc.uid, &iso_rxbuf[ST95_DATA_OFFSET + 1], ISO14443A_UID_SINGLE - 1 );
    }

    // Send Select command 1
    if(_iso14443a_select(dev, ISO14443A_SELECT_LVL1, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        picc.sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        picc.uid_length = ISO14443A_UID_SINGLE;
        return ST95_OK;
    }

    // Select cascade level 2
    if(_iso14443a_anticollision(dev, ISO14443A_SELECT_LVL2, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    //  Check BCC
    if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
        return ST95_ERROR;
    }

    // copy UID from ST95 response
    if (picc.uid_length == ISO14443A_UID_DOUBLE) {
        memcpy(&picc.uid[ISO14443A_UID_SINGLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE );
    }
    else {
        memcpy(&picc.uid[ISO14443A_UID_SINGLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_SINGLE - 1);
    }
    // Send Select command 2
    if(_iso14443a_select(dev, ISO14443A_SELECT_LVL2, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        picc.sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        picc.uid_length = ISO14443A_UID_DOUBLE;
        return ST95_OK;
    }

    // Select cascade level 3
    if(_iso14443a_anticollision(dev, ISO14443A_SELECT_LVL3, iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    //  Check BCC
    if(!_iso14443a_check_bcc(4, iso_rxbuf + ST95_DATA_OFFSET, iso_rxbuf[ST95_DATA_OFFSET + 4])) {
        return ST95_ERROR;
    }

    // copy UID from ST95 response
    if (picc.uid_length == ISO14443A_UID_TRIPLE) {
        memcpy(&picc.uid[ISO14443A_UID_DOUBLE - 1], &iso_rxbuf[ST95_DATA_OFFSET], ISO14443A_UID_DOUBLE );
    }

    // Send Select command 3
    if(_iso14443a_select(dev, ISO14443A_SELECT_LVL3, ISO14443A_NUM_BYTE_SELECT, &iso_rxbuf[ISO14443A_OFFSET_UID_SELECT], iso_rxbuf, ISO14443A_ANSWER_MAX_BYTE) != ST95_OK) {
        return ST95_ERROR;
    }

    if(_iso14443a_is_uid_complete(iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE]) == ST95_OK) {
        picc.sak = iso_rxbuf[ISO14443A_OFFSET_SAK_BYTE];
        picc.uid_length = ISO14443A_UID_TRIPLE;
        return ST95_OK;
    }
    
    return ST95_ERROR;
}