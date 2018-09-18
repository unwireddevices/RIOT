/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_m24srxx
 * @{
 *
 * @file
 * @brief       M24SRxx NFC memory driver implementation
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "m24sr_core.h"
#include "m24sr.h"
#include "m24sr_internal.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "xtimer.h"

#define ENABLE_DEBUG                (1)
#include "debug.h"


#define M24SR_PWD_LEN               0x10
#define NDEF_FILE_LEN_POS           0
#define NDEF_FILE_LEN_NUM_BYTES     2

static cmd_apdu_t cmd;
static uint8_t data_buffer[0xFF];
static uint8_t cmd_data[0xFF];

static uint8_t device_id_byte = 0x00;




static m24sr_waiting_time_mode_t synchro_mode = M24SR_WAITING_TIME_POLLING;
volatile uint8_t  gpo_low = 0;


/**
  * @brief  Initialize the command and response structure
  * 
  * @param  None
  * @retval None
  */
static void _m24sr_init_structure (void);
static void m24sr_set_i2c_synchro_mode(m24sr_waiting_time_mode_t mode);
static uint16_t _m24sr_update_crc (uint8_t ch, uint16_t *crc);
static uint16_t _m24sr_compute_crc (uint8_t *data, uint8_t length);
static int _m24sr_is_correct_crc_residue (uint8_t *data, uint8_t length);
static void _m24sr_build_iblock_cmd (uint16_t category, cmd_apdu_t cmd, uint8_t *iblock, uint16_t *numByte);

#if 0
static int _is_iblock(uint8_t *buffer);
static int _is_rblock(uint8_t *buffer);
#endif
static int _is_sblock(uint8_t *buffer);

static int m24sr_select_application(const m24sr_t *dev);
static int m24sr_select_capability_container_file(const m24sr_t *dev, uint16_t cc_file_id);
static int m24sr_select_ndef_file(const m24sr_t *dev, uint16_t ndef_file_id);
static int m24sr_select_system_file(const m24sr_t *dev, uint16_t sys_file_id);
static int m24sr_read_binary(const m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint16_t len);
static int m24sr_update_binary(const m24sr_t *dev, uint16_t offset, uint8_t *src_data, uint16_t len);
static int m24sr_verify(const m24sr_t *dev, uint16_t pwd_id, uint8_t num_pwd_byte , uint8_t *pwd);

static int m24sr_extended_read_binary(const m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint32_t len);



uint16_t m24sr_manage_i2c_gpo(const m24sr_t *dev, m24sr_gpo_mode_t gpo_i2c_config);


static int _m24sr_fwt_extension(const m24sr_t *dev, uint8_t fwt_byte);



uint16_t m24sr_read_data (const m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size);
uint16_t m24sr_write_data (const m24sr_t *dev, uint16_t offset, uint8_t *src, uint16_t size);



/////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  this functions configure I2C synchronization mode
  * @param  mode : interruption or polling
  * @retval None
  */
void m24sr_set_i2c_synchro_mode(m24sr_waiting_time_mode_t mode) {
#if defined (I2C_GPO_SYNCHRO_ALLOWED) || defined (I2C_GPO_INTERRUPT_ALLOWED)
    synchro_mode = mode;
#else
    if (mode == M24SR_WAITING_TIME_GPO || mode == M24SR_INTERRUPT_GPO)
        synchro_mode = M24SR_WAITING_TIME_POLLING;
    else
        synchro_mode = mode;
#endif /*  I2C_GPO_SYNCHRO_ALLOWED */
}


static void _m24sr_init_structure (void) 
{
    /* build the command */
    cmd.header.CLA = 0x00;
    cmd.header.INS = 0x00;
    /* copy the offset */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x00;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x00;
    /* copy the number of byte to read */
    cmd.body.LE = 0x00;
    cmd.body.data = cmd_data;
}

static uint16_t _m24sr_update_crc (uint8_t ch, uint16_t *crc)
{
    ch = (ch ^ (uint8_t)((*crc) & 0x00FF));
    ch = (ch ^ (ch << 4));
    *crc = (*crc >> 8) ^ ((uint16_t)ch << 8) ^ ((uint16_t)ch << 3) ^ ((uint16_t)ch >> 4);

    return (*crc);
}


static uint16_t _m24sr_compute_crc (uint8_t *data, uint8_t length)
{
    uint8_t block;
    uint16_t crc;

    crc = 0x6363; // ITU-V.41

    do {

        block = *data++;
        _m24sr_update_crc(block, &crc);
    } while (--length);

    return crc ;
}


/**
* @brief    This function computes the CRC16 residue as defined by CRC ISO/IEC 13239
* @param    DataIn      :   input to data
* @param        Length      :   Number of bits of DataIn
* @retval   Status (SW1&SW2)    :   CRC16 residue is correct
* @retval   M24SR_ERROR_CRC     :  CRC16 residue is false
*/
static int _m24sr_is_correct_crc_residue (uint8_t *data, uint8_t length)
{
    uint16_t res_crc = 0;

    /* check the CRC16 Residue */
    if (length != 0)
        res_crc = _m24sr_compute_crc (data, length);

    if (res_crc == 0x0000) {
        /* Good CRC, but error status from M24SR */
        return (uint16_t)(((data[length - UB_STATUS_OFFSET] << 8) & 0xFF00) | (data[length - LB_STATUS_OFFSET] & 0x00FF));
    } else {
        res_crc = 0;
        res_crc = _m24sr_compute_crc (data, 5);
        if (res_crc != 0x0000) {
            /* Bad CRC */
            return M24SR_WRONG_CRC;
        } else {
            /* Good CRC, but error status from M24SR */
            return (uint16_t)(((data[1] << 8) & 0xFF00) | (data[2] & 0x00FF));
        }
    }
}


/**
  * @brief This functions creates an I block command according to the structure and category APDU Command.
  * 
  * @param[in]  category - block formation parameters
  * @param[in]  cmd - structue which contains the field of the different parameter
  * @param[out] iblock - pointer of the block created
  * @param[out] numByte - number of byte of the block
  * @retval     None
  */
static void _m24sr_build_iblock_cmd (uint16_t category, cmd_apdu_t cmd, uint8_t *iblock, uint16_t *numByte) {
    uint16_t crc16;
    static uint8_t block_num = 0x01;

    (*numByte) = 0;

    /* add the PCD byte */
    if ((category & M24SR_PCB_NEEDED) != 0) {
        /* toggle the block number */
        block_num = TOGGLE (block_num);
        /* Add the I block byte */
        iblock[(*numByte)++] = 0x02|block_num;
    }
    /* add the DID byte */
    if ((block_num & M24SR_DEV_ID_NEEDED) != 0) {
        /* Add the I block byte */
        iblock[(*numByte)++] = device_id_byte;
    }
    /* add the Class byte */
    if ((category & M24SR_CLA_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.header.CLA;
    }
    /* add the instruction byte */
    if ( (category & M24SR_INS_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.header.INS;
    }
    /* add the Selection Mode byte */
    if ((category & M24SR_P1_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.header.P1;
    }
    /* add the Selection Mode byte */
    if ((category & M24SR_P2_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.header.P2;
    }
    /* add Data field length byte */
    if ((category & M24SR_LC_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.body.LC;
    }
    /* add Data field  */
    if ((category & M24SR_DATA_NEEDED) != 0) {
        memcpy(&(iblock[(*numByte)]) , cmd.body.data, cmd.body.LC ) ;
        (*numByte) += cmd.body.LC;
    }
    /* add Le field  */
    if ((category & M24SR_LE_NEEDED) != 0) {
        iblock[(*numByte)++] = cmd.body.LE;
    }
    /* add CRC field  */
    if ((category & M24SR_CRC_NEEDED) != 0) {
        crc16 = _m24sr_compute_crc(iblock, (uint8_t) (*numByte));
        /* append the CRC16 */
        iblock [(*numByte)++] = ((uint8_t)(crc16 & 0x00FF));
        iblock [(*numByte)++] = ((uint8_t)((crc16 & 0xFF00) >> 8));
    }
}


#if 0

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an I-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a I-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a I-Block
*/
static int _is_iblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_IBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an R-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a R-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a R-Block
*/
static int _is_rblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_RBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}
#endif

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an s-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a S-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a S-Block
*/
static int _is_sblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_SBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}

// Standart cmd set
/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param t [description]
 * @param data [description]
 * @param len [description]
 * @return [description]
 */
static int m24sr_select_application(const m24sr_t *dev) {
    /* CLA  | INS  |  P1  |  P2  |  Lc  |  Data            | Le   */
    /* 0x00 | 0xA4 | 0x04 | 0x00 | 0x07 | 0xD2760000850101 | 0x00 */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;
    

    const uint8_t buffer[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_SELECT;
    /* copy the offset */
    cmd.header.P1 = 0x04;
    cmd.header.P2 = 0x00;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x07;
    /* copy the data */
    memcpy(cmd.body.data, buffer, cmd.body.LC);
    /* copy the number of byte to read */
    cmd.body.LE = 0x00;
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_0, cmd, data, &len);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, len);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(data, num_byte_read); //@TODO Return values not error. Return value is code SW
    DEBUG ("Return SW code 0x%04X", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param dev [description]
 * @param cmd [description]
 * @param resp [description]
 * @param data [description]
 * @param len [description]
 * @return [description]
 */
static int m24sr_select_capability_container_file(const m24sr_t *dev, uint16_t cc_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0xE103 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;
    
    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_SELECT;
    /* copy the offset */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x0C;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x02;
    /* copy the File Id */
    cmd.body.data[0] = GET_MSB(cc_file_id);
    cmd.body.data[1] = GET_LSB(cc_file_id);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &len);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, len);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(data, num_byte_read); //@TODO Return values not error. Return value is code SW
    DEBUG("Return SW code 0x%04X", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_select_ndef_file(const m24sr_t *dev, uint16_t ndef_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0x0001 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_SELECT;
    /* copy the offset */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x0C;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x02;
    /* copy the File Id */
    cmd.body.data[0] = GET_MSB(ndef_file_id);
    cmd.body.data[1] = GET_LSB(ndef_file_id);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &len);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, len);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(data, num_byte_read); //@TODO Return values not error. Return value is code SW
    DEBUG("Return SW code 0x%04X", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_select_system_file(const m24sr_t *dev, uint16_t sys_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0xE101 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_SELECT;
    /* copy the offset */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x0C;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x02;
    /* copy the File Id */
    cmd.body.data[0] = GET_MSB(sys_file_id);
    cmd.body.data[1] = GET_LSB(sys_file_id);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &len);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, len);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(data, num_byte_read); //@TODO Return values not error. Return value is code SW
    DEBUG("Return SW code 0x%04X", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_read_binary(const m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint16_t len) {
    /* CLA  | INS  | P1  P2 | Lc  | Data | Le                        */
    /* 0x00 | 0xB0 | Offset |  -  | -    | NubBytes (0x01 =< Le =< 0xF6) */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *data = data_buffer;

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_READ_BINARY;
    /* copy the offset */
    cmd.header.P1 = GET_MSB(offset);
    cmd.header.P2 = GET_LSB(offset);
    /* copy the number of byte to read */
    cmd.body.LE = len;
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_2, cmd, data, &num_byte);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, len + M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(data, len + M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    memcpy(dst_data, data, len);
    DEBUG("Return SW code 0x%04X", status);    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}


static int m24sr_update_binary(const m24sr_t *dev, uint16_t offset, uint8_t *src_data, uint16_t len) {
    /* CLA  | INS  | P1  P2 | Lc                            | Data          | Le */
    /* 0x00 | 0xD6 | Offset | NubBytes (0x01 =< Le =< 0xF6) | Data Lc Bytes |  - */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;

    uint8_t *data = data_buffer;

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_UPDATE_BINARY;
    /* copy the offset */
    cmd.header.P1 = GET_MSB(offset);
    cmd.header.P2 = GET_LSB(offset);
    /* copy the number of byte of the data field */
    cmd.body.LC = len;
    /* copy the File Id */
    memcpy(cmd.body.data , src_data, len);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &num_byte);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* if the response is a Watiting frame extenstion request */
    if (_is_sblock(data) == M24SR_OK) {
        /*check the CRC */
        if ( _m24sr_is_correct_crc_residue(data, M24SR_WATING_TIME_EXT_RESPONSE_NUM_BYTE) != M24SR_WRONG_CRC) {
            /* send the FrameExension response*/
            status = _m24sr_fwt_extension(dev, data[M24SR_OFFSET_PCB + 1]); //@FIXME
        }
    } else {
        status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
        DEBUG("Return SW code 0x%04X", status);    
        if (status != SW_OK) {
            return M24SR_ERROR;
        }
    }    
    return ret;
}


static int m24sr_verify(const m24sr_t *dev, uint16_t pwd_id, uint8_t num_pwd_byte , uint8_t *pwd) {
    /* CLA  | INS  | P1 P2  | Lc   | Data         | Le */
    /* 0x00 | 0x20 | 0x000X | 0x10 | PWD Lc Bytes | -  */
    /*               0x0001 - Read NDEF PWD XMIT       */
    /*               0x0002 - Write NDEF PWD XMIT      */
    /*               0x0003 - I2C PWD XMIT             */
    int ret = M24SR_OK;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len = 0;

    /*check the parameters */
    if (pwd_id > 0x0003) {
        return M24SR_ERROR_PARAM;
    }
    if ((num_pwd_byte != 0x00) && (num_pwd_byte != 0x10)) {
        return M24SR_ERROR_PARAM;
    }

     /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_VERIFY;
    /* copy the Password Id */
    cmd.header.P1 = GET_MSB(pwd_id) ;
    cmd.header.P2 = GET_LSB(pwd_id) ;
    /* copy the number of byte of the data field */
    cmd.body.LC = num_pwd_byte;

    if (num_pwd_byte == 0x10) {
        /* copy the password */
        memcpy(cmd.body.data, pwd, num_pwd_byte);
        /* build the I2C cmd */
        _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &len);
    } else {
        /* build the I2C cmd */
        _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_3, cmd, data, &len);
    }

    /* send the request */
    ret = m24sr_send_i2c_cmd(dev, data, len);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }

    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    DEBUG("Return SW code 0x%04X", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    
    return ret;
}

#if 0
void m24sr_change_reference_data(uint16_t pwd_id, uint8_t *pwd) {
    /* CLA  | INS  | P1 P2  | Lc   | Data         | Le */
    /* 0x00 | 0x24 | 0x000X | 0xX0 | PWD Lc Bytes | -  */
    /*               0x0001 - Read NDEF PWD XMIT       */
    /*               0x0002 - Write NDEF PWD XMIT      */
    /*               0x0003 - I2C PWD XMIT             */
    /*                        0x00 - No PWD            */
    /*                        0x10 - PWD Enable        */

    uint8_t   *pBuffer = uM24SRbuffer;
    uint16_t  status ;
    uint16_t  NbByte;

    /*check the parameters */
    if (pwd_id > 0x0003)
    {
        return M24SR_ERROR_PARAM;
    }

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_CHANGE_REF_DATA;
    /* copy the Password Id */
    cmd.header.P1 = GET_MSB(pwd_id);
    cmd.header.P2 = GET_LSB(pwd_id);
    /* copy the number of byte of the data field */
    cmd.body.LC = M24SR_PASSWORD_NUM_BYTE ;
    /* copy the password */
    memcpy(cmd.body.data, pwd, M24SR_PASSWORD_NUM_BYTE);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, pBuffer, &NbByte);
    /* send the request */
    ret = m24sr_send_i2c_cmd(dev, pBuffer, NbByte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /*check CRC and response code*/
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    resp.SW1 = GET_MSB(status);
    resp.SW2 = GET_LSB(status);
    memcpy(resp.data, data, len);

    if (status != SW_OK)
        return M24SR_ERROR;
    
    return ret;
}

void m24sr_enable_verification_requirement(uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0x00 | 0x28 | 0x000X | -  | -    | -  */
    /*               0x0001 - ENA Read Protect NDEF  */
    /*               0x0002 - ENA Write Protect NDEF */
    uint8_t buffer[16] ={0};

    uint16_t status;
    uint16_t num_byte = 0x0000;

    /* check the parameters */
    if((mode_protect != 0x0001) && (mode_protect != 0x0002)) {
        return M24SR_ERROR_PARAM;
    }

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_ENABLE_VERIFY_REQ;
    /* copy the password id */
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, buffer, &num_byte);
    /* send the request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* The right access to be updated in EEPROM need at least 6ms */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }

    /*check CRC and response code*/
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    resp.SW1 = GET_MSB(status);
    resp.SW2 = GET_LSB(status);
    memcpy(resp.data, data, len);

    if (status != SW_OK)
        return M24SR_ERROR;
    
    return ret;


}


int m24sr_disable_verification_requirement(const m24sr_t dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0x00 | 0x26 | 0x000X | -  | -    | -  */
    /*               0x0001 - DIS Read Protect NDEF  */
    /*               0x0002 - DIS Write Protect NDEF */
    int ret = MS24SR_OK;

    uint8_t buffer[] = {};

    uint16_t status;
    uint16_t num_byte;

    if ((mode_protect != 0x0001) && (mode_protect != 0x0002)) {
        return M24SR_ERROR_PARAM;
    }

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_DISABLE_VERIFY_REQ;
    /* copy the password id */
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, buffer, &num_byte);
    /* send request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* The right access to be updated in EEPROM need at least 6ms */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }

    /*check CRC and response code*/
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    resp.SW1 = GET_MSB(status);
    resp.SW2 = GET_LSB(status);
    memcpy(resp.data, data, len);

    if (status != SW_OK)
        return M24SR_ERROR;
    
    return ret;

}
#endif

// Proprietary STM cmd set
static int m24sr_extended_read_binary(const m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint32_t len) {
    /* CLA  | INS  | P1 P2  | Lc | Data  | Le */
    /* 0xA2 | 0xB0 | Offset |  - |   -   | Len  */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *buffer = data_buffer;


    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_READ_BINARY;
    /* copy the offset */
    cmd.header.P1 = GET_MSB(offset);
    cmd.header.P2 = GET_LSB(offset);
    /* copy the number of byte to read */
    cmd.body.LE = len;

    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_2,  cmd, buffer, &num_byte);

    ret = m24sr_send_i2c_cmd(dev, buffer, num_byte);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    ret = m24sr_rcv_i2c_response(dev, buffer, len + M24SR_STATUS_NUM_BYTE);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;

    status = _m24sr_is_correct_crc_residue(buffer, len + M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    memcpy(dst_data, buffer, len);
    DEBUG("Return SW code 0x%04X", status);    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

#if 0
int m24sr_enable_permanent_state(const m24sr_t *dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0xA2 | 0x28 | 0x000X | -  | -    | -  */
    /*               0x0001 - ENA Read Protect NDEF  */
    /*               0x0002 - ENA Write Protect NDEF */
    int ret = M24SR_OK;

    uint8_t buffer[] = {0};
    uint16_t status;
    uint16_t num_byte;

    /* check the parameters */
    if ((mode_protect != 0x0001) && (mode_protect != 0x0002))
        return M24SR_ERROR_PARAM;
    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_ENABLE_VERIFY_REQ;
    /* cope the password id*/
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_, cmd, buffer, &num_byte);

    /* send the request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;

    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    resp.SW1 = GET_MSB(status);
    resp.SW2 = GET_LSB(status);

    if (status != SW_OK)
        return M24SR_ERROR;


    return ret;
}


int m24sr_disable_permanent_state(const m24sr_t *dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0xA2 | 0x26 | 0x000X | -  | -    | -  */
    /*               0x0001 - DIS Read Protect NDEF  */
    /*               0x0002 - DIS Write Protect NDEF */
    int ret = M24SR_OK;

    uint8_t buffer[] = {0};
    uint16_t status;
    uint16_t num_byte;

    /* check the parameters */
    if ((mode_protect != 0x0001) && (mode_protect != 0x0002))
        return M24SR_ERROR_PARAM;
    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_DISABLE_VERIFY_REQ;
    /* cope the password id*/
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_, cmd, buffer, &num_byte);

    /* send the request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK)
        return M24SR_NOBUS;

    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    resp.SW1 = GET_MSB(status);
    resp.SW2 = GET_LSB(status);

    if (status != SW_OK)
        return M24SR_ERROR;


    return ret;
}
#endif


#if defined(M24SR_UPDATE_FILE_TYPE)
static int m24sr_update_file_type (const m24sr_t *dev, uint8_t file_type) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data | Le */
    /* 0xA2 | 0xD6 | 0x00 | 0x00 | 0x01 | 0x0X |  - */
    /*                                    0x04 - Type NDEF        */
    /*                                    0x05 - Type Proprietary */

    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;

    uint8_t *data = data_buffer;

    /* build the command */
    cmd.header.CLA = CLA_DEFAULT;
    cmd.header.INS = INS_UPDATE_BINARY;
    /* copy the offset */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x00;
    /* copy the number of byte of the data field */
    cmd.body.LC = 0x01;
    /* copy the File Id */
    memcpy(cmd.body.data , &file_type, 0x01);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &num_byte);
    /* send the request */ 
    ret = m24sr_send_i2c_cmd(dev, data, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* if the response is a Watiting frame extenstion request */
    if (_is_sblock(data) == M24SR_OK) {
        /*check the CRC */
        if ( _m24sr_is_correct_crc_residue(data, M24SR_WATING_TIME_EXT_RESPONSE_NUM_BYTE) != M24SR_WRONG_CRC) {
            /* send the FrameExension response*/
            status = _m24sr_fwt_extension (dev, data[M24SR_OFFSET_PCB + 1]); //@FIXME
        }

    }
    else {
        status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
        if (status != SW_OK) {
            //todo Insert DEBUG() 
            return M24SR_ERROR;
        }
    }
    return ret;
}
#endif

#if 0
static int _m24sr_gpo_send_interrupt (const m24sr_t *dev) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data | Le */
    /* 0xA2 | 0xD6 | 0x00 | 0x1E | 0x00 |  -   |  - */
    int ret = M24SR_OK;

    uint8_t     *buffer = data_buffer;
    uint16_t    status ; 
    uint16_t    num_byte;
    
    status = m24sr_manage_i2c_gpo(dev, INTERRUPT);
    
    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_UPDATE_BINARY;
    /* copy the Password Id */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x1E;
    cmd.body.LC = 0x00 ;
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, buffer, &num_byte);

    /* send the request */ 
    ret = m24sr_send_i2c_cmd (dev, buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy (dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */ 
    ret = m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    if (status != SW_OK) {
        //@todo insert debug
        return M24SR_ERROR;
    }

    return ret;
}

static int _m24sr_gpo_state_control (const m24sr_t *dev, uint8_t state) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data | Le */
    /* 0xA2 | 0xD6 | 0x00 | 0x1F | 0x01 | 0x0X |  - */
    /*                                    0x00 - Reset Value (GPO Low) */
    /*                                    0x01 - Set Value   (GPO HZ)*/
    
    int ret = M24SR_OK;
    uint8_t  *buffer = data_buffer;
    uint16_t num_byte;
    uint16_t status;

    /*check the parameters */
    if ((state != 0x01) && (state != 0x00))
    {
        return M24SR_ERROR_PARAM;
    }

    status = m24sr_manage_i2c_gpo(dev, STATE_CONTROL);

    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_UPDATE_BINARY;
    /* copy the Password Id */
    cmd.header.P1 = 0x00;
    cmd.header.P2 = 0x1F;
    cmd.body.LC = 0x01;
    /* copy the data */
    memcpy(cmd.body.data , &state, 0x01);
    /* build the I2C command */
    _m24sr_build_iblock_cmd (M24SR_CMD_CATEGORY_1, cmd, buffer, &num_byte);

    /* send the request */
    ret = m24sr_send_i2c_cmd (dev, buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    ret = m24sr_is_answer_rdy (dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE); //@TODO Return values not error. Return value is code SW
    if (status != SW_OK)
        return M24SR_ERROR;

    return ret;
}
#endif

/**
  * @brief  This function sends the FWT extension command (S-Block format)
    * @param    FWTbyte : FWT value
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
static int _m24sr_fwt_extension(const m24sr_t *dev, uint8_t fwt_byte)
{
    int ret = M24SR_OK;
    uint8_t  *buffer = data_buffer;
    uint16_t status;
    uint16_t num_byte = 0;
    uint16_t crc;

    /* create the response */
    buffer[num_byte++] = 0xF2 ;
    buffer[num_byte++] = fwt_byte ;
    /* compute the CRC */
    crc = _m24sr_compute_crc (buffer, 0x02);
    /* append the CRC16 */
    buffer[num_byte++] = GET_LSB(crc ) ;
    buffer[num_byte++] = GET_MSB(crc ) ;

    /* send the request */
    if (m24sr_send_i2c_cmd (dev, buffer, num_byte) != M24SR_OK)
        return M24SR_NOBUS;
    if(m24sr_is_answer_rdy (dev ) != M24SR_OK)
        return M24SR_NOBUS;
    /* read the response */
    if(m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE) != M24SR_OK)
        return M24SR_NOBUS;

    status = _m24sr_is_correct_crc_residue (buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (status != SW_OK) {
        DEBUG("Return SW code 0x%04X", status);
        return M24SR_ERROR;
    }

    return ret;
}

/**
 * @brief This function sends the GetSession command to the M24SR device
 * 
 * @param[in] dev - Device descriptor of M24SR device to read from
 * @return Сommand result
 */
static int _m24sr_get_i2c_session (const m24sr_t *dev) {
    /* CLA | INS  | P1 | P2 | Lc | Data | Le */
    /*  -  | 0x26 | -  | -  |  - |  --  |  - */
    int ret = M24SR_OK;
    uint8_t buffer = M24SR_OPEN_I2C_SESSION;

    ret = m24sr_send_i2c_cmd(dev, &buffer, 0x01);
    if (ret == M24SR_OK) {
        ret = m24sr_poll_i2c(dev);
    }
    return ret;
}

/**
 * @brief This function sends the KillSession command to the M24SR device
 * 
 * @param[in] dev - Device descriptor of M24SR device to read from
 * @return Сommand result
 */
static int _m24sr_kill_rf_session (const m24sr_t *dev) {
    /* CLA | INS  | P1 | P2 | Lc | Data | Le */
    /*  -  | 0x52 | -  | -  |  - |  --  |  - */
    uint8_t buffer = M24SR_KILL_RF_SESSION;
    int  ret = M24SR_OK;

    ret = m24sr_send_i2c_cmd(dev, &buffer, 0x01);
    /* Insure no access will be done just after open session */
    /* The only way here is to poll I2C to know when M24SR is ready */
    /* GPO can not be use with KillSession command */ 
    if (ret == M24SR_OK) {
        ret = m24sr_poll_i2c(dev);
    }
    return ret;
}


static int _m24sr_open_i2c_session (const m24sr_t *dev, m24sr_priority_t priority) {
    int ret = M24SR_OK;
    uint16_t timeout = 1000;

    if (priority == I2C_KILL_RF) {
        ret = _m24sr_kill_rf_session(dev);
    } else if (priority == I2C_OPEN_SESSION) {
        ret = _m24sr_get_i2c_session(dev);
        while (ret != M24SR_OK && timeout) {
            xtimer_usleep(1 * US_PER_MS);
            ret = _m24sr_get_i2c_session(dev);
            timeout--;
        }
    }
    return ret;
}

void m24sr_close_session(const m24sr_t *dev, m24sr_token_mode_t type) {
    if ((type == I2C_TOKEN_RELEASE_HW) && (dev->params.pwr_en_pin != GPIO_UNDEF)) {
        gpio_clear(dev->params.pwr_en_pin);
    } else if (type == I2C_TOKEN_RELEASE_SW) {
        m24sr_release_i2c_token(dev);
    }
}

////////////////////////////////////////////////////////////////////////////////




/**
  * @brief  This function enable or disable RF communication
  * @param  OnOffChoice: GPO configuration to set
  */
void m24sr_rf_config_hw(const m24sr_t *dev, uint8_t state)
{
    /* Disable RF */
    if ( state != 0 ) {
        gpio_clear(dev->params.rfdisable_pin);
    } else {
        gpio_set(dev->params.rfdisable_pin);
    }
}

#if defined(I2C_GPO_INTERRUPT_ALLOWED)
static void _alert_cb(void *arg) {
    m24sr_t *dev = (m24sr_t *)arg;

    if (dev->cb) {
        dev->cb(dev->arg);
    }
}
#endif

/**
  * @brief  This function initializes the M24SR_I2C interface
  * @retval None
  */
int m24sr_i2c_init_hw (m24sr_t *dev, const m24sr_params_t *params, m24sr_cb_t gpo_cb, void *gpo_cb_arg) {
    int retval = M24SR_OK;

    dev->params = *params;

    /* Configure GPIO pins*/
    if (dev->params.gpo_pin != GPIO_UNDEF) {
#if defined(I2C_GPO_INTERRUPT_ALLOWED)
        dev->cb = gpo_cb;
        dev->arg = gpo_cb_arg; 

        retval = gpio_init_int(dev->params.gpo_pin, GPIO_IN, dev->params.gpo_flank, _alert_cb, dev);
        if (retval < 0) {
            DEBUG("[m24sr] error: failed to initialize GPO pin\n");
            return retval;
        }
#else
        (void)gpo_cb;
        (void)gpo_cb_arg;
        retval = gpio_init(dev->params.gpo_pin, GPIO_IN);
        if (retval < 0) {
            DEBUG("[m24sr] error: failed to initialize GPO pin\n");
            return retval;
        }
#endif
    }
    /* Configure GPIO pins for RF DISABLE */
    if (dev->params.rfdisable_pin != GPIO_UNDEF) {
        retval = gpio_init(dev->params.rfdisable_pin, GPIO_OUT);
        if (retval < 0) {
            DEBUG("[m24sr] error: failed to initialize RF DISABLE pin\n");
            return retval;
        }
    }
    /* Configure GPIO pins for Power Enable*/
    if (dev->params.pwr_en_pin != GPIO_UNDEF) {
        retval = gpio_init(dev->params.pwr_en_pin, GPIO_OUT);
        if (retval < 0){
            DEBUG("[m24sr] error: failed to initialize POWER EN pin\n");
            return retval;
        }
    }

    return retval;  
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  This function configure GPO purpose for I2C session
  * @param    GPO_I2Cconfig: GPO configuration to set
  * @retval Status (SW1&SW2) : Status of the operation to complete.
  */
uint16_t m24sr_manage_i2c_gpo(const m24sr_t *dev, m24sr_gpo_mode_t gpo_i2c_config)
{
    uint16_t status;
    uint8_t gpo_config;
    uint8_t default_pwd[M24SR_PWD_LEN] = {0x00};

    if (gpo_i2c_config > STATE_CONTROL) {
        return M24SR_ERROR_PARAM;
    }

    memset(default_pwd, 0x00, M24SR_PWD_LEN);

    /* we must not be in interrupt mode for I2C synchro as we will change GPO purpose */
    m24sr_set_i2c_synchro_mode(M24SR_WAITING_TIME_POLLING);

    m24sr_select_application(dev);
    m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    // read system file
    m24sr_read_binary (dev, 0x0004, &gpo_config, 0x01);

    /* Update only GPO purpose for I2C */
    gpo_config = (gpo_config & 0xF0) | gpo_i2c_config;
    m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    m24sr_verify(dev, I2C_PWD , 0x10 , default_pwd);
    //write system file
    status = m24sr_update_binary(dev, 0x0004 , &gpo_config, 0x01);

    /* if we have set interrupt mode for I2C synchro we can enable interrupt mode */
    if (gpo_i2c_config == I2C_ANSWER_READY && status == M24SR_OK)
#ifdef I2C_GPO_SYNCHRO_ALLOWED
        m24sr_set_i2c_synchro_mode(M24SR_WAITING_TIME_GPO);
#else
        m24sr_set_i2c_synchro_mode(M24SR_INTERRUPT_GPO);
#endif
    return status;
}


/**
  * @brief  This function configure GPO purpose for RF session
    * @param    GPO_RFconfig: GPO configuration to set
  * @retval Status (SW1&SW2) : Status of the operation to complete.
  */
uint16_t m24sr_manage_rf_gpo(const m24sr_t *dev, m24sr_gpo_mode_t gpo_rf_config)
{
    uint16_t status;
    uint8_t gpo_config;
    uint8_t default_pwd[M24SR_PWD_LEN] = {0x00};

    if (gpo_rf_config > STATE_CONTROL)
    {
        return M24SR_ERROR_PARAM;
    }

    m24sr_select_application(dev);
    m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    m24sr_read_binary(dev, 0x0004, &gpo_config, 0x01);

    /* Update only GPO purpose for I2C */
    gpo_config = (gpo_config & 0x0F) | (gpo_rf_config << 4);
    m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    m24sr_verify(dev, I2C_PWD , M24SR_PWD_LEN , default_pwd);
    status = m24sr_update_binary(dev, 0x0004, &gpo_config, 0x01);

    return status;
}


/**
  * @brief  This function enable or disable RF communication
    * @param    OnOffChoice: GPO configuration to set
  * @retval Status (SW1&SW2) : Status of the operation to complete.
  */
void m24sr_rf_config(const m24sr_t *dev, uint8_t rf_config)
{
    m24sr_rf_config_hw(dev, rf_config);
}



/**
  * @brief  This fonction initialize the M24SR
  * @param  CCBuffer : pointer on the buffer to store CC file
  * @param  size : number of byte of data to read
  * @retval SUCCESS : Initalization done
  * @retval ERROR : Not able to Initialize.
  */
int m24sr_init(m24sr_t *dev, const m24sr_params_t *params, gpio_cb_t gpo_pin_cb, void *gpo_pin_cb_arg) {
    int status = M24SR_OK;
    uint8_t trials = 5;
    cc_file_info_t cc_file;

    /* Perform HW initialization */
    status = m24sr_i2c_init_hw (dev, params, gpo_pin_cb, gpo_pin_cb_arg);
    if (status != M24SR_OK)
        return M24SR_ERROR;

    _m24sr_init_structure();

#if defined(I2C_GPO_SYNCHRO_ALLOWED) || defined(I2C_GPO_INTERRUPT_ALLOWED)
    if (_m24sr_kill_rf_session(dev) == M24SR_OK) {
        m24sr_manage_i2c_gpo(I2C_ANSWER_READY);
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
    }
#endif /* I2C_GPO_SYNCHRO_ALLOWED */

    /* Read CC file */
    while (status != M24SR_OK && trials) {
        status = _m24sr_get_i2c_session(dev);
        trials--;
    }
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /*===================================*/
    /* Select the NFC type 4 application */
    /*===================================*/
    status = m24sr_select_application(dev);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /*==================*/
    /* select a CC file */
    /*==================*/
    status = m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    /* read the first 15 bytes of the CC file */
    if (m24sr_read_data(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file_info_t)) == M24SR_OK) {
        
        dev->memory.chipsize = (cc_file.ndef_file_max_size - NDEF_FILE_LEN_NUM_BYTES);
        dev->memory.max_read_byte = cc_file.max_read_byte;
        dev->memory.max_write_byte = cc_file.max_write_byte;

        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_OK;
    } else {
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_OK;
    }
    return status; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  This fonction read the data stored in M24SR at defined offset
  * @param  Offset : Offset in the NDEF file in M24SR
  * @param  DataSize : Number of byte to read
  * @param  pData : pointer on buffer to store read data
  * @retval Status (SW1&SW2) : Status of the operation.
  */
uint16_t m24sr_read_data (const m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size) {
    uint16_t status;
    uint16_t max_read_byte = dev->memory.max_read_byte;

    if (size > max_read_byte) {
        do {
            status = m24sr_read_binary(dev, offset, dst, max_read_byte);
            offset += max_read_byte;
            dst += max_read_byte;
            size -= max_read_byte;
        } while (size > max_read_byte && status == M24SR_OK);
        
        if (status == M24SR_OK && size) {
            status = m24sr_read_binary (dev, offset, dst, (uint8_t)(size));
        }
    } else {
        status = m24sr_read_binary (dev, offset, dst, (uint8_t)(size));
    }

    return status;
}

/**
  * @brief  This fonction read the data stored in M24SR at defined offset without NDEF concideration
  * @param  Offset : Offset in the NDEF file in M24SR
  * @param  DataSize : Number of byte to read
  * @param  pData : pointer on buffer to store read data
  * @retval Status (SW1&SW2) : Status of the operation.
  */
uint16_t m24sr_force_read_data(const m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size) {
    uint16_t status;
    uint16_t max_read_byte = dev->memory.max_read_byte;

    if (size > max_read_byte) {
        do {
            status = m24sr_extended_read_binary (dev, offset, dst, max_read_byte);
            offset += max_read_byte;
            dst += max_read_byte;
            size -= max_read_byte;
        } while (size > max_read_byte && status == M24SR_OK);
        
        if (status == M24SR_OK && size) {
            status = m24sr_extended_read_binary (dev, offset, dst, (uint8_t)(size));
        }
    } else {
        status = m24sr_extended_read_binary (dev, offset, dst, (uint8_t)(size));
    }

    return status;
}




uint16_t m24sr_write_data (const m24sr_t *dev, uint16_t offset, uint8_t *src, uint16_t size)
{
    uint16_t status;
    uint16_t max_write_byte = dev->memory.max_write_byte;

    if (size > max_write_byte) {
        do {
            status = m24sr_update_binary(dev, offset, src, max_write_byte);
            offset += max_write_byte;
            src += max_write_byte;
            size -= max_write_byte;
        } while (size > max_write_byte && status == M24SR_OK);
        
        if (status == M24SR_OK && size) {
            status = m24sr_update_binary (dev, offset, src, (uint8_t)(size));
        }
    } else {
        status = m24sr_update_binary (dev, offset, src, (uint8_t)(size));
    }

    return status;
}
/**
  * @brief  This function configure GPO purpose for RF session
  * @param  GPO_config: GPO configuration to set
  * @param  mode: select RF or I2C, GPO config to update
  * @retval Status : Status of the operation.
  */
uint16_t m24sr_manage_gpo(const m24sr_t *dev, m24sr_gpo_mode_t gpo_config, uint8_t mode) {
    uint16_t status;

    if (mode == RF_GPO) {
        status = m24sr_manage_rf_gpo(dev, gpo_config);
    } else {
        status = m24sr_manage_i2c_gpo(dev, gpo_config);
    }
    return status;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//This global function for emmulatiom I2C eeprom
/////////////////////////////////////////////////////////////////////////////////////////////////////////





int m24sr_eeprom_init(m24sr_t *dev, const m24sr_params_t *params, m24sr_cb_t cb, void *arg) {
    int ret = M24SR_OK;

    ret = m24sr_init(dev, params, cb, arg);

    return ret;
}


int m24sr_eeprom_read(m24sr_t *dev, void *dst, uint32_t addr, uint32_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    uint16_t ndef_file_size = 0x0000;
    int status = M24SR_OK;

    if (addr > total_size) {
        return -EOVERFLOW;
    }
    if ((addr + size) > total_size) {
        size = total_size - addr;
    }
    if (size == 0) {
        return 0;
    }

    status = _m24sr_open_i2c_session(dev, dev->params.priority);      
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /* Select the NFC type 4 application */ 
    status = m24sr_select_application(dev);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /**/
    m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file_info_t));
    
    ndef_file_id = cc_file.ndef_file_id;
    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    if (size < ndef_file_size) {
        status = m24sr_read_data(dev, NDEF_FILE_LEN_POS + addr, dst, size + NDEF_FILE_LEN_NUM_BYTES);
    } else {
        return M24SR_ERROR;
    }
    
    if(status == M24SR_OK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);
    // if (status != M24SR_OK) {
    //     return M24SR_ERROR;
    // }

    return size;
}

int m24sr_eeprom_write(m24sr_t *dev, void *src, uint32_t addr, uint32_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    uint16_t ndef_file_size = 0x0000;
    int status = M24SR_OK;

    
    if (addr > total_size) {
        return -EOVERFLOW;
    }
    if (addr + size > total_size) {
        return -EOVERFLOW;
    }
    if (size == 0) {
        return 0;
    }


    status = _m24sr_open_i2c_session(dev, dev->params.priority);      
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /* Select the NFC type 4 application */ 
    status = m24sr_select_application(dev);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /**/
    m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file_info_t));
    
    ndef_file_id = cc_file.ndef_file_id;
    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    if (size < ndef_file_size) {
        status = m24sr_write_data(dev, NDEF_FILE_LEN_POS + addr, src, size + NDEF_FILE_LEN_NUM_BYTES);
    } else {
        return M24SR_ERROR;
    }
    
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);



    return size;
}

int m24sr_eeprom_erase(m24sr_t *dev, uint32_t addr, uint32_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    uint16_t ndef_file_size = 0x0000;
    int status = M24SR_OK;
    uint8_t src[0xFF]={0x00};

    if (addr > total_size) {
        return -EOVERFLOW;
    }
    if (addr + size > total_size) {
        return -EOVERFLOW;
    }

    status = _m24sr_open_i2c_session(dev, dev->params.priority);      
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /* Select the NFC type 4 application */ 
    status = m24sr_select_application(dev);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    /**/
    m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file_info_t));
    
    ndef_file_id = cc_file.ndef_file_id;
    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    if (size < ndef_file_size) {
        status = m24sr_write_data(dev, NDEF_FILE_LEN_POS + addr, src, size + NDEF_FILE_LEN_NUM_BYTES);
    } else {
        return M24SR_ERROR;
    }
    
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);


    return 0;
}

int m24sr_eeprom_power(m24sr_t *dev, uint8_t power) {

    (void)dev;
    (void)power;

    return 0;
}
