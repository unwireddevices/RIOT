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

#include "byteorder.h"

#define ENABLE_DEBUG                    (0)
#include "debug.h"


#if ENABLE_DEBUG
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        while (len) {
            len--;
            printf("%02X ", *buff++);
        }
        printf("\n");
    }
#else
    #define PRINTBUFF(...)
#endif

static cmd_apdu_t cmd;
static uint8_t data_buffer[0xFF];
static uint8_t cmd_data[0xFF];
static uint8_t device_id_byte = 0x00;

/**
  * @brief  Initialize the command and response structure
  * 
  * @param  None
  * 
  * @return None
  */
static void _m24sr_init_structure (void);

/**
 * @brief this functions configure I2C synchronization mode
 * 
 * @param[in] dev  Pointer to M24SR NFC eeprom device descriptor
 * @param[in] mode Type synchronization mechanism
 * 
 * @return None
 */
static void m24sr_set_i2c_synchro_mode(m24sr_t *dev, m24sr_wait_mode_t mode);

/**
 * @brief  This function updates the CRC
 * 
 * @param[in]  ch  One bytes data
 * @param[out] crc CRC16
 * 
 * @return [description]
 */
static uint16_t _m24sr_update_crc (uint8_t ch, uint16_t *crc);

/**
 * @brief  This function returns the CRC16-CCITT
 * 
 * @param[in] data   Pointer to the data 
 * @param[in] length Number of bytes data
 * 
 * @return CRC16-CCITT
 */
static uint16_t _m24sr_compute_crc (uint8_t *data, uint8_t length);

/**
 * @brief This function computes the CRC16 residue as defined by CRC ISO/IEC 13239
 * 
 * @param[in] data   Pointer to the input data
 * @param[in] length Number of bytes data
 * 
 * @return Status (SW1&SW2) (CRC16 residue is correct) or error wrong crc (CRC16 residue is false)
 */
static int _m24sr_is_correct_crc_residue (uint8_t *data, uint8_t length);

/**
 * @brief  This functions creates an I-block command according to the structures CommandStructure and Command.
 * 
 * @param[in]  category Block formation parameters
 * @param[in]  cmd      Structue which contains the field of the different parameter
 * @param[out] iblock   Pointer of the block created
 * @param[out] numByte  Number of byte of the block
 * 
 * @return None
 */
static void _m24sr_build_iblock_cmd (uint16_t category, cmd_apdu_t cmd, uint8_t *iblock, uint16_t *numByte);

#if 0
/**
 * @brief This function return M24SR_OK if the buffer is an I-block
 * 
 * @param[in] buffer Pointer of the data
 *
 * @return Error code
 */
static int _is_iblock(uint8_t *buffer);

/**
 * @brief This function return M24SR_OK if the buffer is an R-block
 * 
 * @param[in] buffer Pointer of the data
 *
 * @return Error code
 */
static int _is_rblock(uint8_t *buffer);
#endif

/**
 * @brief This function return M24SR_OK if the buffer is an S-block
 * 
 * @param[in] buffer Pointer of the data
 * 
 * @return Error code
 */
static int _is_sblock(uint8_t *buffer);


/**
 * @brief This function sends the SelectApplication command
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * 
 * @return Error code
 */
static int m24sr_select_application(m24sr_t *dev);

/**
 * @brief This function sends the SelectCCFile command
 * 
 * @param[in] dev        Pointer to M24SR NFC eeprom device descriptor
 * @param[in] cc_file_id ID capabilitycontainer file
 * 
 * @return Error code
 */
static int m24sr_select_capability_container_file(m24sr_t *dev, uint16_t cc_file_id);

/**
 * @brief  This function sends the SelectNDEFfile command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] ndef_file_id ID NDEF file
 * 
 * @return Error code
 */
static int m24sr_select_ndef_file(m24sr_t *dev, uint16_t ndef_file_id);

/**
 * @brief This function sends the SelectSystemFile command
 * 
 * @param[in] dev         Pointer to M24SR NFC eeprom device descriptor
 * @param[in] sys_file_id ID system file
 * 
 * @return Error code
 */
static int m24sr_select_system_file(m24sr_t *dev, uint16_t sys_file_id);

/**
 * @brief This function sends a ReadBinary command
 * 
 * @param[in]  dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  offset   First byte to read
 * @param[out] dst_data Pointer of the buffer read from the M24SR device
 * @param[in]  len      Number of byte to read
 * 
 * @return Error code
 */
static int m24sr_read_binary(m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint16_t len);

/**
 * @brief This function sends a UpdateBinary command
 * 
 * @param[in] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in] offset   First byte to write
 * @param[in] src_data Pointer of the buffer write to the M24SR device
 * @param[in] len      Number of byte to write
 * 
 * @return Error code
 */
static int m24sr_update_binary(m24sr_t *dev, uint16_t offset, uint8_t *src_data, uint16_t len);

/**
 * @brief This function sends the Verify command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] pwd_id       PasswordId ( 0x0001 : Read NDEF pwd or 0x0002 : Write NDEF pwd or 0x0003 : I2C pwd)
 * @param[in] num_pwd_byte Number of byte ( 0x00 or 0x10)
 * @param[in] pwd          Pointer on the password
 * 
 * @return Error code
 */
static int m24sr_verify(m24sr_t *dev, uint16_t pwd_id, uint8_t num_pwd_byte , uint8_t *pwd);

/**
 * @brief  This function sends a ST ReadBinary command (no error if access is not inside NDEF file)
 * 
 * @param[in] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in] offset   First byte to read
 * @param[in] dst_data Pointer of the buffer read from the M24SR device
 * @param[in] len      Number of byte to read
 * @return Error code
 */
static int m24sr_extended_read_binary(m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint32_t len);

#if 0 //RFU
/**
 * @brief [brief description]
 * 
 * @param[in] dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in] pwd_id PasswordId ( 0x0001 : Read NDEF pwd or 0x0002 : Write NDEF pwd or 0x0003 : I2C pwd)
 * @param[in] pwd    Pointer on the password
 *
 * @return Error code
 */
static int m24sr_change_reference_data(m24sr_t *dev, uint16_t pwd_id, uint8_t *pwd); 

/**
 * @brief This function sends the EnableVerificationRequirement command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] mode_protect Enable the read or write protection ( 0x0001 : Read or 0x0002 : Write)
 * 
 * @return Error code
 */
static int m24sr_enable_verification_requirement(m24sr_t *dev, uint16_t mode_protect);

/**
 * @brief This function sends the DisableVerificationRequirement command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] mode_protect Disable the read or write protection ( 0x0001 : Read or 0x0002 : Write)
 * 
 * @return Error code
 */
static int m24sr_disable_verification_requirement(m24sr_t dev, uint16_t mode_protect);

/**
 * @brief This function sends the EnablePermananentState command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] mode_protect Enable the read or write protection ( 0x0001 : Read or 0x0002 : Write)
 * 
 * @return Error code
 */
static int m24sr_enable_permanent_state(m24sr_t *dev, uint16_t mode_protect);

/**
 * @brief This function sends the DisablePermanentState command
 * 
 * @param[in] dev          Pointer to M24SR NFC eeprom device descriptor
 * @param[in] mode_protect Disable the read or write protection ( 0x0001 : Read or 0x0002 : Write)
 * 
 * @return Error code
 */
static int m24sr_disable_permanent_state(m24sr_t *dev, uint16_t mode_protect)
#endif

#if defined(M24SR_UPDATE_FILE_TYPE)
/**
 * @brief This function sends the UpdateFileType command
 * 
 * @param[in] dev       Pointer to M24SR NFC eeprom device descriptor
 * @param[in] file_type File type (0x05 : proprietary file or 0x04 : NDEF file)
 * 
 * @return Error code
 */
static int m24sr_update_file_type (m24sr_t *dev, uint8_t file_type); 
#endif

#if 0
/**
 * @brief This function generates a interrupt on GPO pin
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 *
 * @return Error code
 */
static int _m24sr_gpo_send_interrupt (m24sr_t *dev);

/**
 * @brief This function force GPO pin to low state or high Z
 * 
 * @param[in] dev   Pointer to M24SR NFC eeprom device descriptor
 * @param[in] state Select if GPO must be low (reset) or HiZ
 * 
 * @return Error code
 */
static int _m24sr_gpo_state_control (m24sr_t *dev, uint8_t state)
#endif

/**
 * @brief This function configure GPO purpose for I2C session
 * 
 * @param[in] dev            Pointer to M24SR NFC eeprom device descriptor
 * @param[in] gpo_i2c_config GPO configuration to set
 * 
 * @return Error code
 */
static uint16_t m24sr_manage_i2c_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_i2c_config);

/**
 * @brief This function configure GPO purpose for RF session
 * 
 * @param[in] dev            Pointer to M24SR NFC eeprom device descriptor
 * @param[in] gpo_i2c_config GPO configuration to set
 * 
 * @return Error code
 */
static uint16_t m24sr_manage_rf_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_rf_config);

/**
 * @brief This function sends the FWT extension command (S-Block format)
 * 
 * @param[in] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in] fwt_byte FWT value
 * 
 * @return Error code
 */
static int _m24sr_fwt_extension(m24sr_t *dev, uint8_t fwt_byte);

/**
 * @brief This function sends the GetSession command to the M24SR device
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * 
 * @return Error code
 */
static int _m24sr_get_i2c_session (const m24sr_t *dev);


/**
 * @brief This function sends the KillSession command to the M24SR device
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * 
 * @return Error code
 */
static int _m24sr_kill_rf_session (const m24sr_t *dev); 

/**
 * @brief This function configure the M24SR to access I2C
 * 
 * @param[in] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in] priority Type open session
 * 
 * @return Error code
 */
static int _m24sr_open_i2c_session (const m24sr_t *dev, m24sr_priority_t priority); 


/**
 * @brief This function close session
 * 
 * @param[in] dev  Pointer to M24SR NFC eeprom device descriptor
 * @param[in] type Type token release
 * 
 * @return None
 */
static void m24sr_close_session(const m24sr_t *dev, m24sr_token_mode_t type);


#if 0
/**
 * @brief This function enable or disable RF communication
 * 
 * @param dev   Pointer to M24SR NFC eeprom device descriptor
 * @param state GPO configuration to set
 * 
 * @retutrn None
 */
static void m24sr_rf_config_hw(const m24sr_t *dev, uint8_t state);
#endif

/**
 * @briefThis function initializes the M24SR I2C interface
 * 
 * @param[out] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  params   Pointer to static device configuration
 * 
 * @return  Error code
 */
static int m24sr_i2c_init_hw (m24sr_t *dev, const m24sr_params_t *params);

/**
 * @brief This function initialize the M24SR
 * 
 * @param[out] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  params   Pointer to static device configuration
 * 
 * @return  Error code
 */
int m24sr_init(m24sr_t *dev, const m24sr_params_t *params);


/**
  * @brief  This fonction read the data stored in M24SR at defined offset
  * @param  Offset : Offset in the NDEF file in M24SR
  * @param  DataSize : Number of byte to read
  * @param  pData : pointer on buffer to store read data
  * @retval Status (SW1&SW2) : Status of the operation.
  */


/**
 * @brief This function read the data stored in M24SR at defined offset
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  offset Offset in the NDEF file in M24SR
 * @param[out] dst    Pointer on buffer to store read data
 * @param[in]  size   Number of byte to read
 * 
 * @return  Error code
 */
static uint16_t m24sr_read_data (m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size);

/**
 * @brief  This function read the data stored in M24SR at defined offset without NDEF concideration
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  offset Offset in the NDEF file in M24SR
 * @param[out] dst    Pointer on buffer to store read data
 * @param[in]  size   Number of byte to read
 * 
 * @return  Error code
 */
static uint16_t m24sr_force_read_data(m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size);

/**
 * @brief This function write data in M24SR at defined offset
 * 
 * @param[in] dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in] offset Offset in the NDEF file in M24SR
 * @param[in] src    Pointer on buffer to copy in M24SR
 * @param[in] size   Number of byte to write
 * 
 * @return  Error code
 */
static uint16_t m24sr_write_data(m24sr_t *dev, uint16_t offset, uint8_t *src, uint16_t size);

/**
 * @brief This function configure GPO purpose for RF session
 * 
 * @param[in] dev        Pointer to M24SR NFC eeprom device descriptor
 * @param[in] gpo_config GPO configuration to set
 * @param[in] mode       Select RF or I2C, GPO config to update
 * 
 * @return  Error code
 */
static uint16_t m24sr_manage_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_config, uint8_t mode);



/***************************** Implementation private function *****************************************/
static void m24sr_set_i2c_synchro_mode(m24sr_t *dev, m24sr_wait_mode_t mode) {
#if defined(I2C_GPO_SYNCHRO_ALLOWED) || defined(I2C_GPO_INTERRUPT_ALLOWED)
    dev->synchro_mode = mode;
#else
    if (mode == M24SR_WAITING_TIME_GPO || mode == M24SR_INTERRUPT_GPO)
        dev->synchro_mode = M24SR_WAITING_TIME_POLLING;
    else
        dev->synchro_mode = mode;
#endif /*  I2C_GPO_SYNCHRO_ALLOWED */
}

static void _m24sr_init_structure (void) {
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

static uint16_t _m24sr_update_crc (uint8_t ch, uint16_t *crc) {
    ch = (ch ^ (uint8_t)((*crc) & 0x00FF));
    ch = (ch ^ (ch << 4));
    *crc = (*crc >> 8) ^ ((uint16_t)ch << 8) ^ ((uint16_t)ch << 3) ^ ((uint16_t)ch >> 4);

    return (*crc);
}

static uint16_t _m24sr_compute_crc (uint8_t *data, uint8_t length) {
    uint8_t block;
    uint16_t crc;

    crc = 0x6363; // ITU-V.41

    do {

        block = *data++;
        _m24sr_update_crc(block, &crc);
    } while (--length);

    return crc ;
}

static int _m24sr_is_correct_crc_residue (uint8_t *data, uint8_t length) {
    uint16_t res_crc = 0;

    /* check the CRC16 Residue */
    if (length != 0)
        res_crc = _m24sr_compute_crc (data, length);

    if (res_crc == 0x0000) {
        /* Good CRC, but error status from M24SR */
        return (uint16_t)(((data[length - UB_STATUS_OFFSET] << 8) & 0xFF00) | (data[length - LB_STATUS_OFFSET] & 0x00FF));
    } else {
        res_crc = 0;
        res_crc = _m24sr_compute_crc (data, M24SR_STATUS_RESPONSE_NUM_BYTE);
        if (res_crc != 0x0000) {
            /* Bad CRC */
            return M24SR_WRONG_CRC;
        } else {
            /* Good CRC, but error status from M24SR */
            return (uint16_t)(((data[1] << 8) & 0xFF00) | (data[2] & 0x00FF));
        }
    }
}

static void _m24sr_build_iblock_cmd (uint16_t category, cmd_apdu_t cmd, uint8_t *iblock, uint16_t *numByte) {
    uint16_t crc16;
    static uint8_t block_num = 0x01;

    (*numByte) = 0;

    /* add the PCD byte */
    if ((category & M24SR_PCB_NEEDED) != 0) {
        /* toggle the block number */
        block_num = TOGGLE (block_num);
        /* Add the I-block byte */
        iblock[(*numByte)++] = 0x02|block_num;
    }
    /* add the DID byte */
    if ((block_num & M24SR_DEV_ID_NEEDED) != 0) {
        /* Add the I-block byte */
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
static int _is_iblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_IBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}

static int _is_rblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_RBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}
#endif

static int _is_sblock (uint8_t *buffer) {
    if ((buffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_SBLOCK) {
        return M24SR_OK;
    } else {
        return M24SR_ERROR;
    }
}


/******************************************** Standart cmd set *****************************************/
static int m24sr_select_application(m24sr_t *dev) {
    /* CLA  | INS  |  P1  |  P2  |  Lc  |  Data            | Le   */
    /* 0x00 | 0xA4 | 0x04 | 0x00 | 0x07 | 0xD2760000850101 | 0x00 */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;
    

    const uint8_t buffer[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};

    DEBUG("\n");

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
    /* waiting for answer */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        DEBUG("Answer not ready\n");
        return M24SR_NOBUS;
    }

    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        DEBUG("I2C no response\n");
        return M24SR_NOBUS;
    }
    /* check crc*/
    status = _m24sr_is_correct_crc_residue(data, num_byte_read);
    DEBUG ("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_select_capability_container_file(m24sr_t *dev, uint16_t cc_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0xE103 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;
    
    DEBUG("\n");

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
    /* waiting for answer */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }

    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /*check crc*/
    status = _m24sr_is_correct_crc_residue(data, num_byte_read);
    DEBUG("Return SW code 0x%04X\n", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_select_ndef_file(m24sr_t *dev, uint16_t ndef_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0x0001 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;

    DEBUG("\n");

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
    /* waiting for answer */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }

    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /*check crc*/
    status = _m24sr_is_correct_crc_residue(data, num_byte_read);
    DEBUG("Return SW code 0x%04X\n", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_select_system_file(m24sr_t *dev, uint16_t sys_file_id) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data   | Le */
    /* 0x00 | 0xA4 | 0x00 | 0x0C | 0x02 | 0xE101 | -  */
    int ret = M24SR_OK;
    uint8_t num_byte_read = M24SR_STATUS_RESPONSE_NUM_BYTE;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len;

    DEBUG("\n");

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
    /* waiting for answer */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, num_byte_read);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check crc */
    status = _m24sr_is_correct_crc_residue(data, num_byte_read); 
    DEBUG("Return SW code 0x%04X\n", status);
    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

static int m24sr_read_binary(m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint16_t len) {
    /* CLA  | INS  | P1  P2 | Lc  | Data | Le                        */
    /* 0x00 | 0xB0 | Offset |  -  | -    | NubBytes (0x01 =< Le =< 0xF6) */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *data = data_buffer;

    DEBUG("\n");
    DEBUG("Offset is 0x%04X. Length is %d(0x%04X)\n", offset, len, len);

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
    /* waiting for answer */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, data, len + M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check crc */
    status = _m24sr_is_correct_crc_residue(data, len + M24SR_STATUS_RESPONSE_NUM_BYTE);
    memcpy(dst_data, &data[1], len);
    DEBUG("Return SW code 0x%04X\n", status);    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}


static int m24sr_update_binary(m24sr_t *dev, uint16_t offset, uint8_t *src_data, uint16_t len) {
    /* CLA  | INS  | P1  P2 | Lc                            | Data          | Le */
    /* 0x00 | 0xD6 | Offset | NubBytes (0x01 =< Le =< 0xF6) | Data Lc Bytes |  - */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;

    uint8_t *data = data_buffer;

    DEBUG("\n");
    DEBUG("Offset is 0x%04X. Length is %d(0x%04X)\n", offset, len, len);
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
    /* waiting for answer */
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
            status = _m24sr_fwt_extension(dev, data[M24SR_OFFSET_PCB + 1]);
        }
    } else {
        /* check crc */
        status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
        DEBUG("Return SW code 0x%04X\n", status);    
        if (status != SW_OK) {
            return M24SR_ERROR;
        }
    }    
    return ret;
}


static int m24sr_verify(m24sr_t *dev, uint16_t pwd_id, uint8_t num_pwd_byte , uint8_t *pwd) {
    /* CLA  | INS  | P1 P2  | Lc   | Data         | Le */
    /* 0x00 | 0x20 | 0x000X | 0x10 | PWD Lc Bytes | -  */
    /*               0x0001 - Read NDEF PWD XMIT       */
    /*               0x0002 - Write NDEF PWD XMIT      */
    /*               0x0003 - I2C PWD XMIT             */
    int ret = M24SR_OK;
    uint16_t status;

    uint8_t *data = data_buffer;
    uint16_t len = 0;


    DEBUG("\n");

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
    /* check crc */
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    
    return ret;
}

#if 0
static int m24sr_change_reference_data(m24sr_t *dev, uint16_t pwd_id, uint8_t *pwd) {
    /* CLA  | INS  | P1 P2  | Lc   | Data         | Le */
    /* 0x00 | 0x24 | 0x000X | 0xX0 | PWD Lc Bytes | -  */
    /*               0x0001 - Read NDEF PWD XMIT       */
    /*               0x0002 - Write NDEF PWD XMIT      */
    /*               0x0003 - I2C PWD XMIT             */
    /*                        0x00 - No PWD            */
    /*                        0x10 - PWD Enable        */
    
    int ret = M24SR_OK;
    uint16_t status;
    
    uint8_t   *data = data_buffer;
    uint16_t  len = 0;

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
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_1, cmd, data, &len);
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
    /*check CRC and response code*/
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    
    return ret;
}

static int m24sr_enable_verification_requirement(m24sr_t *dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0x00 | 0x28 | 0x000X | -  | -    | -  */
    /*               0x0001 - ENA Read Protect NDEF  */
    /*               0x0002 - ENA Write Protect NDEF */
    
    int ret = M24SR_OK;
    uint16_t status;

    uint8_t   *data = data_buffer;
    uint16_t  len = 0;

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
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, data, &len);
    /* send the request */
    ret = m24sr_send_i2c_cmd(data, len);
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
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    
    return ret;
}


static int m24sr_disable_verification_requirement(m24sr_t dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0x00 | 0x26 | 0x000X | -  | -    | -  */
    /*               0x0001 - DIS Read Protect NDEF  */
    /*               0x0002 - DIS Write Protect NDEF */
    
    int ret = M24SR_OK;
    uint16_t status;

    uint8_t   *data = data_buffer;
    uint16_t  len = 0;

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
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, data, &len);
    /* send request */
    ret = m24sr_send_i2c_cmd(data, len);
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
    status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    
    return ret;
}
#endif



/******************************************** Proprietary STM cmd set *****************************************/
static int m24sr_extended_read_binary(m24sr_t *dev, uint16_t offset, uint8_t *dst_data, uint32_t len) {
    /* CLA  | INS  | P1 P2  | Lc | Data  | Le */
    /* 0xA2 | 0xB0 | Offset |  - |   -   | Len  */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *buffer = data_buffer;

    DEBUG("\n");

    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_READ_BINARY;
    /* copy the offset */
    cmd.header.P1 = GET_MSB(offset);
    cmd.header.P2 = GET_LSB(offset);
    /* copy the number of byte to read */
    cmd.body.LE = len;
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_2,  cmd, buffer, &num_byte);
    /* send request */
    ret = m24sr_send_i2c_cmd(dev, buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, buffer, len + M24SR_STATUS_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check CRC */
    status = _m24sr_is_correct_crc_residue(buffer, len + M24SR_STATUS_RESPONSE_NUM_BYTE); 
    memcpy(dst_data, buffer, len);
    DEBUG("Return SW code 0x%04X\n", status);    
    if (status != SW_OK) {
        return M24SR_ERROR;
    }
    return ret;
}

#if 0
static int m24sr_enable_permanent_state(m24sr_t *dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0xA2 | 0x28 | 0x000X | -  | -    | -  */
    /*               0x0001 - ENA Read Protect NDEF  */
    /*               0x0002 - ENA Write Protect NDEF */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *buffer = data_buffer;

    /* check the parameters */
    if ((mode_protect != 0x0001) && (mode_protect != 0x0002)) {
        return M24SR_ERROR_PARAM;
    }
    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_ENABLE_VERIFY_REQ;
    /* cope the password id*/
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, buffer, &num_byte);
    /* send the request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check CRC */
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }

    return ret;
}


static int m24sr_disable_permanent_state(m24sr_t *dev, uint16_t mode_protect) {
    /* CLA  | INS  | P1 P2  | Lc | Data | Le */
    /* 0xA2 | 0x26 | 0x000X | -  | -    | -  */
    /*               0x0001 - DIS Read Protect NDEF  */
    /*               0x0002 - DIS Write Protect NDEF */
    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;
    uint8_t *buffer = data_buffer;

    /* check the parameters */
    if ((mode_protect != 0x0001) && (mode_protect != 0x0002)) {
        return M24SR_ERROR_PARAM;
    }
    /* build the command */
    cmd.header.CLA = CLA_STM;
    cmd.header.INS = INS_DISABLE_VERIFY_REQ;
    /* cope the password id*/
    cmd.header.P1 = GET_MSB(mode_protect);
    cmd.header.P2 = GET_LSB(mode_protect);
    /* build the I2C command */
    _m24sr_build_iblock_cmd(M24SR_CMD_CATEGORY_4, cmd, buffer, &num_byte);
    /* send the request */
    ret = m24sr_send_i2c_cmd(buffer, num_byte);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy(dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response(dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check CRC */
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }

    return ret;
}
#endif


#if defined(M24SR_UPDATE_FILE_TYPE)
static int m24sr_update_file_type (m24sr_t *dev, uint8_t file_type) {
    /* CLA  | INS  | P1   | P2   | Lc   | Data | Le */
    /* 0xA2 | 0xD6 | 0x00 | 0x00 | 0x01 | 0x0X |  - */
    /*                                    0x04 - Type NDEF        */
    /*                                    0x05 - Type Proprietary */

    int ret = M24SR_OK;
    uint16_t num_byte;
    uint16_t status;

    uint8_t *data = data_buffer;

    DEBUG("\n");

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
    /* if the response is a Watiting frame extenstion request */
    if (_is_sblock(data) == M24SR_OK) {
        /*check the CRC */
        if ( _m24sr_is_correct_crc_residue(data, M24SR_WATING_TIME_EXT_RESPONSE_NUM_BYTE) != M24SR_WRONG_CRC) {
            /* send the FrameExension response*/
            status = _m24sr_fwt_extension (dev, data[M24SR_OFFSET_PCB + 1]);
        }

    }
    else {
        /* check CRC */
        status = _m24sr_is_correct_crc_residue(data, M24SR_STATUS_RESPONSE_NUM_BYTE);
        DEBUG("Return SW code 0x%04X\n", status);
        if (status != SW_OK) {
            return M24SR_ERROR;
        }
    }
    return ret;
}
#endif

#if 0
static int _m24sr_gpo_send_interrupt (m24sr_t *dev) {
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
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy (dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */ 
    ret = m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check CRC */
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE); 
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }

    return ret;
}

static int _m24sr_gpo_state_control (m24sr_t *dev, uint8_t state) {
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
    /* wait for answer ready */
    ret = m24sr_is_answer_rdy (dev);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* read the response */
    ret = m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    if (ret != M24SR_OK) {
        return M24SR_NOBUS;
    }
    /* check CRC */
    status = _m24sr_is_correct_crc_residue(buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK)
        return M24SR_ERROR;

    return ret;
}
#endif

static int _m24sr_fwt_extension(m24sr_t *dev, uint8_t fwt_byte)
{
    int ret = M24SR_OK;
    uint8_t  *buffer = data_buffer;
    uint16_t status;
    uint16_t num_byte = 0;
    uint16_t crc;

    DEBUG("\n");

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
    /* waiting for answer */
    if(m24sr_is_answer_rdy (dev ) != M24SR_OK)
        return M24SR_NOBUS;
    /* read the response */
    if(m24sr_rcv_i2c_response (dev, buffer, M24SR_STATUS_RESPONSE_NUM_BYTE) != M24SR_OK)
        return M24SR_NOBUS;
    /* check CRC*/
    status = _m24sr_is_correct_crc_residue (buffer, M24SR_STATUS_RESPONSE_NUM_BYTE);
    DEBUG("Return SW code 0x%04X\n", status);
    if (status != SW_OK) {
        return M24SR_ERROR;
    }

    return ret;
}

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

    DEBUG("Open session as %s\n", (priority == I2C_KILL_RF)?"kill rf":"get I2C");

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

static void m24sr_close_session(const m24sr_t *dev, m24sr_token_mode_t type) {
    DEBUG("Close session %s\n", (type == I2C_TOKEN_RELEASE_SW)?"software":"not software");
    if ((type == I2C_TOKEN_RELEASE_HW) && (dev->params.pwr_en_pin != GPIO_UNDEF)) {
        gpio_clear(dev->params.pwr_en_pin);
    } else if (type == I2C_TOKEN_RELEASE_SW) {
        m24sr_release_i2c_token(dev);
    }
}

#if 0
static void m24sr_rf_config_hw(const m24sr_t *dev, uint8_t state)
{
    /* Disable RF */
    if ( state != 0 ) {
        gpio_clear(dev->params.rfdisable_pin);
    } else {
        gpio_set(dev->params.rfdisable_pin);
    }
}
#endif

#if defined(I2C_GPO_INTERRUPT_ALLOWED)
static void _irq_handler(void *arg) {
    m24sr_t *dev = (m24sr_t *)arg;
    dev->event_ready = 1;
}
#endif

static int m24sr_i2c_init_hw (m24sr_t *dev, const m24sr_params_t *params) {
    int retval = M24SR_OK;

    dev->params = *params;

    /* Configure GPIO pins*/
    if (dev->params.gpo_pin != GPIO_UNDEF) {
#if defined(I2C_GPO_INTERRUPT_ALLOWED)

        retval = gpio_init_int(dev->params.gpo_pin, GPIO_IN_PU, GPIO_FALLING, _irq_handler, dev);
        if (retval < 0) {
            DEBUG("[m24sr] ERROR: failed to initialize GPO pin\n");
            DEBUG("[m24sr] ERROR: Interrupt pin not initialized\n");
            return retval;
        }
#else
        retval = gpio_init(dev->params.gpo_pin, GPIO_IN_PU);
        if (retval < 0) {
            DEBUG("[m24sr] ERROR: failed to initialize GPO pin\n");
            return retval;
        }
#endif
    }
    /* Configure GPIO pins for RF DISABLE */
    if (dev->params.rfdisable_pin != GPIO_UNDEF) {
        retval = gpio_init(dev->params.rfdisable_pin, GPIO_OUT);
        if (retval < 0) {
            DEBUG("[m24sr] ERROR: failed to initialize RF DISABLE pin\n");
            return retval;
        }
    }
    /* Configure GPIO pins for Power Enable*/
    if (dev->params.pwr_en_pin != GPIO_UNDEF) {
        retval = gpio_init(dev->params.pwr_en_pin, GPIO_OUT);
        if (retval < 0){
            DEBUG("[m24sr] ERROR: failed to initialize POWER EN pin\n");
            return retval;
        }
    }

    return retval;  
}

static uint16_t m24sr_manage_i2c_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_i2c_config)
{
    uint16_t status = M24SR_OK;
    uint8_t gpo_config = 0x00;
    uint8_t default_pwd[M24SR_PASSWORD_NUM_BYTE] = {0x00};

    if (gpo_i2c_config > STATE_CONTROL) {
        return M24SR_ERROR_PARAM;
    }

    memset(default_pwd, 0x00, M24SR_PASSWORD_NUM_BYTE);

    /* we must not be in interrupt mode for I2C synchro as we will change GPO purpose */
    m24sr_set_i2c_synchro_mode(dev, M24SR_WAITING_TIME_POLLING);

    status = m24sr_select_application(dev);
    if(status != M24SR_OK) {
        return status;
    }
    status = m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    if(status != M24SR_OK) {
        return status;
    }
    // read system file
    status = m24sr_read_binary (dev, 0x0004, &gpo_config, 0x01);
    if(status != M24SR_OK) {
        return status;
    }
    /* Update only GPO purpose for I2C */
    gpo_config = (gpo_config & 0xF0) | gpo_i2c_config;
    status = m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    if(status != M24SR_OK) {
        return status;
    }
    status = m24sr_verify(dev, I2C_PWD , 0x10 , default_pwd);
    if(status != M24SR_OK) {
        return status;
    }
    //write system file
    status = m24sr_update_binary(dev, 0x0004 , &gpo_config, 0x01);
    if(status != M24SR_OK) {
        return status;
    }
    /* if we have set interrupt mode for I2C synchro we can enable interrupt mode */
    if (gpo_i2c_config == I2C_ANSWER_READY && status == M24SR_OK)
#ifdef I2C_GPO_SYNCHRO_ALLOWED
        m24sr_set_i2c_synchro_mode(dev, M24SR_WAITING_TIME_GPO);
#else
        m24sr_set_i2c_synchro_mode(dev, M24SR_INTERRUPT_GPO);
#endif
    return status;
}

static uint16_t m24sr_manage_rf_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_rf_config)
{
    uint16_t status;
    uint8_t gpo_config;
    uint8_t default_pwd[M24SR_PASSWORD_NUM_BYTE] = {0x00};

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
    m24sr_verify(dev, I2C_PWD , M24SR_PASSWORD_NUM_BYTE , default_pwd);
    status = m24sr_update_binary(dev, 0x0004, &gpo_config, 0x01);

    return status;
}

int m24sr_init(m24sr_t *dev, const m24sr_params_t *params) {
    int status = M24SR_OK;
      

    /* Perform HW initialization */
    status = m24sr_i2c_init_hw (dev, params);
    if (status != M24SR_OK)
        return M24SR_ERROR;

    dev->synchro_mode = M24SR_WAITING_TIME_POLLING;   

    _m24sr_init_structure();

#if defined(I2C_GPO_SYNCHRO_ALLOWED) || defined(I2C_GPO_INTERRUPT_ALLOWED)
    if (_m24sr_open_i2c_session(dev, I2C_KILL_RF) == M24SR_OK) {
        status = m24sr_manage_i2c_gpo(dev, I2C_ANSWER_READY);
        if(status != M24SR_OK) {
            m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
            return M24SR_ERROR;
        }
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
    }
#endif /* I2C_GPO_SYNCHRO_ALLOWED */

    /* Open I2C session */
    status = _m24sr_open_i2c_session(dev, I2C_OPEN_SESSION);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    /* Select the NFC type 4 application */
    DEBUG("Select the NFC type 4 application\n");
    status = m24sr_select_application(dev);
    if (status != M24SR_OK) {
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_ERROR;
    }

    /* select a SYS file */
    sys_file_info_t sys_file;

    status = m24sr_select_system_file(dev, M24SR_SYS_FILE_ID);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    DEBUG("SYS FILE size %d\n", sizeof(sys_file));
    if (m24sr_read_binary(dev, 0x0000, (uint8_t *)&sys_file, sizeof(sys_file)) == M24SR_OK) {
        PRINTBUFF((uint8_t *)&sys_file, sizeof(sys_file));
        DEBUG("NDEF File Number is %02X\n", sys_file.ndef_file_num);
        DEBUG("Unique identifier by a %d bytes : ", sizeof(sys_file.UID));
        PRINTBUFF((uint8_t *)&sys_file.UID, sizeof(sys_file.UID));
        memcpy(dev->memory.uid, sys_file.UID, sizeof(sys_file.UID));
    } else {
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_ERROR;
    }

    /* select a CC file */
    cc_file_info_t cc_file;

    status = m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    /* read the first 15 bytes of the CC file */
    DEBUG("CC FILE size %d\n", sizeof(cc_file));
    if (m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file)) == M24SR_OK) {
        PRINTBUFF((uint8_t *)&cc_file, sizeof(cc_file));
        DEBUG("[CC FILE] Len is %d(0x%04X).\n", cc_file.cc_file_len, cc_file.cc_file_len);
        DEBUG("[CC FILE] Version is %d (0x%02X)\n", cc_file.version, cc_file.version); 
        DEBUG("[CC FILE] max_read_byte is %d (0x%04X)\n", cc_file.max_read_byte, cc_file.max_read_byte);
        DEBUG("[CC FILE] max_write_byte is %d (0x%04X)\n", cc_file.max_write_byte, cc_file.max_write_byte);
        DEBUG("[CC FILE] t_field is %d (0x%02X)\n", cc_file.t_field, cc_file.t_field);
        DEBUG("[CC FILE] l_field is %d (0x%02X)\n", cc_file.l_field, cc_file.l_field);
        DEBUG("[CC FILE] ndef_file_id is %d (0x%04X)\n", cc_file.ndef_file_id, cc_file.ndef_file_id);
        DEBUG("[CC FILE] ndef_file_max_size is %d (0x%04X)\n", cc_file.ndef_file_max_size, cc_file.ndef_file_max_size);
        DEBUG("[CC FILE] read_access is %d (0x%02X)\n", cc_file.read_access, cc_file.read_access);
        DEBUG("[CC FILE] write_access is %d (0x%02X)\n", cc_file.write_access, cc_file.write_access);

        dev->memory.chipsize       = cc_file.ndef_file_max_size;
        dev->memory.max_read_byte  = cc_file.max_read_byte;
        dev->memory.max_write_byte = cc_file.max_write_byte;
         
        byteorder_swap((void *)&dev->memory.chipsize, sizeof(dev->memory.chipsize));
        byteorder_swap((void *)&dev->memory.max_read_byte, sizeof(dev->memory.max_read_byte));
        byteorder_swap((void *)&dev->memory.max_write_byte, sizeof(dev->memory.max_write_byte));
        //dev->memory.chipsize      -= NDEF_FILE_LEN_NUM_BYTES;


        DEBUG("[CC FILE] chipsize %d(0x%04X)\n", dev->memory.chipsize, dev->memory.chipsize);
        DEBUG("[CC FILE] max_read_byte %d(0x%04X)\n", dev->memory.max_read_byte, dev->memory.max_read_byte);
        DEBUG("[CC FILE] max_write_byte %d(0x%04X)\n", dev->memory.max_write_byte, dev->memory.max_write_byte);

        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_OK;
    } else {
        m24sr_close_session(dev, I2C_TOKEN_RELEASE_SW);
        return M24SR_ERROR;
    }
    return status; 
}

static uint16_t m24sr_read_data(m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size) {
    uint16_t status;
    uint16_t max_read_byte = dev->memory.max_read_byte;

    DEBUG("Offset is 0x%04X. Length is %d(0x%04X)\n", offset, size, size);

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

static uint16_t m24sr_force_read_data(m24sr_t *dev, uint16_t offset, uint8_t *dst, uint16_t size) {
    uint16_t status;
    uint16_t max_read_byte = dev->memory.max_read_byte;

    DEBUG("Offset is 0x%04X. Length is %d(0x%04X)\n", offset, size, size);

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

static uint16_t m24sr_write_data (m24sr_t *dev, uint16_t offset, uint8_t *src, uint16_t size)
{
    uint16_t status;
    uint16_t max_write_byte = dev->memory.max_write_byte;

    DEBUG("Offset is 0x%04X. Length is %d(0x%04X)\n", offset, size, size);

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

static uint16_t m24sr_manage_gpo(m24sr_t *dev, m24sr_gpo_config_t gpo_config, uint8_t mode) {
    uint16_t status;

    if (mode == RF_GPO) {
        status = m24sr_manage_rf_gpo(dev, gpo_config);
    } else {
        status = m24sr_manage_i2c_gpo(dev, gpo_config);
    }
    return status;
}


/***************************** Implementation global function for NDEF *****************************************/
int m24sr_ndef_init(m24sr_t *dev, const m24sr_params_t *params) {
    int ret = M24SR_OK;

    ret = m24sr_init(dev, params);

    return ret;
}


int m24sr_ndef_read(m24sr_t *dev, void *dst, uint16_t addr, uint16_t size) {
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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[NDEF File] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }


    byteorder_swap((void *)&ndef_file_size, sizeof(ndef_file_size));
    DEBUG("[NDEF File] Size is %d(0x%04X)\n", ndef_file_size, ndef_file_size);

    if (size <= ndef_file_size) {
        status = m24sr_read_data(dev, NDEF_FILE_LEN_NUM_BYTES + addr, dst, size);
        DEBUG("[NDEF File] status is %d\n", status);
    } else {
        return M24SR_ERROR;
    }
    
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);

    return size;
}

int m24sr_ndef_write(m24sr_t *dev, void *src, uint16_t addr, uint16_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    uint16_t ndef_file_size = 0x0000;
    int status = M24SR_OK;
    uint16_t write_size = size;

    
    if (addr > total_size) {
        return -EOVERFLOW;
    }
    if (addr + size > total_size) {
        return -EOVERFLOW;
    }
    if (size == 0) {
        return 0;
    }

    DEBUG("Address is 0x%04X. Length is %d(0x%04X)\n", addr, size, size);


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
    m24sr_read_binary(dev, 0x0000, (uint8_t * )&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[NDEF File] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    
    byteorder_swap((void *)&ndef_file_size, sizeof(ndef_file_size));
    write_size += ndef_file_size;
    DEBUG("write_size is %d(0x%04X)\n", write_size, write_size);
    DEBUG("[NDEF File] Read size is %d(0x%04X)\n", ndef_file_size, ndef_file_size);
    DEBUG("[NDEF File] Free size %d(0x%04X)\n", (total_size - ndef_file_size), (total_size - ndef_file_size));

    if (size <= (total_size - ndef_file_size)) {
        status = m24sr_write_data(dev, NDEF_FILE_LEN_NUM_BYTES + addr, src, size);
    } else {
        return M24SR_ERROR;
    }
    
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    DEBUG("[NDEF File] Write size is %d(0x%04X)\n", size, size);
    DEBUG("[NDEF File] Full size is %d(0x%04X)\n", write_size, write_size);
    byteorder_swap((void *)&write_size, sizeof(write_size));
    status = m24sr_write_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&write_size, NDEF_FILE_LEN_NUM_BYTES);

    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);

    DEBUG("[NDEF File] Write size is %d(0x%04X)\n", size, size);
    return size;
}

int m24sr_ndef_erase(m24sr_t *dev, uint16_t addr, uint16_t size) {
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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[NDEF File] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    byteorder_swap((void *)&ndef_file_size, sizeof(ndef_file_size));
    DEBUG("[NDEF File] Read size is %d(0x%04X)\n", ndef_file_size, ndef_file_size);


    memset(src, 0xFF, sizeof(src));

    if (size < (total_size - ndef_file_size)) {
        status = m24sr_write_data(dev, NDEF_FILE_LEN_POS + addr, src, size + NDEF_FILE_LEN_NUM_BYTES);
    } else {
        return M24SR_ERROR;
    }
    
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    DEBUG("[NDEF File] Write size is %d(0x%04X)\n", size, size);
    status = m24sr_write_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&size, NDEF_FILE_LEN_NUM_BYTES);

    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }


    m24sr_close_session(dev, dev->params.token_mode);

    return 0;
}


int m24sr_ndef_erase_all(m24sr_t *dev) {
    //uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    uint16_t ndef_file_size = 0x0000;
    int status = M24SR_OK;
    uint16_t size = 0x0000;

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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[NDEF File] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&ndef_file_size, NDEF_FILE_LEN_NUM_BYTES);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    byteorder_swap((void *)&ndef_file_size, sizeof(ndef_file_size));
    DEBUG("[NDEF File] Read size is %d(0x%04X)\n", ndef_file_size, ndef_file_size);


    DEBUG("[NDEF File] Write size is %d(0x%04X)\n", size, size);
    status = m24sr_write_data(dev, NDEF_FILE_LEN_POS, (uint8_t *)&size, NDEF_FILE_LEN_NUM_BYTES);

    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);
    
    return 0;
}



/***************************** Implementation global function for emmulatiom I2C eeprom*******************************/
int m24sr_eeprom_init(m24sr_t *dev, const m24sr_params_t *params) {
    int ret = M24SR_OK;

    ret = m24sr_init(dev, params);

    return ret;
}


int m24sr_eeprom_read(m24sr_t *dev, void *dst, uint16_t addr, uint16_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
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
    /*Select */
    m24sr_select_capability_container_file(dev, M24SR_CC_FILE_ID);
    m24sr_read_binary(dev, 0x0000, (uint8_t * )&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[EEPROM] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    status = m24sr_read_data(dev, addr, dst, size);
    DEBUG("[EEPROM] status is %d\n", status);
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);

    return size;
}

int m24sr_eeprom_write(m24sr_t *dev, void *src, uint16_t addr, uint16_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[EEPROM] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
   
    status = m24sr_write_data(dev, addr, src, size);
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);

    return size;
}

int m24sr_eeprom_erase(m24sr_t *dev, uint16_t addr, uint16_t size) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[EEPROM] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }

    memset(src, 0xFF, sizeof(src));

    status = m24sr_write_data(dev, addr, src, size);
    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);

    return M24SR_OK;
}


int m24sr_eeprom_erase_all(m24sr_t *dev) {
    uint16_t total_size = dev->memory.chipsize;
    cc_file_info_t cc_file;
    uint16_t ndef_file_id = 0x0000;
    int status = M24SR_OK;
    uint8_t src[total_size];

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
    m24sr_read_binary(dev, 0x0000, (uint8_t *)&cc_file, sizeof(cc_file));
    
    ndef_file_id = cc_file.ndef_file_id;
    byteorder_swap((void *)&ndef_file_id, sizeof(ndef_file_id));
    DEBUG("[EEPROM] ID is %d(0x%04X)\n", ndef_file_id, ndef_file_id);

    /* select NDEF file   */
    status = m24sr_select_ndef_file(dev, ndef_file_id);
    if (status != M24SR_OK) {
        return M24SR_ERROR;
    }
    memset(src, 0xFF, total_size);

    status = m24sr_write_data(dev, 0, src, total_size);

    if(status != M24SR_OK) {
        return M24SR_ERROR;
    }

    m24sr_close_session(dev, dev->params.token_mode);
    
    return 0;
}


int m24sr_eeprom_power(m24sr_t *dev, enum m24sr_power_state power) {

    if (dev->params.pwr_en_pin != GPIO_UNDEF) {
        switch (power) {
            case M24SR_POWER_UP:
                gpio_set(dev->params.pwr_en_pin);
                break;
            case M24SR_POWER_DOWN:
                gpio_clear(dev->params.pwr_en_pin);
                break;
            default:
                break;
        }
    } else {
        (void)dev;
        (void)power;
    }

    return 0;
}