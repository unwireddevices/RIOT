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

#include "m24sr.h"

#include "assert.h"
#include "periph/gpio.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"


/* shortcuts for I2C bus parameters */
#define DEV_I2C            (dev->params.i2c)
#define DEV_ADDR           (dev->params.i2c_addr)


#if ENABLE_DEBUG
#define PRINTBUFF _printbuff
static void _printbuff(uint8_t *buff, unsigned len)
{
    while (len) {
        len--;
        printf("%02x ", *buff++);
    }
    printf("\n");
}
#else
#define PRINTBUFF(...)
#endif

static int _write_i2c(const m24sr_t *dev, uint8_t *buff, unsigned len)
{
    int ret = -1;

    i2c_acquire(dev->params.i2c);
    ret = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, buff, len, 0);
    i2c_release(dev->params.i2c);

    DEBUG("m24sr: -> ");
    PRINTBUFF(buff, len);
    return ret;
}

static int _read_i2c(const m24sr_t *dev, uint8_t *buff, unsigned len)
{
    int ret = -1;

    i2c_acquire(dev->params.i2c);
    ret = i2c_read_bytes(dev->params.i2c, dev->params.i2c_addr, buff, len, 0);
    i2c_release(dev->params.i2c);

    DEBUG("m24sr: <- ");
    PRINTBUFF(buff, len);
    return ret;
}


/**
  * @brief  This functions sends the command buffer
  * @param  NbByte : Number of byte to send
  * @param  pBuffer : pointer to the buffer to send to the M24SR
  * @retval M24SR_STATUS_SUCCESS : the function is succesful
  * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
static int _m24sr_send_i2c_cmd(const m24sr_t *dev, uint8_t NbByte , uint8_t *pBuffer)
{
    if (_write_i2c(dev, pBuffer, NbByte) == 0)
        return M24SR_STATUS_SUCCESS;
    else
        return M24SR_ERROR_I2CTIMEOUT;
}

/**
  * @brief  This functions reads a response of the M24SR device
  * @param  NbByte : Number of byte to read (shall be >= 5)
  * @param  pBuffer : Pointer on the buffer to retrieve M24SR response
  * @retval M24SR_STATUS_SUCCESS : The function is succesful
  * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
static int _m24sr_rcv_i2c_response(const m24sr_t *dev, uint8_t NbByte , uint8_t *pBuffer )
{
    if ( _read_i2c(dev, pBuffer, NbByte) == 0)
        return M24SR_STATUS_SUCCESS;
    else
        return M24SR_ERROR_I2CTIMEOUT;
}

/**
  * @brief  This functions polls the I2C interface
  * @retval M24SR_STATUS_SUCCESS : the function is succesful
  * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
int _m24sr_poll_i2c (const m24sr_t *dev)
{

    int status = 0x0;
    /* Get tick */
    const uint32_t timestamp = (xtimer_now_usec() / US_PER_MS);

    /* Wait until M24SR is ready or timeout occurs */
    do
    {
        status = _write_i2c(dev, 0x00, 0);
    } while ((((xtimer_now_usec() / US_PER_MS) - timestamp) < M24SR_I2C_TIMEOUT) && (status != 0));

    if (status == 0)
        return M24SR_STATUS_SUCCESS;
    else
        return M24SR_ERROR_I2CTIMEOUT;
}


static int _m24sr_release_i2c_token(const m24sr_t *dev) {

    int status = 0x0;
    uint8_t data[] = {0x00};

    status = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTOP);
    if (status != 0)
        return M24SR_ERROR_I2CTIMEOUT;

    status = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTART);
    if (status == 0)
        return M24SR_STATUS_SUCCESS;
    else
        return M24SR_ERROR_I2CTIMEOUT;
}


/**
  * @brief  This function sends the Deselect command (S-Block format)
    * @retval M24SR_ACTION_COMPLETED : the function is succesful.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
static int m24sr_deselect (const m24sr_t *dev, m24sr_token_mode_t mode)
{
    int status = 0x0;

    switch (mode) {
        case I2C_TOKEN_RELEASE_SW: {
            if (_m24sr_release_token(dev) != M24SR_STATUS_SUCCESS) {
                status = M24SR_STATUS_SUCCESS;
            }
            else 
                status = M24SR_ERROR_I2CTIMEOUT;
            break;
        }
        case I2C_TOKEN_RELEASE_HW: {
            //do nothin @TODO: add
            break;  
        }
        default:
            status = M24SR_ERROR_I2CTIMEOUT;
            break;
    }

    return status;
}



/**
  * @brief  This function enable or disable RF communication
  * @param  OnOffChoice: GPO configuration to set
  */
void m24sr_rf_config_hw(const m24sr_t *dev, uint8_t OnOffChoice)
{
    /* Disable RF */
    if ( OnOffChoice != 0 )
    {
        gpio_clear(dev->param.rfdisable_pin);
    }
    else
    {
        gpio_set(dev->param.rfdisable_pin);
    }
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
static uint16_t _m24sr_is_correct_crc_residue (uint8_t *data, uint8_t length)
{
    uint16_t resCRC = 0;

    /* check the CRC16 Residue */
    if (length != 0)
        resCRC = _m24sr_compute_crc (data, length);

    if ( resCRC == 0x0000)
    {
        /* Good CRC, but error status from M24SR */
        return ( ((data[length - UB_STATUS_OFFSET] << 8) & 0xFF00) | (data[length - LB_STATUS_OFFSET] & 0x00FF) );
    }
    else
    {
        resCRC = 0;
        resCRC = _m24sr_compute_crc (data, 5);
        if ( ResCRC != 0x0000)
        {
            /* Bad CRC */
            return M24SR_ERROR_CRC;
        }
        else
        {
            /* Good CRC, but error status from M24SR */
            return ( ((data[1] << 8) & 0xFF00) | (data[2] & 0x00FF) );
        }
    }
}


m24sr_waiting_time_mode_t uSynchroMode = M24SR_WAITINGTIME_POLLING;
volatile uint8_t  GPO_Low = 0;


/**
  * @brief  this functions configure I2C synchronization mode
  * @param  mode : interruption or polling
  * @retval None
  */
void m24sr_set_i2c_synchro_mode(m24sr_waiting_time_mode_t mode)
{
#if defined (I2C_GPO_SYNCHRO_ALLOWED) || defined (I2C_GPO_INTERRUPT_ALLOWED)
    uSynchroMode = mode;
#else
    if (mode == M24SR_WAITINGTIME_GPO || mode == M24SR_INTERRUPT_GPO)
        uSynchroMode = M24SR_WAITINGTIME_POLLING;
    else
        uSynchroMode = mode;
#endif /*  I2C_GPO_SYNCHRO_ALLOWED */
}


/**
  * @brief  This functions returns M24SR_STATUS_SUCCESS when a response is ready
  * @retval M24SR_STATUS_SUCCESS : a response of the M24LR is ready
  * @retval M24SR_ERROR_DEFAULT : the response of the M24LR is not ready
  */
int m24sr_is_answer_rdy(const m24sr_t *dev)
{
    uint16_t status;
    uint32_t retry = 0xFFFFF;
    uint8_t stable = 0;

    switch (uSynchroMode)
    {
        case M24SR_WAITINGTIME_POLLING :
            if( _m24sr_poll_i2c(dev) != M24SR_STATUS_SUCCESS)
                return M24SR_ERROR_DEFAULT;
            return M24SR_STATUS_SUCCESS;
        case M24SR_WAITINGTIME_TIMEOUT :
            // M24SR FWI=5 => (256*16/fc)*2^5=9.6ms but M24SR ask for extended time to program up to 246Bytes.
            // can be improved by
            M24SR_WaitMs(80);
            return M24SR_STATUS_SUCCESS;
        case M24SR_WAITINGTIME_GPO :
            /* mbd does not support interrupt for the moment with nucleo board */
            do
            {
                if (m24sr_gpo_read(dev) == GPO_PIN_RESET)
                {
                    stable ++;
                }
                retry --;
            }
            while (stable < 5 && retry > 0);
            if (!retry)
               return M24SR_ERROR_DEFAULT;
            return M24SR_STATUS_SUCCESS;
        case M24SR_INTERRUPT_GPO :
            /* Check if the GPIO is not already low before calling this function */
            if (m24sr_gpo_read(dev) == GPO_PIN_SET)
            {
                while (GPO_Low == 0);
            }
            GPO_Low = 0;
            return M24SR_STATUS_SUCCESS;

        default :
            return M24SR_ERROR_DEFAULT;
    }
}



/**
  * @brief  This function retrieve current tick
  * @param  ptickstart: pointer on a variable to store current tick value
  */
void M24SR_GetTick( uint32_t *ptickstart )
{
    *ptickstart = xtimer_now().ticks32;
}

/**
  * @brief  This function wait the time given in param (in milisecond)
  * @param  time_ms: time value in milisecond
  */
void m24sr_wait_millis(uint32_t time_ms)
{
    xtimer_sleep(time_ms*1000);
}

/**
  * @brief  This function read the state of the M24SR GPO
  * @param  none
  * @retval GPIO PinState : state of the M24SR GPO
  */
int  m24sr_gpo_read(const m24sr_t *dev)
{
    return gpio_read(dev->params.gpo_pin);
}

/**
  * @brief  This function initializes the M24SR_I2C interface
  * @retval None
  */
int m24sr_i2c_init_hw (m24sr_t *dev, const m24sr_params_t *params, gpio_cb_t gpo_pin_cb, void *gpo_pin_cb_arg)
{
    int retval = 0x0;

    dev->params = *params;

    /* Configure GPIO pins*/
#if I2C_GPO_INTERRUPT_ALLOWED
    retval = gpio_init_int(dev->params.gpo_pin, GPIO_IN, dev->params.gpo_flank, gpo_pin_cb, gpo_pin_cb_arg);
    if (retval < 0) {
        DEBUG("[m24sr] error: failed to initialize GPO pin\n");
        return retval;
    }
#else
    (void *) gpo_pin_cb;
    (void *) gpo_pin_cb_arg;
    retval = gpio_init(dev->params.gpo_pin, GPIO_IN);
    if (retval < 0) {
        DEBUG("[m24sr] error: failed to initialize GPO pin\n");
        return retval;
    }
#endif

    if (dev->params.rfdisable_pin != GPIO_UNDEF) {
            /* Configure GPIO pins for RF DISABLE*/
        retval = gpio_init(dev->params.rfdisable_pin, GPIO_IN);
        if (retval < 0) {
            DEBUG("[m24sr] error: failed to initialize RF DISABLE pin\n");
            return retval;
        }
    }

    if (dev->params.pwr_en_pin != GPIO_UNDEF) {
        retval = gpio_init(dev->params.pwr_en_pin, GPIO_OUT);
        if (retval < 0){
            DEBUG("[m24sr] error: failed to initialize POWER EN pin\n");
            return retval;
        }
    }

    return retval;  
}


/**
  * @brief  Initialize the command and response structure
  * @param  None
  * @retval None
  */
static void _m24sr_init_structure ( void ) {
    /* build the command */
    Command.Header.CLA = 0x00;
    Command.Header.INS = 0x00;
    /* copy the offset */
    Command.Header.P1 = 0x00 ;
    Command.Header.P2 = 0x00 ;
    /* copy the number of byte of the data field */
    Command.Body.LC = 0x00 ;
    /* copy the number of byte to read */
    Command.Body.LE = 0x00 ;
    Command.Body.pData = DataBuffer;

    /* initializes the response structure*/
    Response.pData = DataBuffer;
    Response.SW1 = 0x00;
    Response.SW2 = 0x00;
}


/**
 * Structure of the command sets
 */

/**
  * @brief      This functions creates an I block command according to the structures CommandStructure and Command.
  * @param        Command : structue which contains the field of the different parameter
  * @param        CommandStructure : structure that contain the structure of the command (if the different field are presnet or not
  * @param        NbByte : number of byte of the command
  * @param        pCommand : pointer of the command created
  */
static void _m24sr_build_iblock_cmd ( uint16_t cmd_str, cmd_apdu_t apdu_cmd, uint16_t *nbByte , uint8_t *iblock) {
    uint16_t    uCRC16;
    static uint8_t BlockNumber = 0x01;

    (*NbByte) = 0;

    /* add the PCD byte */
    if ((cmd_str & M24SR_PCB_NEEDED) != 0) {
        /* toggle the block number */
        BlockNumber = TOGGLE (BlockNumber);
        /* Add the I block byte */
        iblock[(*NbByte)++] = 0x02|BlockNumber;
    }
    /* add the DID byte */
    if ((BlockNumber & M24SR_DID_NEEDED) != 0) {
        /* Add the I block byte */
        iblock[(*NbByte)++] = uDIDbyte;
    }
    /* add the Class byte */
    if ((cmd_str & M24SR_CLA_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.header.CLA ;
    }
    /* add the instruction byte byte */
    if ( (cmd_str & M24SR_INS_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.header.INS ;
    }
    /* add the Selection Mode byte */
    if ((cmd_str & M24SR_P1_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.header.P1 ;
    }
    /* add the Selection Mode byte */
    if ((cmd_str & M24SR_P2_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.header.P2 ;
    }
    /* add Data field lengthbyte */
    if ((cmd_str & M24SR_LC_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.body.LC ;
    }
    /* add Data field  */
    if ((cmd_str & M24SR_DATA_NEEDED) != 0) {
        memcpy(&(iblock[(*NbByte)]) , apdu_cmd.body.pData, apdu_cmd.body.LC ) ;
        (*NbByte) += apdu_cmd.body.LC ;
    }
    /* add Le field  */
    if ((cmd_str & M24SR_LE_NEEDED) != 0) {
        iblock[(*NbByte)++] = apdu_cmd.body.LE ;
    }
    /* add CRC field  */
    if ((cmd_str & M24SR_CRC_NEEDED) != 0) {
        uCRC16 = _m24sr_compute_crc(iblock, (uint8_t) (*NbByte));
        /* append the CRC16 */
        iblock [(*NbByte)++] = GETLSB (uCRC16 ) ;
        iblock [(*NbByte)++] = GETMSB (uCRC16 ) ;
    }
}

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an I-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a I-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a I-Block
*/
static int _is_iblock (uint8_t *pBuffer) {
    if ((pBuffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_IBLOCK) {
        return M24SR_STATUS_SUCCESS;
    }
    else {
        return M24SR_ERROR_DEFAULT;
    }
}

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an R-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a R-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a R-Block
*/
static int _is_rblock (uint8_t *pBuffer) {
    if ((pBuffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_RBLOCK) {
        return M24SR_STATUS_SUCCESS;
    }
    else {
        return M24SR_ERROR_DEFAULT;
    }
}

/**
* @brief    This function return M24SR_STATUS_SUCCESS if the pBuffer is an s-block
* @param    pBuffer     :   pointer of the data
* @retval   M24SR_STATUS_SUCCESS  :  the data is a S-Block
* @retval   M24SR_ERROR_DEFAULT     :  the data is not a S-Block
*/
static int _is_sblock (uint8_t *pBuffer) {
    if ((pBuffer[M24SR_OFFSET_PCB] & M24SR_MASK_BLOCK) == M24SR_MASK_SBLOCK) {
        return M24SR_STATUS_SUCCESS;
    }
    else {
        return M24SR_ERROR_DEFAULT;
    }
}


/**
  * @brief  This function sends the GetSession command to the M24SR device
    * @retval M24SR_ACTION_COMPLETED : the function is succesful.
  * @retval Status (SW1&SW2) : if operation does not complete.
  */
int m24sr_get_session(const m24sr_t *dev ) {
    uint8_t Buffer = M24SR_OPENSESSION;
    int16_t status;

    if(_m24sr_send_i2c_cmd ( 0x01 , &Buffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    /* Insure no access will be done just after open session */
    /* The only way here is to poll I2C to know when M24SR is ready */
    /* GPO can not be use with GetSession command */
    if(_m24sr_poll_i2c ( ))
        return M24SR_ERROR_I2CTIMEOUT;

    return M24SR_ACTION_COMPLETED;
}



/**
  * @brief  This function sends the SelectApplication command
    * @retval M24SR_ACTION_COMPLETED : the function is succesful.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
int m24sr_select_application(const m24sr_t *dev)
{
    uint8_t *pBuffer = uM24SRbuffer;
    uint8_t NbByteToRead = M24SR_STATUSRESPONSE_NBBYTE;
    uint8_t uLc = 0x07;
    uint8_t pData[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
    uint8_t uLe = 0x00;
    uint16_t  status ;
    uint16_t  uP1P2 = 0x0400;
    uint16_t  NbByte;

    /* build the command */
    Command.Header.CLA = C_APDU_CLA_DEFAULT;
    Command.Header.INS = C_APDU_SELECT_FILE;
    /* copy the offset */
    Command.Header.P1 = GETMSB  (uP1P2 ) ;
    Command.Header.P2 = GETLSB  (uP1P2 ) ;
    /* copy the number of byte of the data field */
    Command.Body.LC = uLc ;
    /* copy the data */
    memcpy(Command.Body.pData, pData, uLc);
    /* copy the number of byte to read */
    Command.Body.LE = uLe ;
    /* build the I2C command */
    _m24sr_build_iblock_cmd ( M24SR_CMDSTRUCT_SELECTAPPLICATION,  Command, &NbByte , pBuffer);

    /* send the request */
    if (_m24sr_send_i2c_cmd (dev, NbByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    if ( m24sr_is_answer_rdy ( ))
        return M24SR_ERROR_I2CTIMEOUT;
    /* read the response */
    if( _m24sr_rcv_i2c_response(dev, NbByteToRead , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    status = _m24sr_is_correct_crc_residue(pBuffer, NbByteToRead);
    return status;
}


/**
  * @brief  This function sends the SelectSystemFile command
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
uint16_t m24sr_select_sys_file(const m24sr_t *dev)
{
    uint8_t *pBuffer = uM24SRbuffer;
    uint8_t  NbByteToRead = M24SR_STATUSRESPONSE_NBBYTE;
    uint8_t  uLc = 0x02;
    uint16_t status;
    uint16_t uP1P2 = 0x000C;
    uint16_t uNbFileId = SYSTEM_FILE_ID;
    uint16_t NbByte;

    /* build the command */
    Command.Header.CLA = C_APDU_CLA_DEFAULT;
    Command.Header.INS = C_APDU_SELECT_FILE;
    /* copy the offset */
    Command.Header.P1 = GETMSB  (uP1P2 ) ;
    Command.Header.P2 = GETLSB  (uP1P2 ) ;
    /* copy the number of byte of the data field */
    Command.Body.LC = uLc ;
    /* copy the File Id */
    Command.Body.pData[0] = GETMSB  (uNbFileId ) ;
    Command.Body.pData[1] = GETLSB  (uNbFileId ) ;
    /* build the I2C command */
    _m24sr_build_iblock_cmd( M24SR_CMDSTRUCT_SELECTCCFILE,  Command, &NbByte , pBuffer);

    /* send the request */
    if (_m24sr_send_i2c_cmd (dev, NbByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    if( m24sr_is_answer_rdy ( ))
       return M24SR_ERROR_I2CTIMEOUT;
    /* read the response */
    if( _m24sr_rcv_i2c_response ( NbByteToRead , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    status = _m24sr_is_correct_crc_residue (pBuffer, NbByteToRead);
    return status;
}

/**
  * @brief  This function sends a read binary command
    * @param    Offset : first byte to read
    * @param    NbByteToRead : number of byte to read
    * @param    pBufferRead : pointer of the buffer read from the M24SR device
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
uint16_t m24sr_read_binary(uint16_t Offset , uint8_t NbByteToRead , uint8_t *pBufferRead )
{
    uint8_t     *pBuffer = uM24SRbuffer ;
    uint16_t    status ;
    uint16_t    NbByte;

    /* build the command */
    Command.Header.CLA = C_APDU_CLA_DEFAULT;
    Command.Header.INS = C_APDU_READ_BINARY;
    /* copy the offset */
    Command.Header.P1 = GETMSB  (Offset ) ;
    Command.Header.P2 = GETLSB  (Offset ) ;
    /* copy the number of byte to read */
    Command.Body.LE = NbByteToRead ;

    _m24sr_build_iblock_cmd( M24SR_CMDSTRUCT_READBINARY,  Command, &NbByte , pBuffer);

    if( _m24sr_send_i2c_cmd ( NbByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    if ( m24sr_is_answer_rdy ( ))
        return M24SR_ERROR_I2CTIMEOUT;
    if( _m24sr_rcv_i2c_response ( NbByteToRead + M24SR_STATUSRESPONSE_NBBYTE , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    status = _m24sr_is_correct_crc_residue (pBuffer, NbByteToRead + M24SR_STATUSRESPONSE_NBBYTE);
    /* retrieve the data without SW1 & SW2 as provided as return value of the function */
    memcpy(pBufferRead , &pBuffer[1], NbByteToRead);
    return status;
}


/**
  * @brief  This function sends the Verify command
    * @param    uPwdId : PasswordId ( 0x0001 : Read NDEF pwd or 0x0002 : Write NDEF pwd or 0x0003 : I2C pwd)
    * @param    NbPwdByte : Number of byte ( 0x00 or 0x10)
    * @param    pPwd : pointer on the passwaord
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
uint16_t m24sr_verify(uint16_t uPwdId, uint8_t NbPwdByte , uint8_t *pPwd )
{
    uint8_t   *pBuffer = uM24SRbuffer ;
    uint16_t  status = 0x0000 ;
    uint16_t  NbByte;

    /*check the parameters */
    if (uPwdId > 0x0003)
    {
        return M24SR_ERROR_PARAMETER;
    }
    if ( (NbPwdByte != 0x00) && (NbPwdByte != 0x10))
    {
        return M24SR_ERROR_PARAMETER;
    }

    /* build the command */
    Command.Header.CLA = C_APDU_CLA_DEFAULT;
    Command.Header.INS = C_APDU_VERIFY;
    /* copy the Password Id */
    Command.Header.P1 = GETMSB  (uPwdId ) ;
    Command.Header.P2 = GETLSB  (uPwdId ) ;
    /* copy the number of byte of the data field */
    Command.Body.LC = NbPwdByte ;

    if (NbPwdByte == 0x10)
    {
        /* copy the password */
        memcpy(Command.Body.pData, pPwd, NbPwdByte);
        /* build the I2C command */
        _m24sr_build_iblock_cmd ( M24SR_CMDSTRUCT_VERIFYBINARYWITHPWD,  Command, &NbByte , pBuffer);
    }
    else
    {
        /* build the I2C command */
        _m24sr_build_iblock_cmd ( M24SR_CMDSTRUCT_VERIFYBINARYWOPWD,  Command, &NbByte , pBuffer);
    }

    /* send the request */
    if( _m24sr_send_i2c_cmd ( NbByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    /* wait for answer ready */
    if( m24sr_is_answer_rdy ( ))
        return M24SR_ERROR_I2CTIMEOUT;
    /* read the response */
    if( _m24sr_rcv_i2c_response ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    status = _m24sr_is_correct_crc_residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
    return status;
}


/**
  * @brief  This function sends a Update binary command
    * @param    Offset : first byte to read
    * @param    NbByteToWrite : number of byte to write
    * @param    pBufferRead : pointer of the buffer read from the M24SR device
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
uint16_t m24sr_update_binary( uint16_t Offset , uint8_t NbByteToWrite, uint8_t *pDataToWrite )
{
    uint8_t   *pBuffer = uM24SRbuffer ;
    uint16_t  status ;
    uint16_t  NbByte;

    /* build the command */
    Command.Header.CLA = C_APDU_CLA_DEFAULT;
    Command.Header.INS = C_APDU_UPDATE_BINARY;
    /* copy the offset */
    Command.Header.P1 = GETMSB  (Offset ) ;
    Command.Header.P2 = GETLSB  (Offset ) ;
    /* copy the number of byte of the data field */
    Command.Body.LC = NbByteToWrite ;
    /* copy the File Id */
    memcpy(Command.Body.pData , pDataToWrite, NbByteToWrite );

    _m24sr_build_iblock_cmd ( M24SR_CMDSTRUCT_UPDATEBINARY,  Command, &NbByte , pBuffer);

    if( _m24sr_send_i2c_cmd ( NbByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    if( m24sr_is_answer_rdy ( ))
        return M24SR_ERROR_I2CTIMEOUT;
    if( _m24sr_rcv_i2c_response ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    /* if the response is a Watiting frame extenstion request */
    if (_is_sblock (pBuffer) == M24SR_STATUS_SUCCESS)
    {
        /*check the CRC */
        if (_m24sr_is_correct_crc_residue (pBuffer , M24SR_WATINGTIMEEXTRESPONSE_NBBYTE) != M24SR_ERROR_CRC)
        {
            /* send the FrameExension response*/
            status = m24sr_fwt_extension (  pBuffer [M24SR_OFFSET_PCB + 1] );
        }
    }
    else
    {
        status = _m24sr_is_correct_crc_residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
    }

    return status;
}


/**
  * @brief  This function sends the FWT extension command (S-Block format)
    * @param    FWTbyte : FWT value
  * @retval Status (SW1&SW2) : Status of the operation to complete.
    * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
static uint16_t m24sr_fwt_extension( uint8_t FWTbyte )
{
    uint8_t  pBuffer[M24SR_STATUSRESPONSE_NBBYTE];
    uint16_t status;
    uint16_t NthByte = 0;
    uint16_t uCRC16;

    /* create the response */
    pBuffer[NthByte++] = 0xF2 ;
    pBuffer[NthByte++] = FWTbyte ;
    /* compute the CRC */
    uCRC16 = _m24sr_compute_crc (pBuffer, 0x02);
    /* append the CRC16 */
    pBuffer [NthByte++] = GETLSB(uCRC16 ) ;
    pBuffer [NthByte++] = GETMSB(uCRC16 ) ;

    /* send the request */
    if ( _m24sr_send_i2c_cmd ( NthByte , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;
    if( m24sr_is_answer_rdy ( ))
        return M24SR_ERROR_I2CTIMEOUT;
    /* read the response */
    if( _m24sr_rcv_i2c_response ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ))
        return M24SR_ERROR_I2CTIMEOUT;

    status = _m24sr_is_correct_crc_residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
    return status;
}






/**
  * @brief  This function configure GPO purpose for I2C session
    * @param    GPO_I2Cconfig: GPO configuration to set
  * @retval Status (SW1&SW2) : Status of the operation to complete.
  */
uint16_t m24sr_manage_i2c_gpo(m24sr_gpo_mode_t gpo_i2c_config)
{
    uint16_t status;
    uint8_t GPO_config;
    uint8_t DefaultPassword[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                  };

    if ( gpo_i2c_config > STATE_CONTROL)
    {
        return M24SR_ERROR_PARAMETER;
    }

    /* we must not be in interrupt mode for I2C synchro as we will change GPO purpose */
    m24sr_set_i2c_synchro_mode(M24SR_WAITINGTIME_POLLING);

    m24sr_select_application();
    m24sr_select_sys_file();

    m24sr_read_binary (0x0004, 0x01, &GPO_config);

    /* Update only GPO purpose for I2C */
    GPO_config = (GPO_config & 0xF0) | GPO_I2Cconfig;
    m24sr_select_sys_file();
    m24sr_verify( I2C_PWD , 0x10 , DefaultPassword );
    status = m24sr_update_binary ( 0x0004 , 0x01, &(GPO_config) );

    /* if we have set interrupt mode for I2C synchro we can enable interrupt mode */
    if ( GPO_I2Cconfig == I2C_ANSWER_READY && status == M24SR_ACTION_COMPLETED)
#ifdef I2C_GPO_SYNCHRO_ALLOWED
        m24sr_set_i2c_synchro_mode(M24SR_WAITINGTIME_GPO);
#else
        m24sr_set_i2c_synchro_mode(M24SR_INTERRUPT_GPO);
#endif

    return status;
}




/**
  * @brief  This fonction initialize the M24SR
  * @param  CCBuffer : pointer on the buffer to store CC file
  * @param  size : number of byte of data to read
  * @retval SUCCESS : Initalization done
  * @retval ERROR : Not able to Initialize.
  */
uint16_t m24sr_init (const m24sr_t *dev, const m24sr_params_t *params, gpio_cb_t gpo_pin_cb, void *gpo_pin_cb_arg, uint8_t* CCBuffer, uint8_t size )
{
    uint16_t status = M24SR_OK;
    uint16_t trials = 5; /* wait 1sec, driver is configured to let 200ms for command to complete */
    /* which is enough for all commands except GetSession if RF session is already opened */
    /* Smartphone generaly release the session within the second, but customer can modify this value */

    /* Perform HW initialization */
    if (m24sr_i2c_init_hw (m24sr_t *dev, const m24sr_params_t *params, gpio_cb_t gpo_pin_cb, void *gpo_pin_cb_arg) != 0)
        status = ERROR;

    _m24sr_init_structure();

#if defined (I2C_GPO_SYNCHRO_ALLOWED) || defined (I2C_GPO_INTERRUPT_ALLOWED)
    if (m24sr_kill_rf_session() == M24SR_ACTION_COMPLETED)
    {
        m24sr_manage_i2c_gpo(I2C_ANSWER_READY);
        m24sr_deselect ();
    }
#endif /* I2C_GPO_SYNCHRO_ALLOWED */
}

    /* Read CC file */
    while ( status != M24SR_ACTION_COMPLETED && trials)
    {
        status = m24sr_get_session();
        trials--;
    }
    if (status != M24SR_ACTION_COMPLETED)
        return ERROR;
    /*===================================*/
    /* Select the NFC type 4 application */
    /*===================================*/
    errorchk( M24SR_SelectApplication() );

    /*==================*/
    /* select a CC file */
    /*==================*/
    errorchk (M24SR_SelectCCfile() );

    /* read the first 15 bytes of the CC file */
    if ( M24SR_ReadData ( 0x0000 , 0x0F , CCBuffer ) == M24SR_ACTION_COMPLETED)
    {
        NDEF_FileID = (uint16_t) ((CCBuffer[0x09] << 8) | CCBuffer[0x0A]);
        errorchk( M24SR_Deselect () );
        return SUCCESS;
    }
    else
        errorchk( M24SR_Deselect () );

Error:
    return ERROR;

}



// /** @addtogroup M24SR_Driver
//   * @{
//   * @brief  <b>This folder contains the driver layer of M24SR family (M24SR64, M24SR16, M24SR04, M24SR02)</b>
//   */


// /** @defgroup M24SR
//   * @{
//     *   @brief  This file contains the driver which implements all the M24SR commands.
//   */

// static C_APDU Command;
// static R_APDU Response;
// static uint8_t DataBuffer[0xFF];
// uint8_t uM24SRbuffer [0xFF];
// static uint8_t uDIDbyte = 0x00;





/

// /**
//   * @}
//   */


// /** @defgroup drvM24SR_Public_Functions
//   * @{
//   */







// /**
//   * @brief  This function sends the KillSession command to the M24SR device
//   * @param  None
//     * @retval M24SR_ACTION_COMPLETED : the function is succesful.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t m24sr_kill_rf_session ( void )
// {
//     uint8_t pBuffer[] = {M24SR_KILLSESSION};
//     int8_t  status;


//     if (_m24sr_send_i2c_cmd(dev, 0x01, pBuffer) != 0)
//         return M24SR_ERROR_I2CTIMEOUT;

//     /* Insure no access will be done just after open session */
//     /* The only way here is to poll I2C to know when M24SR is ready */
//     /* GPO can not be use with KillSession command */
//     if (_m24sr_poll_i2c(dev) != 0)
//         return M24SR_ERROR_I2CTIMEOUT;

//     return M24SR_ACTION_COMPLETED;
// }








// /**
//   * @brief  This function sends the SelectCCFile command
//     * @retval M24SR_ACTION_COMPLETED : the function is succesful.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   * @retval Status (SW1&SW2) : if operation does not complete for another reason.
//   */
// uint16_t M24SR_SelectCCfile ( void )
// {
//     uint8_t   *pBuffer = uM24SRbuffer ,
//                NbByteToRead = M24SR_STATUSRESPONSE_NBBYTE;
//     uint8_t       uLc = 0x02;
//     uint16_t  status ;
//     uint16_t  uP1P2 = 0x000C,
//               uNbFileId = CC_FILE_ID,
//               NbByte;

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_DEFAULT;
//     Command.Header.INS = C_APDU_SELECT_FILE;
//     /* copy the offset */
//     Command.Header.P1 = GETMSB  (uP1P2 ) ;
//     Command.Header.P2 = GETLSB  (uP1P2 ) ;
//     /* copy the number of byte of the data field */
//     Command.Body.LC = uLc ;
//     /* copy the File Id */
//     Command.Body.pData[0] = GETMSB  (uNbFileId ) ;
//     Command.Body.pData[1] = GETLSB  (uNbFileId ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_SELECTCCFILE,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( NbByteToRead , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, NbByteToRead);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }




// /**
//   * @brief  This function sends the SelectNDEFfile command
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_SelectNDEFfile ( uc16 NDEFfileId )
// {
//     uint8_t   *pBuffer = uM24SRbuffer ,
//                NbByteToRead = M24SR_STATUSRESPONSE_NBBYTE;
//     uint8_t       uLc = 0x02;
//     uint16_t  status ;
//     uint16_t  uP1P2 = 0x000C,
//               NbByte;

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_DEFAULT;
//     Command.Header.INS = C_APDU_SELECT_FILE;
//     /* copy the offset */
//     Command.Header.P1 = GETMSB  (uP1P2 ) ;
//     Command.Header.P2 = GETLSB  (uP1P2 ) ;
//     /* copy the number of byte of the data field */
//     Command.Body.LC = uLc ;
//     /* copy the offset */
//     Command.Body.pData[0] = GETMSB  (NDEFfileId ) ;
//     Command.Body.pData[1] = GETLSB  (NDEFfileId ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_SELECTNDEFFILE,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( NbByteToRead , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, NbByteToRead);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;

// }





// /**
//   * @brief  This function sends a ST read binary command (no error if access is not inside NDEF file)
//     * @param    Offset : first byte to read
//     * @param    NbByteToRead : number of byte to read
//     * @param    pBufferRead : pointer of the buffer read from the M24SR device
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_STReadBinary ( uc16 Offset, uc8 NbByteToRead, uint8_t *pBufferRead )
// {
//     uint8_t   *pBuffer = uM24SRbuffer ;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_ST;
//     Command.Header.INS = C_APDU_READ_BINARY;
//     /* copy the offset */
//     Command.Header.P1 = GETMSB  (Offset ) ;
//     Command.Header.P2 = GETLSB  (Offset ) ;
//     /* copy the number of byte to read */
//     Command.Body.LE = NbByteToRead ;

//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_READBINARY,  Command, &NbByte , pBuffer);

//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     errchk( M24SR_ReceiveI2Cresponse ( NbByteToRead + M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, NbByteToRead + M24SR_STATUSRESPONSE_NBBYTE);
//     /* retrieve the data without SW1 & SW2 as provided as return value of the function */
//     memcpy(pBufferRead , &pBuffer[1], NbByteToRead);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;

// }







// /**
//   * @brief  This function sends the ChangeReferenceData command
//     * @param    uPwdId : PasswordId ( 0x0001 : Read NDEF pwd or 0x0002 : Write NDEF pwd or 0x0003 : I2C pwd)
//     * @param    pPwd : pointer on the passwaord
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_ChangeReferenceData ( uc16 uPwdId, uc8 *pPwd )
// {
//     uint8_t   *pBuffer = uM24SRbuffer;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /*check the parameters */
//     if (uPwdId > 0x0003)
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_DEFAULT;
//     Command.Header.INS = C_APDU_CHANGE;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uPwdId ) ;
//     Command.Header.P2 = GETLSB  (uPwdId ) ;
//     /* copy the number of byte of the data field */
//     Command.Body.LC = M24SR_PASSWORD_NBBYTE ;
//     /* copy the password */
//     memcpy(Command.Body.pData, pPwd, M24SR_PASSWORD_NBBYTE);
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_CHANGEREFDATA,  Command, &NbByte , pBuffer);


//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }


// /**
//   * @brief  This function sends the EnableVerificationRequirement command
//     * @param    uReadOrWrite : enable the read or write protection ( 0x0001 : Read or 0x0002 : Write  )
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_EnableVerificationRequirement ( uc16 uReadOrWrite  )
// {
//     uint8_t   *pBuffer = uM24SRbuffer;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /*check the parameters */
//     if ( (uReadOrWrite != 0x0001) && (uReadOrWrite != 0x0002))
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_DEFAULT;
//     Command.Header.INS = C_APDU_ENABLE;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uReadOrWrite ) ;
//     Command.Header.P2 = GETLSB  (uReadOrWrite ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_ENABLEVERIFREQ,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     /* The right access to be updated in EEPROM need at least 6ms */
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }

// /**
//   * @brief  This function sends the DisableVerificationRequirement command
//     * @param    uReadOrWrite : enable the read or write protection ( 0x0001 : Read or 0x0002 : Write  )
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_DisableVerificationRequirement ( uc16 uReadOrWrite  )
// {
//     uint8_t   *pBuffer = uM24SRbuffer;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /*check the parameters */
//     if ( (uReadOrWrite != 0x0001) && (uReadOrWrite != 0x0002))
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_DEFAULT;
//     Command.Header.INS = C_APDU_DISABLE;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uReadOrWrite ) ;
//     Command.Header.P2 = GETLSB  (uReadOrWrite ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_DISABLEVERIFREQ,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     /* The right access to be updated in EEPROM need at least 6ms */
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }

// /**
//   * @brief  This function sends the EnablePermananentState command
//     * @param    uReadOrWrite : enable the read or write protection ( 0x0001 : Read or 0x0002 : Write  )
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_EnablePermanentState ( uc16 uReadOrWrite  )
// {
//     uint8_t   *pBuffer = uM24SRbuffer;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /*check the parameters */
//     if ( (uReadOrWrite != 0x0001) && (uReadOrWrite != 0x0002))
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_ST;
//     Command.Header.INS = C_APDU_ENABLE;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uReadOrWrite ) ;
//     Command.Header.P2 = GETLSB  (uReadOrWrite ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_ENABLEVERIFREQ,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }

// /**
//   * @brief  This function sends the DisablePermanentState command
//     * @param    uReadOrWrite : enable the read or write protection ( 0x0001 : Read or 0x0002 : Write  )
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_DisablePermanentState ( uc16 uReadOrWrite  )
// {
//     uint8_t   *pBuffer = uM24SRbuffer;
//     uint16_t  status ;
//     uint16_t  NbByte;

//     /*check the parameters */
//     if ( (uReadOrWrite != 0x0001) && (uReadOrWrite != 0x0002))
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_ST;
//     Command.Header.INS = C_APDU_DISABLE;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uReadOrWrite ) ;
//     Command.Header.P2 = GETLSB  (uReadOrWrite ) ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_DISABLEVERIFREQ,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }

// /**
//   * @brief  This function generates a interrupt on GPO pin
//     * @param    None
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_SendInterrupt ( void  )
// {
//     uint8_t     *pBuffer = uM24SRbuffer;
//     uint16_t    uP1P2 = 0x001E;
//     uint16_t    status ;
//     uint16_t    NbByte;

//     status = M24SR_ManageI2CGPO( INTERRUPT);

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_ST;
//     Command.Header.INS = C_APDU_INTERRUPT;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uP1P2 ) ;
//     Command.Header.P2 = GETLSB  (uP1P2 ) ;
//     Command.Body.LC = 0x00 ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_SENDINTERRUPT,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }

// /**
//   * @brief  This function force GPO pin to low state or high Z
//     * @param    uSetOrReset : select if GPO must be low (reset) or HiZ
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//     * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
//   */
// uint16_t M24SR_StateControl ( uc8 uSetOrReset )
// {
//     uint8_t     *pBuffer = uM24SRbuffer;
//     uint16_t    uP1P2 = 0x001F;
//     uint16_t    status ;
//     uint16_t    NbByte;

//     /*check the parameters */
//     if ( (uSetOrReset != 0x01) && (uSetOrReset != 0x00))
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     status = M24SR_ManageI2CGPO( STATE_CONTROL);

//     /* build the command */
//     Command.Header.CLA = C_APDU_CLA_ST;
//     Command.Header.INS = C_APDU_INTERRUPT;
//     /* copy the Password Id */
//     Command.Header.P1 = GETMSB  (uP1P2 ) ;
//     Command.Header.P2 = GETLSB  (uP1P2 ) ;
//     Command.Body.LC = 0x01 ;
//     /* copy the data */
//     memcpy(Command.Body.pData , &uSetOrReset, 0x01 );
//     //Command.Body.LE = 0x00 ;
//     /* build the I�C command */
//     M24SR_BuildIBlockCommand ( M24SR_CMDSTRUCT_GPOSTATE,  Command, &NbByte , pBuffer);

//     /* send the request */
//     errchk( M24SR_SendI2Ccommand ( NbByte , pBuffer ));
//     errchk( M24SR_IsAnswerReady ( ));
//     /* read the response */
//     errchk( M24SR_ReceiveI2Cresponse ( M24SR_STATUSRESPONSE_NBBYTE , pBuffer ));

//     status = M24SR_IsCorrectCRC16Residue (pBuffer, M24SR_STATUSRESPONSE_NBBYTE);
//     return status;

// Error :
//     return M24SR_ERROR_I2CTIMEOUT;
// }



// /**
//   * @brief  This function configure GPO purpose for RF session
//     * @param    GPO_RFconfig: GPO configuration to set
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//   */
// uint16_t M24SR_ManageRFGPO( uc8 GPO_RFconfig)
// {
//     uint16_t status;
//     uint8_t GPO_config;
//     uint8_t DefaultPassword[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//                                   };

//     if ( GPO_RFconfig > STATE_CONTROL)
//     {
//         return M24SR_ERROR_PARAMETER;
//     }

//     M24SR_SelectApplication();
//     M24SR_SelectSystemfile();
//     M24SR_ReadBinary ( 0x0004 , 0x01 , &GPO_config );

//     /* Update only GPO purpose for I2C */
//     GPO_config = (GPO_config & 0x0F) | (GPO_RFconfig << 4);
//     M24SR_SelectSystemfile();
//     M24SR_Verify( I2C_PWD , 0x10 , DefaultPassword );
//     status = M24SR_UpdateBinary ( 0x0004 , 0x01, &(GPO_config) );

//     return status;
// }


// /**
//   * @brief  This function enable or disable RF communication
//     * @param    OnOffChoice: GPO configuration to set
//   * @retval Status (SW1&SW2) : Status of the operation to complete.
//   */
// void M24SR_RFConfig( uc8 OnOffChoice)
// {
//     M24SR_RFConfig_Hard(OnOffChoice);
// }




// uint8_t I2CPassword[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//                           };

// static uint32_t NDEFSessionOpenID = NDEF_SESSION_CLOSED;

// /* Init NDEF_FileID with bad value in case Init failed */
// static uint16_t NDEF_FileID = 0xDEAD;

// #ifdef __MBED__
// extern void wait_ms(int ms);
// #endif

// /** @defgroup libM24SR_Private_Functions
//   * @{
//   */

// /**
//   * @}
//   */


// /** @defgroup libM24SR_Public_Functions
//   * @{
//   */



// /**
//   * @brief  This fonction retrieve the NDEF file ID of NDEF file present in M24SR
//   * @param  NDEF_fileID : To store NDEF ID
//   * @retval SUCCESS : File ID read
//   * @retval ERROR : Not able to read file ID.
//   */
// uint16_t M24SR_GetNDEFFileId ( uint16_t *NDEF_fileID )
// {
//     if ( NDEF_FileID != 0xDEAD)
//     {
//         *NDEF_fileID = NDEF_FileID;
//         return SUCCESS;
//     }
//     else
//     {
//         return ERROR;
//     }
// }


// /**
//   * @brief  This fonction configure the M24SR to access NDEF message by I2C
//   * @param  NDEF_fileID : NDEF identification to select NDEF in M24SR
//   * @param  Priority: 2 options: check if M24SR available to open session (no RF session on going)
//   *                              Kill RF session and open I2C sesssion.
//   * @retval SUCCESS : Session is opened
//   * @retval ERROR : Not able to open session.
//   */
// uint16_t M24SR_OpenNDEFSession ( uint16_t NDEF_fileID, uint16_t Priority )
// {
//     uint16_t status = ERROR;
//     uint16_t trials = 5; /* wait 1sec, driver is configured to let 200ms for command to complete */
//     /* which is enough for all commands except GetSession if RF session is already opened */
//     /* Smartphone generaly release the session within the second, but customer can modify this value */

//     if (NDEFSessionOpenID == NDEF_SESSION_CLOSED)
//     {
//         if ( Priority == TAKE_SESSION)
//         {
//             status = M24SR_KillSession();
//         }
//         else
//         {
//             while ( status != M24SR_ACTION_COMPLETED && trials)
//             {
//                 status = M24SR_GetSession();
//                 trials--;
//             }
//         }
//         if (status != M24SR_ACTION_COMPLETED)
//         {
//             /* seems session already open on RF side */
//             /* But in case of I2C issue try to init again */
//             M24SR_Init();
//             return ERROR;
//         }

//         /*===================================*/
//         /* Select the NFC type 4 application */
//         /*===================================*/
//         errorchk( M24SR_SelectApplication() );

//         /*====================*/
//         /* select NDEF file   */
//         /*====================*/
//         errorchk( M24SR_SelectNDEFfile(NDEF_fileID) );

//         NDEFSessionOpenID = (uint32_t)(NDEF_fileID);

//         return SUCCESS;
//     }
//     else if (NDEFSessionOpenID == NDEF_fileID)
//     {
//         /* Session already Open not an issue caller can perform access in NDEF file */
//         return SUCCESS;
//     }

// Error:
//     return ERROR;
// }

// /**
//   * @brief  This fonction close the NDEF Session.
//   * @param  NDEF_fileID : NDEF identification to select NDEF in M24SR
//   * @retval SUCCESS : Session is closed
//   * @retval ERROR : Not able to close session.
//   */
// uint16_t M24SR_CloseNDEFSession ( uint16_t NDEF_fileID )
// {
//     uint16_t status = ERROR;

//     if (NDEFSessionOpenID == (uint32_t)(NDEF_fileID))
//     {
//         errorchk( M24SR_Deselect () );
//         NDEFSessionOpenID = NDEF_SESSION_CLOSED;

//         return SUCCESS;
//     }
//     else if (NDEFSessionOpenID == NDEF_SESSION_CLOSED)
//     {
//         /* Not an error as session is already closed */
//         return SUCCESS;
//     }

// Error:
//     return ERROR;
// }

// /**
//   * @brief  This fonction read the data stored in M24SR at defined offset
//   * @param  Offset : Offset in the NDEF file in M24SR
//   * @param  DataSize : Number of byte to read
//   * @param  pData : pointer on buffer to store read data
//   * @retval Status (SW1&SW2) : Status of the operation.
//   */
// uint16_t M24SR_ReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData)
// {
//     uint16_t status;

//     if ( DataSize > M24SR_READ_MAX_NBBYTE)
//     {
//         do
//         {
//             status = M24SR_ReadBinary ( Offset, M24SR_READ_MAX_NBBYTE , pData);
//             Offset += M24SR_READ_MAX_NBBYTE;
//             pData += M24SR_READ_MAX_NBBYTE;
//             DataSize -= M24SR_READ_MAX_NBBYTE;
//         } while ( DataSize > M24SR_READ_MAX_NBBYTE && status == M24SR_ACTION_COMPLETED);
//         if ( status == M24SR_ACTION_COMPLETED && DataSize)
//             status = M24SR_ReadBinary ( Offset, (uint8_t)(DataSize) , pData);
//     }
//     else
//         status = M24SR_ReadBinary ( Offset, (uint8_t)(DataSize) , pData);

//     return status;
// }

// /**
//   * @brief  This fonction read the data stored in M24SR at defined offset without NDEF concideration
//   * @param  Offset : Offset in the NDEF file in M24SR
//   * @param  DataSize : Number of byte to read
//   * @param  pData : pointer on buffer to store read data
//   * @retval Status (SW1&SW2) : Status of the operation.
//   */
// uint16_t M24SR_ForceReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData)
// {
//     uint16_t status;

//     if ( DataSize > M24SR_READ_MAX_NBBYTE)
//     {
//         do
//         {
//             status = M24SR_STReadBinary ( Offset, M24SR_READ_MAX_NBBYTE , pData);
//             Offset += M24SR_READ_MAX_NBBYTE;
//             pData += M24SR_READ_MAX_NBBYTE;
//             DataSize -= M24SR_READ_MAX_NBBYTE;
//         } while ( DataSize > M24SR_READ_MAX_NBBYTE && status == M24SR_ACTION_COMPLETED);
//         if ( status == M24SR_ACTION_COMPLETED && DataSize)
//             status = M24SR_STReadBinary ( Offset, (uint8_t)(DataSize) , pData);
//     }
//     else
//         status = M24SR_STReadBinary ( Offset, (uint8_t)(DataSize) , pData);

//     return status;
// }

// /**
//   * @brief  This fonction write data in M24SR at defined offset
//   * @param  Offset : Offset in the NDEF file in M24SR
//   * @param  DataSize : Number of byte to read
//   * @param  pData : pointer on buffer to copy in M24SR
//   * @retval Status (SW1&SW2) : Status of the operation.
//   */
// uint16_t M24SR_WriteData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData)
// {
//     uint16_t status;

//     if ( DataSize > M24SR_WRITE_MAX_NBBYTE)
//     {
//         do
//         {
//             status = M24SR_UpdateBinary ( Offset, M24SR_WRITE_MAX_NBBYTE , pData);
//             Offset += M24SR_WRITE_MAX_NBBYTE;
//             pData += M24SR_WRITE_MAX_NBBYTE;
//             DataSize -= M24SR_WRITE_MAX_NBBYTE;
//         } while ( DataSize > M24SR_WRITE_MAX_NBBYTE && status == M24SR_ACTION_COMPLETED);
//         if ( status == M24SR_ACTION_COMPLETED && DataSize)
//             status = M24SR_UpdateBinary ( Offset, (uint8_t)(DataSize) , pData);
//     }
//     else
//         status = M24SR_UpdateBinary ( Offset, (uint8_t)(DataSize) , pData);

//     return status;
// }

// /**
//   * @brief  This fonction activate the need of a password for next read access
//   * @param  pCurrentWritePassword : Write password is needed to have the right to enable Read Password
//   * @param  pNewPassword : The password that will be requiered for next read access
//   * @retval SUCCESS : Read password is activated
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_EnableReadPassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         /* Set new password */
//         M24SR_ChangeReferenceData ( READ_PWD, pNewPassword );
//         M24SR_EnableVerificationRequirement( READ_PWD );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction desactivate the need of a password for next read access
//   * @param  pCurrentWritePassword : Write password is needed to have the right to disable Read Password
//   * @retval SUCCESS : Read password is desactivated
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_DisableReadPassword( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         /* Set new password */
//         M24SR_DisableVerificationRequirement( READ_PWD );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction activate the need of a password for next write access
//   * @param  pCurrentWritePassword : Write password must be prensented to have the right to modify write Password
//   * @param  pNewPassword : The password that will be requiered for next write access
//   * @retval SUCCESS : Write password is activated
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_EnableWritePassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword)
// {
//     uint16_t status;

//     /* check we have the good password */
//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         /* Set new password */
//         M24SR_ChangeReferenceData ( WRITE_PWD, pNewPassword );
//         M24SR_EnableVerificationRequirement( WRITE_PWD );
//         status = SUCCESS;
//     }
//     else /* we don't have the good password */
//     {
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction desactivate the need of a password for next write access
//   * @param  pCurrentWritePassword : Write password must be prensented to have the right to disable it
//   * @retval SUCCESS : Write password is desactivated
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_DisableWritePassword( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_DisableVerificationRequirement( WRITE_PWD );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction desactivate the need of read and write password for next access
//   * @param  pSuperUserPassword : I2C super user password to overwrite read and write password
//   * @retval SUCCESS : M24SR access is now free (no password needed)
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_DisableAllPassword( uint8_t* pSuperUserPassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( I2C_PWD , 0x10 , pSuperUserPassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_DisablePermanentState( READ_PWD );
//         M24SR_DisablePermanentState( WRITE_PWD );

//         M24SR_DisableVerificationRequirement( READ_PWD );
//         M24SR_DisableVerificationRequirement( WRITE_PWD );

//         /* reset password */
//         M24SR_ChangeReferenceData ( READ_PWD, pSuperUserPassword );
//         M24SR_ChangeReferenceData ( WRITE_PWD, pSuperUserPassword );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction enable read only mode
//   * @param  pCurrentWritePassword : Write password is needed to have right to enable read only mode
//   * @retval SUCCESS : M24SR access is now forbidden in write mode
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_EnableReadOnly( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_EnablePermanentState( WRITE_PWD ); /* lock write to have read only */
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction disable read only mode
//   * @param  pCurrentWritePassword : Write password is needed to have right to disable read only mode
//   * @retval SUCCESS : M24SR write access is now allowed
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_DisableReadOnly( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( I2C_PWD , 0x10 , I2CPassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_DisablePermanentState( WRITE_PWD ); /* disable write protection to disable read only mode */
//         M24SR_DisableVerificationRequirement( WRITE_PWD );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* we don't have the good I2C password nothing to do anymore */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction enable write only mode
//   * @param  pCurrentWritePassword : Write password is needed to have right to enable write only mode
//   * @retval SUCCESS : M24SR access is now forbidden in read mode
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_EnableWriteOnly( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( WRITE_PWD , 0x10 , pCurrentWritePassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_EnablePermanentState( READ_PWD ); /* disable read access and keep write */
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This fonction disable write only mode
//   * @param  pCurrentWritePassword : Write password is needed to have right to disable write only mode
//   * @retval SUCCESS : M24SR read access is now allowed
//   * @retval ERROR : operation does not complete
//   */
// uint16_t M24SR_DisableWriteOnly( uint8_t* pCurrentWritePassword)
// {
//     uint16_t status = SUCCESS;

//     if (M24SR_Verify( I2C_PWD , 0x10 , I2CPassword ) == M24SR_PWD_CORRECT)
//     {
//         M24SR_DisablePermanentState( READ_PWD ); /* disable write only -> enable write acces */
//         M24SR_DisableVerificationRequirement( READ_PWD );
//         status = SUCCESS;
//     }
//     else
//     {
//         /* M24SR already lock but password not known */
//         status = ERROR;
//     }

//     return status;
// }

// /**
//   * @brief  This function configure GPO purpose for RF session
//   * @param  GPO_config: GPO configuration to set
//   * @param  mode: select RF or I2C, GPO config to update
//   * @retval Status : Status of the operation.
//   */
// uint16_t M24SR_ManageGPO( uc8 GPO_config, uc8 mode)
// {
//     uint16_t status;

//     if ( mode == RF_GPO)
//     {
//         status = M24SR_ManageRFGPO ( GPO_config );
//     }
//     else
//     {
//         status = M24SR_ManageI2CGPO ( GPO_config );
//     }
//     return status;
// }
// /**
//   * @}
//   */

// /**
//   * @}
//   */

// /**
//   * @}
//   */





