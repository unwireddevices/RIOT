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
 * @brief       Definition for the M24SRxx NFC memory
 * 
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
 
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _M24SR_H
#define _M24SR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#include "periph/i2c.h"
#include "periph/gpio.h"


/**
 * @brief   LIS2HH12 configuration parameters
 */
typedef struct {
    i2c_t   i2c;                 /**< I2C device   */
    uint8_t i2c_addr;            /**< I2C address */
    gpio_t  gpo_pin;             /**< Interrupt GPO   */
    gpio_flank_t gpo_flank      /**< Interrupt GPO flank*/
    gpio_t  rfdisable_pin;            /**< GPIO to switch RF on/off */
    gpio_t  pwr_en_pin;         /**< GPIO to switch power on/off*/
} m24sr_params_t;



typedef enum {
  I2C_TOKEN_RELEASE_HW = 0,
  I2C_TOKEN_RELEASE_SW,
  I2C_TOKEN_REALISE_NUM
} m24sr_token_mode_t;

/**
 * @brief   LIS2HH12 device descriptor
 */
typedef struct {
    m24sr_params_t params;     /**< device configuration */
} m24sr_t;

/**
 * @brief   Status and error return codes
 */
enum {
    M24SR_OK    =  0,            /**< everything was fine */
    M24SR_NOBUS = -1,            /**< bus interface error */
    M24SR_NODEV = -2,            /**< unable to talk to device */
};


/* Exported types ------------------------------------------------------------*/

/**
  * @brief  APDU-Header command structure
  */
typedef struct
{
    uint8_t CLA;                            /**< Command class */
    uint8_t INS;                            /**< Operation code */
    uint8_t P1;                             /**< Selection Mode */
    uint8_t P2;                             /**< Selection Option */
} cmd_apdu_header_t;


/**
  * @brief  APDU-Body command structure
  */
typedef struct 
{
    uint8_t LC;                             /**< Data field length */ 
    uint8_t *data ;                        /**< Command parameters */ 
    uint8_t LE;                             /**< Expected length of data to be returned */
} cmd_apdu_body_t;

/**
  * @brief  APDU Command structure 
  */
typedef struct
{
    cmd_apdu_header_t header;
    cmd_apdu_body_t   body;
} cmd_apdu_t;

/**
  * @brief  SC response structure
  */
typedef struct
{
    uint8_t *pData ;                        /**< Data returned from the card */
    uint8_t SW1;                            /**< Command Processing status */
    uint8_t SW2;                            /**< Command Processing qualification */
} R_APDU;

/**
  * @brief  GPO mode structure 
  */
typedef enum{
    RF_GPO= 0,
    I2C_GPO
}M24SR_GPO_MODE;

/**
  * @brief  GPO state structure 
  */
typedef enum{
    HIGH_IMPEDANCE = 0,
    SESSION_OPENED,
    WIP,
    I2C_ANSWER_READY,
    INTERRUPT,
    STATE_CONTROL
} m24sr_gpo_mode_t;

/* Exported constants --------------------------------------------------------*/

/** @defgroup M24SR_Exported_Constants
  * @{
  */

/** @defgroup M24SR_EEPROM_Size_Version
  * @{
  */     
/* ---------------------- M24SR properties -----------------------------------*/
#define M24SR02_NDEF_MAX_SIZE                               0x0100
#define M24SR04_NDEF_MAX_SIZE                               0x0200
#define M24SR16_NDEF_MAX_SIZE                               0x0800
#define M24SR64_NDEF_MAX_SIZE                               0x2000   
/**
  * @}
  */

/** @defgroup M24SR_Flag_to_select_open_session_command
  * @{
  */     
#define ASK_FOR_SESSION                                     0x0000
#define TAKE_SESSION                                        0xFFFF   
/**
  * @}
  */

/* M24SR buffer size is 0xF6 can be retrieve dynamicaly in CC file */
#define M24SR_READ_MAX_NBBYTE                               0xF6 
#define M24SR_WRITE_MAX_NBBYTE                              0xF6


#define NDEF_SESSION_CLOSED                                 0xDEADBEEF


/* ---------------------- status code ----------------------------------------*/
#define M24SR_ACTION_COMPLETED                              0x9000
#define UB_STATUS_OFFSET                                    4
#define LB_STATUS_OFFSET                                    3

#define M24SR_NBBYTE_INVALID                                0xFFFE

/** @defgroup M24SR_File_Identifier
  * @{
  */     
#define SYSTEM_FILE_ID                                      0xE101   
#define CC_FILE_ID                                          0xE103
#define NDEF_FILE_ID                                        0x0001   
/**
  * @}
  */
    
/** @defgroup M24SR_Password_Management
  * @{
  */
#define READ_PWD                                            0x0001
#define WRITE_PWD                                           0x0002
#define I2C_PWD                                             0x0003

/*-------------------------- Verify command answer ----------------------------*/
#define M24SR_PWD_NOT_NEEDED                                0x9000
#define M24SR_PWD_NEEDED                                    0x6300
#define M24SR_PWD_CORRECT                                   0x9000
/**
  * @}
  */

     
/** @defgroup M24SR_Command_Management
  * @{
  */    
    
/* special M24SR command ----------------------------------------------------------------------*/    
#define M24SR_OPENSESSION                                   0x26
#define M24SR_KILLSESSION                                   0x52

/* APDU Command: class list -------------------------------------------*/
#define C_APDU_CLA_DEFAULT                                  0x00
#define C_APDU_CLA_ST                                       0xA2

/*------------------------ Data Area Management Commands ---------------------*/
#define C_APDU_SELECT_FILE                                  0xA4
#define C_APDU_GET_RESPONCE                                 0xC0
#define C_APDU_STATUS                                       0xF2
#define C_APDU_UPDATE_BINARY                                0xD6
#define C_APDU_READ_BINARY                                  0xB0
#define C_APDU_WRITE_BINARY                                 0xD0
#define C_APDU_UPDATE_RECORD                                0xDC
#define C_APDU_READ_RECORD                                  0xB2

/*-------------------------- Safety Management Commands ----------------------*/
#define C_APDU_VERIFY                                       0x20
#define C_APDU_CHANGE                                       0x24
#define C_APDU_DISABLE                                      0x26
#define C_APDU_ENABLE                                       0x28

/*-------------------------- Gpio Management Commands ------------------------*/
#define C_APDU_INTERRUPT                                    0xD6

/*  Length  ----------------------------------------------------------------------------------*/
#define M24SR_STATUS_NBBYTE                                 2
#define M24SR_CRC_NBBYTE                                    2
#define M24SR_STATUSRESPONSE_NBBYTE                         5
#define M24SR_DESELECTREQUEST_NBBYTE                        3
#define M24SR_DESELECTRESPONSE_NBBYTE                       3
#define M24SR_WATINGTIMEEXTRESPONSE_NBBYTE                  4
#define M24SR_PASSWORD_NBBYTE                               0x10

/*  Command structure   ------------------------------------------------------------------------*/
#define M24SR_CMDSTRUCT_SELECTAPPLICATION                   0x01FF
#define M24SR_CMDSTRUCT_SELECTCCFILE                        0x017F
#define M24SR_CMDSTRUCT_SELECTNDEFFILE                      0x017F
#define M24SR_CMDSTRUCT_READBINARY                          0x019F
#define M24SR_CMDSTRUCT_UPDATEBINARY                        0x017F
#define M24SR_CMDSTRUCT_VERIFYBINARYWOPWD                   0x013F
#define M24SR_CMDSTRUCT_VERIFYBINARYWITHPWD                 0x017F
#define M24SR_CMDSTRUCT_CHANGEREFDATA                       0x017F
#define M24SR_CMDSTRUCT_ENABLEVERIFREQ                      0x011F
#define M24SR_CMDSTRUCT_DISABLEVERIFREQ                     0x011F
#define M24SR_CMDSTRUCT_SENDINTERRUPT                       0x013F
#define M24SR_CMDSTRUCT_GPOSTATE                            0x017F

/*  Command structure Mask -------------------------------------------------------------------*/
#define M24SR_PCB_NEEDED                                    0x0001      /* PCB byte present or not */
#define M24SR_CLA_NEEDED                                    0x0002      /* CLA byte present or not */
#define M24SR_INS_NEEDED                                    0x0004      /* Operation code present or not*/ 
#define M24SR_P1_NEEDED                                     0x0008      /* Selection Mode  present or not*/
#define M24SR_P2_NEEDED                                     0x0010      /* Selection Option present or not*/
#define M24SR_LC_NEEDED                                     0x0020      /* Data field length byte present or not */
#define M24SR_DATA_NEEDED                                   0x0040      /* Data present or not */
#define M24SR_LE_NEEDED                                     0x0080      /* Expected length present or not */
#define M24SR_CRC_NEEDED                                    0x0100      /* 2 CRC bytes present  or not */

#define M24SR_DID_NEEDED                                    0x08            /* DID byte present or not */

/**
  * @}
  */
    
    

/*  Offset  ----------------------------------------------------------------------------------*/
#define M24SR_OFFSET_PCB                                    0
#define M24SR_OFFSET_CLASS                                  1
#define M24SR_OFFSET_INS                                    2
#define M24SR_OFFSET_P1                                     3


/*  mask    ------------------------------------------------------------------------------------*/
#define M24SR_MASK_BLOCK                                    0xC0
#define M24SR_MASK_IBLOCK                                   0x00
#define M24SR_MASK_RBLOCK                                   0x80
#define M24SR_MASK_SBLOCK                                   0xC0

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @brief Get Most Significant Byte
  * @param  val: number where MSB must be extracted
  * @retval MSB
  */ 
#define GETMSB(val)                                         ( (uint8_t) ((val & 0xFF00 )>>8) ) 

/** @brief Get Least Significant Byte
  * @param  val: number where LSB must be extracted
  * @retval LSB
  */ 
#define GETLSB(val)                                         ( (uint8_t) (val & 0x00FF )) 

/** @brief Used to toggle the block number by adding 0 or 1 to default block number value
  * @param  val: number to know if incrementation is needed
  * @retval  0 or 1 if incrementation needed
  */
#define TOGGLE(val)                                         ((val != 0x00)? 0x00 : 0x01)


/*  public function --------------------------------------------------------------------------*/

void     M24SR_Init (void);
uint16_t M24SR_GetSession (void);
uint16_t M24SR_KillSession (void);
uint16_t M24SR_Deselect (void);
uint16_t M24SR_SelectApplication (void);
uint16_t M24SR_SelectCCfile (void);
uint16_t M24SR_SelectNDEFfile (uint16_t NDEFfileId);
uint16_t M24SR_SelectSystemfile (void);
uint16_t M24SR_ReadBinary (uint16_t Offset, uint8_t NbByteToRead, uint8_t *pBufferRead);
uint16_t M24SR_STReadBinary (uint16_t Offset, uint8_t NbByteToRead, uint8_t *pBufferRead);
uint16_t M24SR_UpdateBinary (uint16_t Offset, uint8_t NbByteToWrite, uint8_t *pDataToWrite);
uint16_t M24SR_Verify (uint16_t uPwdId, uint8_t NbPwdByte, uint8_t *pPwd);
uint16_t M24SR_ChangeReferenceData (uint16_t uPwdId, uint8_t *pPwd);
uint16_t M24SR_EnableVerificationRequirement (uint16_t uReadOrWrite);
uint16_t M24SR_DisableVerificationRequirement (uint16_t uReadOrWrite);
uint16_t M24SR_EnablePermanentState (uint16_t uReadOrWrite);
uint16_t M24SR_DisablePermanentState (uint16_t uReadOrWrite);
uint16_t M24SR_SendInterrupt (void);
uint16_t M24SR_StateControl (uint8_t uSetOrReset);
uint16_t M24SR_ManageI2CGPO (uint8_t GPO_I2Cconfig);
uint16_t M24SR_ManageRFGPO (uint8_t GPO_RFconfig);
void     M24SR_RFConfig (uint8_t OnOffChoice);





uint16_t M24SR_Initialization (uint8_t* pCCBuffer, uint8_t size );
    
uint16_t M24SR_GetNDEFFileId ( uint16_t *NDEF_fileID );
uint16_t M24SR_OpenNDEFSession ( uint16_t NDEF_fileID, uint16_t Priority );
uint16_t M24SR_ReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t M24SR_ForceReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t M24SR_WriteData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t M24SR_CloseNDEFSession ( uint16_t NDEF_fileID );   

uint16_t M24SR_EnableReadPassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword);  
uint16_t M24SR_DisableReadPassword( uint8_t* pCurrentWritePassword );   
uint16_t M24SR_EnableWritePassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword); 
uint16_t M24SR_DisableWritePassword( uint8_t* pCurrentWritePassword );
uint16_t M24SR_DisableAllPassword( uint8_t* pSuperUserPassword);

uint16_t M24SR_EnableReadOnly( uint8_t* pCurrentWritePassword); 
uint16_t M24SR_DisableReadOnly( uint8_t* pCurrentWritePassword);    
uint16_t M24SR_EnableWriteOnly( uint8_t* pCurrentWritePassword);    
uint16_t M24SR_DisableWriteOnly( uint8_t* pCurrentWritePassword);

uint16_t M24SR_ManageGPO( uint8_t GPO_config, uint8_t mode);    


/** @addtogroup M24SR_I2C
  * @{
  */
    
/**
  * @brief  Synchronization Mechanism structure 
  */
typedef enum{
    M24SR_WAITINGTIME_UNKNOWN = 0,
    M24SR_WAITINGTIME_POLLING,
    M24SR_WAITINGTIME_TIMEOUT,
    M24SR_WAITINGTIME_GPO,
    M24SR_INTERRUPT_GPO
}m24sr_waiting_time_mode_t;    



/** @defgroup M24SR_I2C_Acces_Configuration 
  * @{
  */
#define M24SR_I2C_TIMEOUT       200 /* I2C Time out (ms), this is the maximum time needed by M24SR to complete any command */
#define M24SR_I2C_POLLING       1 /* In case M24SR will reply ACK failed allow to perform retry before returning error (HAL option not used) */
#define M24SR_ADDR              0xAC   /*!< M24SR address */
/**
  * @}
  */

/** @defgroup M24SR_I2C_Error_Code_From_M24SR
  * @{
  */
/* error code ---------------------------------------------------------------------------------*/
#define M24SR_ERRORCODE_FILEOVERFLOW                        0x6280
#define M24SR_ERRORCODE_ENDOFFILE                           0x6282
#define M24SR_ERRORCODE_PASSWORDREQUIRED                    0x63C0
#define M24SR_ERRORCODE_PASSWORDINCORRECT2RETRY             0x63C2
#define M24SR_ERRORCODE_PASSWORDINCORRECT1RETRY             0x63C1
#define M24SR_ERRORCODE_RFSESSIONKILLED                     0x6500
#define M24SR_ERRORCODE_UNSUCCESSFULUPDATING                0x6581
#define M24SR_ERRORCODE_WRONGHLENGTH                        0x6700
#define M24SR_ERRORCODE_COMMANDINCORRECT                    0x6981
#define M24SR_ERRORCODE_SECURITYSTATUS                      0x6982
#define M24SR_ERRORCODE_REFERENCEDATANOTUSABLE              0x6984
#define M24SR_ERRORCODE_INCORRECTPARAMETER                  0x6A80
#define M24SR_ERRORCODE_FILENOTFOUND                        0x6A82
#define M24SR_ERRORCODE_FILEOVERFLOWLC                      0x6A84
#define M24SR_ERRORCODE_INCORRECTP1P2                       0x6A86
#define M24SR_ERRORCODE_INSNOTSUPPORTED                     0x6D00
#define M24SR_ERRORCODE_CLASSNOTSUPPORTED                   0x6E00
#define M24SR_ERRORCODE_DAFAULT                             0x6F00
/**
  * @}
  */

/** @defgroup M24SR_I2C_Error_Code_From_M24SR_SW
  * @{
  */
/* Status and error code -----------------------------------------------------*/     
#define M24SR_STATUS_SUCCESS                                0x0000
#define M24SR_ERROR_DEFAULT                                 0x0010
#define M24SR_ERROR_I2CTIMEOUT                              0x0011
#define M24SR_ERROR_CRC                                     0x0012
#define M24SR_ERROR_NACK                                    0x0013
#define M24SR_ERROR_PARAMETER                               0x0014 
#define M24SR_ERROR_NBATEMPT                                0x0015 
#define M24SR_ERROR_NOACKNOWLEDGE                           0x0016
/**
  * @}
  */

void          M24SR_I2CInit                                 ( void );
void          M24SR_GPOInit                                 ( void );
void          M24SR_WaitMs                          ( uint32_t time_ms );
void          M24SR_GetTick                         ( uint32_t *ptickstart );
void          M24SR_GPO_ReadPin                     ( uint8_t *pPinState);
void          M24SR_RFDIS_WritePin                  ( uint8_t PinState);
void            M24SR_SetI2CSynchroMode                         ( uint8_t mode );
int8_t        M24SR_I2CTokenRelease                 ( void );
int8_t        M24SR_SendI2Ccommand                                  ( uint8_t NbByte , uint8_t *pBuffer );
int8_t        M24SR_IsAnswerReady                               ( void );
int8_t        M24SR_PollI2C                                                 ( void );
int8_t        M24SR_ReceiveI2Cresponse                          ( uint8_t NbByte , uint8_t *pBuffer );
void          M24SR_RFConfig_Hard                                       ( uint8_t OnOffChoice);

/**
  * @}
  */
    
#ifdef __cplusplus
}
#endif

#endif /* _M24SR_H */