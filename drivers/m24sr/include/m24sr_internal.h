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
 
#ifndef _M24SR_INTERNAL_H
#define _M24SR_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------- M24SR properties -----------------------------------*/
#define M24SR02_NDEF_MAX_SIZE                               0x0100
#define M24SR04_NDEF_MAX_SIZE                               0x0200
#define M24SR16_NDEF_MAX_SIZE                               0x0800
#define M24SR64_NDEF_MAX_SIZE                               0x2000   

#define ASK_FOR_SESSION                                     0x0000
#define TAKE_SESSION                                        0xFFFF
#define RELEASE_SESSION                                     0xBABE   

/* M24SR buffer size is 0xF6 can be retrieve dynamicaly in CC file */
#define M24SR_READ_MAX_NUM_BYTE                             0xF6 
#define M24SR_WRITE_MAX_NUM_BYTE                            0xF6


#define NDEF_SESSION_CLOSED                                 0xDEADBEEF


/* ---------------------- status code ----------------------------------------*/
#define M24SR_ACTION_COMPLETED                              0x9000
#define UB_STATUS_OFFSET                                    4
#define LB_STATUS_OFFSET                                    3

#define M24SR_NUM_BYTE_INVALID                              0xFFFE

/* ---------------------- file indetifier ------------------------------------*/
#define M24SR_SYS_FILE_ID                                   0xE101   
#define M24SR_CC_FILE_ID                                    0xE103
#define M24SR_NDEF_FILE_ID                                  0x0001   

    
#define READ_PWD                                            0x0001
#define WRITE_PWD                                           0x0002
#define I2C_PWD                                             0x0003

/* special M24SR command -----------------------------------------------------*/    
#define M24SR_OPEN_I2C_SESSION                              0x26
#define M24SR_KILL_RF_SESSION                               0x52

/* APDU Command: class list --------------------------------------------------*/
#define CLA_DEFAULT                                         0x00
#define CLA_STM                                             0xA2

/*------------------------ Data Area Management Commands ---------------------*/
#define INS_SELECT                                          0xA4
#define INS_GET_RESPONSE                                    0xC0
#define INS_UPDATE_BINARY                                   0xD6
#define INS_READ_BINARY                                     0xB0
#define INS_WRITE_BINARY                                    0xD0
#define INS_UPDATE_RECORD                                   0xDC
#define INS_READ_RECORD                                     0xB2

/*-------------------------- Safety Management Commands ----------------------*/
#define INS_VERIFY                                          0x20
#define INS_CHANGE_REF_DATA                                 0x24
#define INS_DISABLE_VERIFY_REQ                              0x26
#define INS_ENABLE_VERIFY_REQ                               0x28

/* error code ----------------------------------------------------------------*/
#define SW_OK                                               0x9000
#define SW_FILE_OVERFLOW                                    0x6280
#define SW_END_OF_FILE                                      0x6282
#define SW_PASSWORD_REQUIRED                                0x6300
#define SW_PASSWORD_INCORRECT_2_RETRY                       0x63C2
#define SW_PASSWORD_INCORRECT_1_RETRY                       0x63C1
#define SW_PASSWORD_INCORRECT_0_RETRY                       0x63C0
#define SW_RF_SESSION_KILLED                                0x6500
#define SW_UNSUCCESSFUL_UPDATING                            0x6581
#define SW_WRONGH_LENGTH                                    0x6700
#define SW_COMMAND_INCORRECT                                0x6981
#define SW_SECURITY_STATUS                                  0x6982
#define SW_REFERENCE_DATA_NOT_USABLE                        0x6984
#define SW_INCORRECT_PARAMETER                              0x6A80
#define SW_FILE_NOT_FOUND                                   0x6A82
#define SW_FILE_OVERFLOW_LC                                 0x6A84
#define SW_INCORRECT_P1_P2                                  0x6A86
#define SW_INS_NOT_SUPPORTED                                0x6D00
#define SW_CLASS_NOT_SUPPORTED                              0x6E00
#define SW_FAULT                                            0x6F00

/*  Length  ------------------------------------------------------------------*/
#define M24SR_STATUS_NUM_BYTE                               2
#define M24SR_CRC_NUM_BYTE                                  2
#define M24SR_STATUS_RESPONSE_NUM_BYTE                      5
#define M24SR_DESELECT_REQUEST_NUM_BYTE                     3
#define M24SR_DESELECT_RESPONSE_NUM_BYTE                    3
#define M24SR_WATING_TIME_EXT_RESPONSE_NUM_BYTE             4
#define M24SR_PASSWORD_NUM_BYTE                             0x10



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
#define M24SR_DEV_ID_NEEDED                                 0x08        /* Device ID byte present or not */
/*  Command structure   ------------------------------------------------------------------------*/

#define M24SR_CMD_CATEGORY_0                            (M24SR_PCB_NEEDED  | \
                                                        M24SR_CLA_NEEDED  | \
                                                        M24SR_INS_NEEDED  | \
                                                        M24SR_P1_NEEDED   | \
                                                        M24SR_P2_NEEDED   | \
                                                        M24SR_LC_NEEDED   | \
                                                        M24SR_DATA_NEEDED | \
                                                        M24SR_LE_NEEDED   | \
                                                        M24SR_CRC_NEEDED)

#define M24SR_CMD_CATEGORY_1                            (M24SR_PCB_NEEDED  | \
                                                        M24SR_CLA_NEEDED  | \
                                                        M24SR_INS_NEEDED  | \
                                                        M24SR_P1_NEEDED   | \
                                                        M24SR_P2_NEEDED   | \
                                                        M24SR_LC_NEEDED   | \
                                                        M24SR_DATA_NEEDED | \
                                                        M24SR_CRC_NEEDED)

#define M24SR_CMD_CATEGORY_2                            (M24SR_PCB_NEEDED  | \
                                                        M24SR_CLA_NEEDED  | \
                                                        M24SR_INS_NEEDED  | \
                                                        M24SR_P1_NEEDED   | \
                                                        M24SR_P2_NEEDED   | \
                                                        M24SR_LE_NEEDED   | \
                                                        M24SR_CRC_NEEDED)

#define M24SR_CMD_CATEGORY_3                            (M24SR_PCB_NEEDED  | \
                                                        M24SR_CLA_NEEDED  | \
                                                        M24SR_INS_NEEDED  | \
                                                        M24SR_P1_NEEDED   | \
                                                        M24SR_P2_NEEDED   | \
                                                        M24SR_LC_NEEDED   | \
                                                        M24SR_CRC_NEEDED)

#define M24SR_CMD_CATEGORY_4                            (M24SR_PCB_NEEDED  | \
                                                        M24SR_CLA_NEEDED  | \
                                                        M24SR_INS_NEEDED  | \
                                                        M24SR_P1_NEEDED   | \
                                                        M24SR_P2_NEEDED   | \
                                                        M24SR_CRC_NEEDED)



/*  Offset  ----------------------------------------------------------------------------------*/
#define M24SR_OFFSET_PCB                                    0
#define M24SR_OFFSET_CLASS                                  1
#define M24SR_OFFSET_INS                                    2
#define M24SR_OFFSET_P1                                     3
#define M24SR_OFFSET_P2                                     4


/*  mask    ------------------------------------------------------------------------------------*/
#define M24SR_MASK_BLOCK                                    0xC0
#define M24SR_MASK_IBLOCK                                   0x00
#define M24SR_MASK_RBLOCK                                   0x80
#define M24SR_MASK_SBLOCK                                   0xC0

/**
  * @brief  Synchronization Mechanism structure 
  */
typedef enum{
    M24SR_WAITING_TIME_UNKNOWN = 0,
    M24SR_WAITING_TIME_POLLING,
    M24SR_WAITING_TIME_GPO,
    M24SR_INTERRUPT_GPO,
} m24sr_waiting_time_mode_t;    



#define M24SR_I2C_TIMEOUT       200     /* I2C Time out (ms), this is the maximum time needed by M24SR to complete any command */
#define M24SR_I2C_POLLING       1       /* In case M24SR will reply ACK failed allow to perform retry before returning error (HAL option not used) */
#define M24SR_ADDR              0xAC   /*!< M24SR address */

#ifdef __cplusplus
}
#endif

#endif /* _M24SR_INTERNAL_H */