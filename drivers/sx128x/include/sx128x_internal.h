/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       Semtech SX128X internal functions
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef __SX128X_INTERNAL_H__
#define __SX128X_INTERNAL_H__

#include <inttypes.h>

#include "sx128x.h"
#include "sx128x_registers.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * * \brief Define which DIOs are connected 
*/
#define RADIO_DIO1_ENABLE	1
#define RADIO_DIO2_ENABLE	0
#define RADIO_DIO3_ENABLE	0

/**
 * @brief 
 * 
 */
void sx1280_wait_on_busy(void);

/**
 * @brief 
 * 
 * @param irqHandlers 
 */
void sx1280_init(DioIrqHandler **irqHandlers);

/**
 * @brief 
 * 
 */
void sx1280_io_init(void);

/**
 * @brief Soft resets the radio
 * 
 */
void sx1280_reset(void);

/**
 * @brief Clears the instruction ram memory block
 * 
 */
void sx1280_clear_instruction_ram(void);

/**
 * @brief Wakes up the radio
 */
void sx1280_wakeup(void);

/**
 * @brief Send a command that write data to the radio
 *
 * @param [in]  opcode        Opcode of the command
 * @param [in]  buffer        Buffer to be send to the radio
 * @param [in]  size          Size of the buffer to send
 */
void sx1280_write_command(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

/**
 * @brief Send a command that read data from the radio
 *
 * @param [in]  opcode        Opcode of the command
 * @param [out] buffer        Buffer holding data from the radio
 * @param [in]  size          Size of the buffer
 */
void sx1280_read_command(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

/**
 * @brief Write data to the radio memory
 *
 * @param [in]  address       The address of the first byte to write in the radio
 * @param [in]  buffer        The data to be written in radio's memory
 * @param [in]  size          The number of bytes to write in radio's memory
 */
void sx1280_write_registers(uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Write a single byte of data to the radio memory
 *
 * @param [in]  address       The address of the first byte to write in the radio
 * @param [in]  value         The data to be written in radio's memory
 */
void sx1280_write_register(uint16_t address, uint8_t value);

/**
 * @brief Read data from the radio memory
 *
 * @param [in]  address       The address of the first byte to read from the radio
 * @param [out] buffer        The buffer that holds data read from radio
 * @param [in]  size          The number of bytes to read from radio's memory
 */
void sx1280_read_registers(uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Read a single byte of data from the radio memory
 *
 * @param [in]  address       The address of the first byte to write in the
 *                            radio
 *
 * @retval      value         The value of the byte at the given address in
 *                            radio's memory
 */
uint8_t sx1280_read_register(uint16_t address);

/**
 * @brief Write data to the buffer holding the payload in the radio
 *
 * @param [in]  offset        The offset to start writing the payload
 * @param [in]  buffer        The data to be written (the payload)
 * @param [in]  size          The number of byte to be written
 */
void sx1280_write_buffer(uint8_t offset, uint8_t *buffer, uint8_t size);

/**
 * @brief Read data from the buffer holding the payload in the radio
 *
 * @param [in]  offset        The offset to start reading the payload
 * @param [out] buffer        A pointer to a buffer holding the data from the radio
 * @param [in]  size          The number of byte to be read
 */
void sx1280_read_buffer(uint8_t offset, uint8_t *buffer, uint8_t size);

/**
 * @brief Returns the status of DIOs pins
 *
 * @retval      dioStatus     A byte where each bit represents a DIO state:
 *                            [ DIOx | BUSY ]
 */
uint8_t sx1280_get_dio_status(void);

/**
 * @brief 
 * 
 * @param irqHandlers 
 */
void sx1280_io_irq_init( DioIrqHandler **irqHandlers );

#ifdef __cplusplus
}
#endif

#endif /* __SX128X_INTERNAL_H__ */
