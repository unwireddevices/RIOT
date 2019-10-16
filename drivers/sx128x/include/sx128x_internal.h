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

/**
 * @brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 * 
 * @param[in]  dev           Pointer to device descriptor
 */
void sx1280_hal_wait_on_busy(const sx128x_t *dev);

/**
 * @brief 
 * 
 * @param dev 
 * @param irq_handlers 
 */
void sx1280_hal_init(sx128x_t *dev, dio_irq_handler *irq_handlers);

/**
 * @brief 
 * 
 * @param dev 
 * @param irq_handlers 
 * @return int 
 */
int sx1280_hal_io_irq_init(sx128x_t *dev, dio_irq_handler *irq_handlers);


/**
 * @brief Resets the radio
 * 
 * @param[in]  dev           Pointer to device descriptor
 */
void sx1280_hal_reset(const sx128x_t *dev);

/**
 * @brief Clears the instruction ram memory block
 * 
 * @param[in]  dev           Pointer to device descriptor
 */
void sx1280_hal_clear_instruction_ram(const sx128x_t *dev);

/**
 * @brief Wakes up the radio
 * 
 * @param[in]  dev           Pointer to device descriptor
 */
void sx1280_hal_wakeup(const sx128x_t *dev);

/**
 * @brief Send a command that write data to the radio
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  opcode        Opcode of the command
 * @param[in]  buffer        Buffer to be send to the radio
 * @param[in]  size          Size of the buffer to send
 */
void sx1280_hal_write_command(const sx128x_t *dev, sx128x_radio_commands_t command, uint8_t *buffer, uint16_t size);

/**
 * @brief Send a command that read data from the radio
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  opcode        Opcode of the command
 * @param[out] buffer        Buffer holding data from the radio
 * @param[in]  size          Size of the buffer
 */
void sx1280_hal_read_command(const sx128x_t *dev, sx128x_radio_commands_t command, uint8_t *buffer, uint16_t size);

/**
 * @brief Writes multiple radio registers starting at address
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  address       The address of the first byte to write in the radio
 * @param[in]  buffer        The data to be written in radio's memory
 * @param[in]  size          The number of bytes to write in radio's memory
 */
void sx1280_hal_write_registers(const sx128x_t *dev, uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Writes the radio register at the specified address
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  address       Register address
 * @param[in]  value         New register value
 */
void sx1280_hal_write_register(const sx128x_t *dev, uint16_t address, uint8_t value);

/**
 * @brief Reads multiple radio registers starting at address
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  address       The address of the first byte to read from the radio
 * @param[out] buffer        The buffer that holds data read from radio
 * @param[in]  size          The number of bytes to read from radio's memory
 */
void sx1280_hal_read_registers(const sx128x_t *dev, uint16_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief Reads the radio register at the specified address
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  address       Register address
 *
 * @retval     value         The register value
 */
uint8_t sx1280_hal_read_register(const sx128x_t *dev, uint16_t address);

/**
 * @brief Write data to the buffer holding the payload in the radio
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  offset        The offset to start writing the payload
 * @param[in]  buffer        The data to be written (the payload)
 * @param[in]  size          The number of byte to be written
 */
void sx1280_hal_write_buffer(const sx128x_t *dev, uint8_t offset, uint8_t *buffer, uint8_t size);

/**
 * @brief Read data from the buffer holding the payload in the radio
 *
 * @param[in]  dev           Pointer to device descriptor
 * @param[in]  offset        The offset to start reading the payload
 * @param[out] buffer        A pointer to a buffer holding the data from the radio
 * @param[in]  size          The number of byte to be read
 */
void sx1280_hal_read_buffer(const sx128x_t *dev, uint8_t offset, uint8_t *buffer, uint8_t size);

/**
 * @brief Gets the current status of the radio DIOs
 *
 * @param[in]  dev           Pointer to device descriptor
 * 
 * @retval     dio_status    A byte where each bit represents a DIO state:
 *                           [Bit #3: DIO3, Bit #2: DIO2,
 *                            Bit #1: DIO1, Bit #0: BUSY]
 */
uint8_t sx1280_hal_get_dio_status(const sx128x_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __SX128X_INTERNAL_H__ */
