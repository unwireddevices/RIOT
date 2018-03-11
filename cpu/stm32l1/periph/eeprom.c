/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

#include <stdint.h>
#include <stddef.h>

#include "nvram.h"
#include "assert.h"
#include "cpu.h"

#include "eeprom.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @ingroup     nvram
 * @{
 *
 * @file        eeprom.c
 * @brief       STM32L1 EEPROM driver
 * @author      Eugeny P. <ep@unwds.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLASH_BUSY = 1,
    FLASH_ERROR_WRP,
    FLASH_ERROR_PROGRAM,
    FLASH_COMPLETE,
    FLASH_TIMEOUT
} l1_flash_status_t;

/**
 * @brief  Unlocks the data memory and FLASH_PECR register access.
 * @param  None
 * @return None
 */
static void eeprom_unlock(void) {
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != RESET) {
        /* Unlocking the Data memory and FLASH_PECR register access*/
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
    }
    
    while ((FLASH->PECR & FLASH_PECR_PELOCK) != RESET) {};
}

/**
 * @brief  Locks the Data memory and FLASH_PECR register access.
 * @param  None
 * @return None
 */
static void eeprom_lock(void)
{
    /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
    FLASH->PECR |= FLASH_PECR_PELOCK;
}

/**
 * @brief Returns the current FLASH device status
 */
static l1_flash_status_t get_status(void) {
    l1_flash_status_t status = FLASH_COMPLETE;

    if ((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) {
        status = FLASH_BUSY;
    }
    else {
        if ((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR) != (uint32_t)0x00) {
            status = FLASH_ERROR_WRP;
        }
        else {
            if ((FLASH->SR & (uint32_t)0x1E00) != (uint32_t)0x00) {
                status = FLASH_ERROR_PROGRAM;
            }
            else {
                status = FLASH_COMPLETE;
            }
        }
    }

    /* Return the FLASH Status */
    return status;
}

/**
 * @brief Busy-waits within a timeout if the FLASH peripheral is performing an operation
 *
 * @param[in] timeout timeout value in cycles
 * @return current status of FLASH device
 */
static l1_flash_status_t flash_wait_for_last_operation(uint32_t timeout) {
    l1_flash_status_t status = FLASH_COMPLETE;

    /* Check for the FLASH Status */
    status = get_status();

    /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
    while ((status == FLASH_BUSY) && (timeout != 0x00)) {
        status = get_status();
        timeout--;
    }

    if (timeout == 0x00) {
        status = FLASH_TIMEOUT;
    }

    /* Return the operation status */
    return status;
}

/**
  * @brief  Erase a word in data memory.
  * @param  Address: specifies the address to be erased.
  * @note   For STM32L1XX_MD, A data memory word is erased in the data memory only 
  *         if the address to load is the start address of a word (multiple of a word).
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to disable the data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
static l1_flash_status_t flash_data_eeprom_eraseword(uint32_t address) {
    l1_flash_status_t status = FLASH_COMPLETE;

    /* Check the parameters */
    assert(IS_FLASH_DATA_ADDRESS(address));

    /* Wait for last operation to be completed */
    status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);

    if (status == FLASH_ERROR_WRP) {
        FLASH->SR |= (uint32_t)FLASH_FLAG_WRPERR;
        status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);
    }

    if(status == FLASH_COMPLETE)
    {
        /* Write "00000000h" to valid address in the data memory" */
        *(__IO uint32_t *) address = 0x00000000;
    }

    /* Return the erase status */
    return status;
}

/**
  * @brief  Programs a word at a specified address in data memory.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to disable the data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @note   This function assumes that the is data word is already erased.
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
static l1_flash_status_t flash_data_eeprom_fastprogramword(uint32_t address, uint32_t data) {
    l1_flash_status_t status = FLASH_COMPLETE;

    /* Check the parameters */
    assert(IS_FLASH_DATA_ADDRESS(address));

    /* Wait for last operation to be completed */
    status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);

    if (status == FLASH_ERROR_WRP) {
        FLASH->SR |= (uint32_t)FLASH_FLAG_WRPERR;
        status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);
    }

    if(status == FLASH_COMPLETE) {
        /* Clear the FTDW bit */
        FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FTDW));

        /* If the previous operation is completed, proceed to program the new data */    
        *(__IO uint32_t *)address = data;

        /* Wait for last operation to be completed */
        status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);       
    }
    /* Return the Write Status */
    return status;
}

/**
 * @brief  Write a Byte at a specified address in data EEPROM.
 * @param  address: specifies the address to be written.
 * @param  data: specifies the data to be written.
 * @retval FLASH Status: The returned value can be:
 *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
static l1_flash_status_t program_byte(uint32_t address, uint8_t data) {
    l1_flash_status_t status = FLASH_COMPLETE;

    uint32_t tmp = 0, tmpaddr = 0;

    /* Check the parameters */
    assert(IS_FLASH_DATA_ADDRESS(address));

    /* Wait for last operation to be completed */
    status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);
    
    if (status == FLASH_ERROR_WRP) {
        FLASH->SR |= (uint32_t)FLASH_FLAG_WRPERR;
        status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);
    }

    if (status == FLASH_COMPLETE) {
        if ((get_cpu_category() < 3) && (data == (uint8_t) 0x00)) {
            tmpaddr = address & 0xFFFFFFFC;
            tmp = *(__IO uint32_t *) tmpaddr;
            tmpaddr = 0xFF << ((uint32_t) (0x8 * (address & 0x3)));
            tmp &= ~tmpaddr;
            
            status = flash_data_eeprom_eraseword(address & 0xFFFFFFFC);
            DEBUG("[EEPROM] Erase word: %d\n", status);
            status = flash_data_eeprom_fastprogramword((address & 0xFFFFFFFC), tmp);
            DEBUG("[EEPROM] Fast program word: %d\n", status);
        } else {
            DEBUG("[EEPROM] Cat. 3+ CPU: writing %d to %08x\n", data, (unsigned int)address);
            *(__IO uint8_t *)address = data;

            /* Wait for last operation to be completed */
            status = flash_wait_for_last_operation(FLASH_ER_PRG_TIMEOUT);
        }
    } else {
        DEBUG("[EEPROM] Error: timeout status %d\n", status);
    }

    /* Return the Write Status */
    return status;
}

/**
 * @brief Copy data from system memory to NVRAM.
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 * @param[in]  src   Pointer to the first byte in the system memory address space
 * @param[in]  dst   Starting address in the NVRAM device address space
 * @param[in]  len   Number of bytes to copy
 *
 * @return           Number of bytes written on success
 * @return           <0 on errors
 */
static int nvram_write(nvram_t *dev, const uint8_t *src, uint32_t dst, size_t len);

/**
 * @brief Copy data from NVRAM to system memory.
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 * @param[out] dst   Pointer to the first byte in the system memory address space
 * @param[in]  src   Starting address in the NVRAM device address space
 * @param[in]  len   Number of bytes to copy
 *
 * @return           Number of bytes read on success
 * @return           <0 on errors
 */
static int nvram_read(nvram_t *dev, uint8_t *dst, uint32_t src, size_t len);

/**
 * @brief Clears all contents of NVRAM.
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 *
 * @return           Number of bytes in NVRAM on success
 * @return           <0 on errors
 */
static int nvram_clear(nvram_t *dev);

/**
 * @brief Clears part of NVRAM content
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 * @param[in]  start Start address of the fragment to clear
 * @param[in]  size  Size of the fragment to clear
 *
 * @return           Number of cleared bytes in NVRAM on success
 * @return           <0 on errors
 */
static int nvram_clear_bytes(nvram_t *dev, uint32_t start, size_t size);

int nvram_eeprom_init(nvram_t *dev) {
    dev->write = nvram_write;
    dev->read = nvram_read;
    dev->clear = nvram_clear;
    dev->clearpart = nvram_clear_bytes;

    return 0;
}

static int nvram_read(nvram_t *dev, uint8_t *dst, uint32_t src, size_t len) {
    (void) dev;
    uint32_t eeprom_addr = EEPROM_BASE + src;
    uint32_t i = 0;

    DEBUG("[EEPROM] RD @ %08x: ", (unsigned int)eeprom_addr);
    for (i = 0; i < len; i++) {
        /* Read byte from EEPROM memory */
        dst[i] = *((uint8_t *) (eeprom_addr + i));
        DEBUG("%02x ", dst[i]);
    }
    DEBUG("\n");

    return i;
}

static int nvram_write(nvram_t *dev, const uint8_t *src, uint32_t dst, size_t len) {
    (void) dev;
    eeprom_unlock();

    uint32_t eeprom_addr = EEPROM_BASE + dst;
    uint32_t i = 0;
    
    DEBUG("[EEPROM] WR @ %08x: ", (unsigned int)eeprom_addr);
    for (i = 0; i < len; i++) {
        /* Program byte */
        DEBUG("%02x ", src[i]);
        program_byte(eeprom_addr + i, src[i]);
    }
    DEBUG("\n");

    eeprom_lock();

    return i;
}

static int nvram_clear(nvram_t *dev) {
    (void) dev;
    eeprom_unlock();

    uint32_t i = 0;
    size_t len = get_cpu_eeprom_size();
    for (i = 0; i < len; i++) {
        /* Program byte */
        program_byte(EEPROM_BASE + i, 0xFF);
    }

    eeprom_lock();

    return i;
}

static int nvram_clear_bytes(nvram_t *dev, uint32_t start, size_t size) {
    (void) dev;
    eeprom_unlock();

    uint32_t i = 0;
    for (i = 0; i < size; i++) {
        /* Program byte */
        program_byte(EEPROM_BASE + start + i, 0xFF);
    }

    eeprom_lock();

    return i;
}

#ifdef __cplusplus
}
#endif
