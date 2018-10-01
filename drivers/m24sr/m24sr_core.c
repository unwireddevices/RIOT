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
#include "m24sr_core.h"
#include "m24sr_internal.h"

#include "assert.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "xtimer.h"

#define ENABLE_DEBUG                (0)
#include "debug.h"


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




/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param dev [description]
 * @param buffer [description]
 * @param len [description]
 * @return [description]
 */
static int _write_i2c(const m24sr_t *dev, uint8_t *buffer, uint32_t len)
{
    int ret = -1;

    DEBUG("m24sr: -> ");
    PRINTBUFF(buffer, len);

    i2c_acquire(dev->params.i2c);
    ret = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, buffer, len, 0);
    i2c_release(dev->params.i2c);

    return ret;
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param dev [description]
 * @param buffer [description]
 * @param len [description]
 * @return [description]
 */
static int _read_i2c(const m24sr_t *dev, uint8_t *buffer, uint32_t len)
{
    int ret = -1;

    i2c_acquire(dev->params.i2c);
    ret = i2c_read_bytes(dev->params.i2c, dev->params.i2c_addr, buffer, len, 0);
    i2c_release(dev->params.i2c);

    DEBUG("m24sr: <- ");
    PRINTBUFF(buffer, len);
    return ret;
}


 /**
  * @brief This functions sends the command buffer
  * 
  * @param dev [description]
  * @param buffer pointer to the buffer to send to the M24SR
  * @param len Number of byte to send
  * @return Error (M24SR_STATUS_SUCCESS or M24SR_ERROR_I2CTIMEOUT)
  */ 

int m24sr_send_i2c_cmd(const m24sr_t *dev, uint8_t *buffer, uint8_t len) {
    int ret = (_write_i2c(dev, buffer, len) == 0) ? (M24SR_OK) : (M24SR_NOBUS);
    //DEBUG("Write I2C is %s\n",(ret == 0)?"M24SR_OK":"M24SR_NOBUS");
    return ret;
}

/**
 * @brief This functions reads a response of the M24SR device
 * 
 * @param[in] dev - 
 * @param[out] buffer - Pointer on the buffer to retrieve M24SR response
 * @param[in] len - Number of byte to read (shall be >= 5)
 * @return Status (M24SR_STATUS_SUCCESS : The function is succesful, M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.) 
 */
int m24sr_rcv_i2c_response(const m24sr_t *dev, uint8_t *buffer, uint8_t len) {
    int ret = (_read_i2c(dev, buffer, len) == 0) ? (M24SR_OK) : (M24SR_NOBUS);
    //DEBUG("Read I2C is %s\n",(ret == 0)?"M24SR_OK":"M24SR_NOBUS"); 
    return ret;
}

/**
  * @brief  This functions polls the I2C interface
  * @retval M24SR_STATUS_SUCCESS : the function is succesful
  * @retval M24SR_ERROR_I2CTIMEOUT : The I2C timeout occured.
  */
int m24sr_poll_i2c (const m24sr_t *dev) {

    int ret = M24SR_OK;
    uint8_t data[] = {0x00};
    uint32_t current_timestamp = 0;
    const uint32_t start_timestamp = (xtimer_now_usec() / US_PER_MS);

    /* Wait until M24SR is ready or timeout occurs */

    do {
        //ret = _write_i2c(dev, 0x00, 0);
        ret = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTOP);
        current_timestamp = (xtimer_now_usec() / US_PER_MS);
    } while (((current_timestamp - start_timestamp) < M24SR_I2C_TIMEOUT) && (ret != 0));
    
    if (((current_timestamp - start_timestamp) > M24SR_I2C_TIMEOUT) || (ret != 0)) {
        ret = M24SR_NODEV;
    }
    else {
        ret = M24SR_OK;
    }
    i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTART);
    // DEBUG("Polling end. Status is %d\n",ret);
    return ret;
}


int m24sr_release_i2c_token(const m24sr_t *dev) {

    int status = M24SR_OK;
    uint8_t data[] = {0x00};

    // DEBUG("Release Token I2C\n");

    status = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTOP);
    if (status != 0)
        return M24SR_NOBUS;
    // DEBUG("Delay\n");
    xtimer_usleep(40000);

    status = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, data, 0, I2C_NOSTART);
    // DEBUG("End release token (status is %d)\n", status);
    if (status == 0)
        return M24SR_OK;
    else
        return M24SR_NOBUS;
}


/**
  * @brief  This functions returns M24SR_STATUS_SUCCESS when a response is ready
  * @retval M24SR_STATUS_SUCCESS : a response of the M24LR is ready
  * @retval M24SR_ERROR_DEFAULT : the response of the M24LR is not ready
  */
int m24sr_is_answer_rdy(m24sr_t *dev) {
    uint32_t retry = 0xFFFFF;
    uint8_t stable = 0;

    switch (dev->synchro_mode) {
        case M24SR_WAITING_TIME_POLLING:
            if(m24sr_poll_i2c(dev) != M24SR_OK)
                return M24SR_NOBUS;
            return M24SR_OK;
        case M24SR_WAITING_TIME_GPO:
            do {
                if (gpio_read(dev->params.gpo_pin) == 0) {
                    stable ++;
                }
                retry --;
            }
            while (stable < 5 && retry > 0);
            if (!retry)
               return M24SR_NOBUS;
            return M24SR_OK;
        case M24SR_INTERRUPT_GPO:
            /* Check if the GPIO is not already low before calling this function */
            if (gpio_read(dev->params.gpo_pin) == 1) {
                while (dev->event_ready == 0);
            }
            dev->event_ready = 0;
            return M24SR_OK;
        default:
            return M24SR_ERROR;
    }
}
