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
 * @brief       implementation of internal functions for sx128x
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 * @}
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sx128x.h"
#include "sx128x_internal.h"
#include "sx128x_params.h"

#include "periph/gpio.h"

#include "lptimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "log.h"

/**
 * @brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * @warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE                             (0xFFF/2)

static uint8_t hal_tx_buffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t hal_rx_buffer[MAX_HAL_BUFFER_SIZE] = {0x00};

void sx1280_hal_wait_on_busy(const sx128x_t *dev)
{
    volatile uint16_t timeout = 0xFFFF;

    do {
        //do Nothing
    } while ((gpio_read(dev->params.busy_pin) != 0x0000) && (--timeout));
}

void sx1280_hal_init(sx128x_t *dev, dio_irq_handler *irq_handlers)
{
    /* Initialize RESET pin */
    gpio_init(dev->params.reset_pin, GPIO_OUT);

    /* Initialize SPI NSS pin */
    gpio_init(dev->params.nss_pin, GPIO_OUT);

    /* Initialize BUSY pin */
    gpio_init(dev->params.busy_pin, GPIO_IN);

    /* Initialize SPI */
    spi_init(dev->params.spi_dev);

    /* Initialize Low Power Timer */
    lptimer_init();

    sx1280_hal_reset(dev);

    sx1280_hal_io_irq_init(dev, irq_handlers);
}

int sx1280_hal_io_irq_init(sx128x_t *dev, dio_irq_handler *irq_handlers)
{
    int status = 0;

    if (dev->params.dio1_pin != GPIO_UNDEF) {
        status = gpio_init_int(dev->params.dio1_pin, GPIO_IN, GPIO_RISING, irq_handlers[0], dev);
        if (status < 0) {
            LOG_ERROR("[SX128X] Error: failed to initialize DIO1 pin\n");
        }
    }

    if (dev->params.dio2_pin != GPIO_UNDEF) {
        status = gpio_init_int(dev->params.dio2_pin, GPIO_IN, GPIO_RISING, irq_handlers[0], dev);
        if (status < 0) {
            LOG_ERROR("[SX128X] Error: failed to initialize DIO2 pin\n");
        }
    }

    if (dev->params.dio3_pin != GPIO_UNDEF) {
        status = gpio_init_int(dev->params.dio3_pin, GPIO_IN, GPIO_RISING, irq_handlers[0], dev);
        if (status < 0) {
            LOG_ERROR("[SX128X] Error: failed to initialize DIO3 pin\n");
        }
    }

    if (dev->params.dio1_pin != GPIO_UNDEF &&
        dev->params.dio2_pin != GPIO_UNDEF &&
        dev->params.dio3_pin != GPIO_UNDEF) 
    {
        LOG_ERROR("DIO pins don't defined");
    }

    return status;
}

void sx1280_hal_reset(const sx128x_t *dev)
{
    lptimer_sleep(20);
    gpio_clear(dev->params.reset_pin);
    lptimer_sleep(50);
    gpio_set(dev->params.reset_pin);
    lptimer_sleep(20);
}

void sx1280_hal_clear_instruction_ram(const sx128x_t *dev)
{
    /* Clearing the instruction RAM is writing 0x00s on every bytes of the
     * instruction RAM */
    uint16_t hal_size = 3 + SX128X_IRAM_SIZE;

    hal_tx_buffer[0] = SX128X_RADIO_WRITE_REGISTER;
    hal_tx_buffer[1] = (SX128X_IRAM_START_ADDRESS >> 8) & 0x00FF;
    hal_tx_buffer[2] = SX128X_IRAM_START_ADDRESS & 0x00FF;

    memset(&hal_tx_buffer[3], 0x00, SX128X_IRAM_SIZE);

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, NULL, hal_size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

void sx1280_hal_wakeup(const sx128x_t *dev)
{
    __disable_irq( );

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    uint16_t hal_size = 2;

    hal_tx_buffer[0] = SX128X_RADIO_GET_STATUS;
    hal_tx_buffer[1] = 0x00;

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, NULL, hal_size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    // Wait for chip to be ready.
    sx1280_hal_wait_on_busy(dev);

    __enable_irq( );
}

void sx1280_hal_write_command(const sx128x_t *dev, sx128x_radio_commands_t command, uint8_t *buffer, uint16_t size)
{

    uint16_t hal_size  = size + 1;

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    hal_tx_buffer[0] = command;
    memcpy(hal_tx_buffer + 1, buffer, size);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, NULL, hal_size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    if(command != SX128X_RADIO_SET_SLEEP) {
        sx1280_hal_wait_on_busy(dev);
    }
}

void sx1280_hal_read_command(const sx128x_t *dev, sx128x_radio_commands_t command, uint8_t *buffer, uint16_t size)
{
    uint16_t hal_size = 2 + size;

    hal_tx_buffer[0] = command;
    hal_tx_buffer[1] = 0x00;
    for(uint16_t index = 0; index < size; index++) {
        hal_tx_buffer[2 + index] = 0x00;
    }

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, hal_rx_buffer, hal_size);

    memcpy(buffer, hal_rx_buffer + 2, size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

void sx1280_hal_write_registers(const sx128x_t *dev, uint16_t address, uint8_t *buffer, uint16_t size)
{
    uint16_t hal_size = size + 3;

    hal_tx_buffer[0] = SX128X_RADIO_WRITE_REGISTER;
    hal_tx_buffer[1] = (address & 0xFF00) >> 8;
    hal_tx_buffer[2] = address & 0x00FF;
    memcpy(hal_tx_buffer + 3, buffer, size);

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, NULL, hal_size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

void sx1280_hal_write_register(const sx128x_t *dev, uint16_t address, uint8_t value)
{
    sx1280_hal_write_registers(dev, address, &value, 1);
}

void sx1280_hal_read_registers(const sx128x_t *dev, uint16_t address, uint8_t *buffer, uint16_t size)
{
    uint16_t hal_size = 4 + size;

    hal_tx_buffer[0] = SX128X_RADIO_READ_REGISTER;
    hal_tx_buffer[1] = (address & 0xFF00 ) >> 8;
    hal_tx_buffer[2] = address & 0x00FF;
    hal_tx_buffer[3] = 0x00;

    for(uint16_t index = 0; index < size; index++) {
        hal_tx_buffer[4 + index] = 0x00;
    }

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, hal_rx_buffer, hal_size);

    memcpy(buffer, hal_rx_buffer + 4, size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

uint8_t sx1280_hal_read_register(const sx128x_t *dev, uint16_t address)
{
    uint8_t data;

    sx1280_hal_read_registers(dev, address, &data, 1);

    return data;
}

void sx1280_hal_write_buffer(const sx128x_t *dev, uint8_t offset, uint8_t *buffer, uint8_t size)
{
    uint16_t hal_size = size + 2;

    hal_tx_buffer[0] = SX128X_RADIO_WRITE_BUFFER;
    hal_tx_buffer[1] = offset;

    memcpy(hal_tx_buffer + 2, buffer, size);

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, NULL, hal_size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

void sx1280_hal_read_buffer(const sx128x_t *dev, uint8_t offset, uint8_t *buffer, uint8_t size)
{
    uint16_t hal_size = size + 3;

    hal_tx_buffer[0] = SX128X_RADIO_READ_BUFFER;
    hal_tx_buffer[1] = offset;
    hal_tx_buffer[2] = 0x00;

    for(uint16_t index = 0; index < size; index++) {
        hal_tx_buffer[3 + index] = 0x00;
    }

    sx1280_hal_wait_on_busy(dev);

    spi_acquire(dev->params.spi_dev, SPI_CS_UNDEF, dev->params.spi_mode, dev->params.spi_speed);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi_dev, SPI_CS_UNDEF, false, hal_tx_buffer, hal_rx_buffer, hal_size);

    memcpy(buffer, hal_rx_buffer + 3, size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi_dev);

    sx1280_hal_wait_on_busy(dev);
}

uint8_t sx1280_hal_get_dio_status(const sx128x_t *dev)
{
    uint8_t status = gpio_read(dev->params.busy_pin);

    if (dev->params.dio1_pin != GPIO_UNDEF) {
        status |= (gpio_read(dev->params.dio1_pin) << 1);
    }

    if (dev->params.dio2_pin != GPIO_UNDEF) {
        status |= (gpio_read(dev->params.dio2_pin) << 2);
    }

    if (dev->params.dio3_pin != GPIO_UNDEF) {
        status |= (gpio_read(dev->params.dio3_pin) << 3);
    }

    if (dev->params.dio1_pin != GPIO_UNDEF &&
        dev->params.dio2_pin != GPIO_UNDEF &&
        dev->params.dio3_pin != GPIO_UNDEF) 
    {
        puts("DIO pins don't defined");
    }
    return status;
}
