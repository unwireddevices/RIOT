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

#include "net/lora.h"

#include "sx128x.h"
#include "sx128x_registers.h"
#include "sx128x_internal.h"
#include "sx128x_params.h"

#include "periph/gpio.h"

#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


#define SX127X_SPI_SPEED    (SPI_CLK_1MHZ)
#define SX127X_SPI_MODE     (SPI_MODE_0)

/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_BUFFER_SIZE   0xFFF

#define IRQ_HIGH_PRIORITY  0

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1280Init,
    SX1280HalReset,
    SX1280GetStatus,
    SX1280HalWriteCommand,
    SX1280HalReadCommand,
    SX1280HalWriteRegisters,
    SX1280HalWriteRegister,
    SX1280HalReadRegisters,
    SX1280HalReadRegister,
    SX1280HalWriteBuffer,
    SX1280HalReadBuffer,
    SX1280HalGetDioStatus,
    SX1280GetFirmwareVersion,
    SX1280SetRegulatorMode,
    SX1280SetStandby,
    SX1280SetPacketType,
    SX1280SetModulationParams,
    SX1280SetPacketParams,
    SX1280SetRfFrequency,
    SX1280SetBufferBaseAddresses,
    SX1280SetTxParams,
    SX1280SetDioIrqParams,
    SX1280SetSyncWord,
    SX1280SetRx,
    SX1280GetPayload,
    SX1280SendPayload,
    SX1280SetRangingRole,
    SX1280SetPollingMode,
    SX1280SetInterruptMode,
    SX1280SetRegistersDefault,
    SX1280GetOpMode,
    SX1280SetSleep,
    SX1280SetFs,
    SX1280SetTx,
    SX1280SetRxDutyCycle,
    SX1280SetCad,
    SX1280SetTxContinuousWave,
    SX1280SetTxContinuousPreamble,
    SX1280GetPacketType,
    SX1280SetCadParams,
    SX1280GetRxBufferStatus,
    SX1280GetPacketStatus,
    SX1280GetRssiInst,
    SX1280GetIrqStatus,
    SX1280ClearIrqStatus,
    SX1280Calibrate,
    SX1280SetSaveContext,
    SX1280SetAutoTx,
    SX1280StopAutoTx,
    SX1280SetAutoFS,
    SX1280SetLongPreamble,
    SX1280SetPayload,
    SX1280SetSyncWordErrorTolerance,
    SX1280SetCrcSeed,
    SX1280SetBleAccessAddress,
    SX1280SetBleAdvertizerAccessAddress,
    SX1280SetCrcPolynomial,
    SX1280SetWhiteningSeed,
    SX1280EnableManualGain,
    SX1280DisableManualGain,
    SX1280SetManualGainValue,
    SX1280SetLNAGainSetting,
    SX1280SetRangingIdLength,
    SX1280SetDeviceRangingAddress,
    SX1280SetRangingRequestAddress,
    SX1280GetRangingResult,
    SX1280SetRangingCalibration,
    SX1280GetRangingPowerDeltaThresholdIndicator,
    SX1280RangingClearFilterResult,
    SX1280RangingSetFilterNumSamples,
    SX1280GetFrequencyError,
};

static uint8_t tx_buffer[MAX_BUFFER_SIZE] = {0x00};
static uint8_t rx_buffer[MAX_BUFFER_SIZE] = {0x00};

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void sx1280_wait_on_busy(sx1280_t *dev)
{
    while(gpio_read(dev->busy) == 1);
}

void sx1280_init( DioIrqHandler **irqHandlers)
{
    sx1280_reset();
    sx1280_io_irq_init(irqHandlers);
}

void sx1280_io_irq_init(DioIrqHandler **irqHandlers)
{
#if(RADIO_DIO1_ENABLE)
    GpioSetIrq(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0]);
#endif
#if(RADIO_DIO2_ENABLE)
	GpioSetIrq(RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0]);
#endif
#if(RADIO_DIO3_ENABLE)
	GpioSetIrq(RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0]);
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO" 
#endif
}

void sx1280_reset(void)
{
    HAL_Delay(20);
    GpioWrite( RADIO_nRESET_PORT, RADIO_nRESET_PIN, 0 );
    HAL_Delay(50);
    GpioWrite( RADIO_nRESET_PORT, RADIO_nRESET_PIN, 1 );
    HAL_Delay(20);
}

void SX1280HalClearInstructionRam( void )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for( uint16_t index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    SpiIn( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX1280HalWaitOnBusy( );
}

void SX1280HalWakeup( void )
{
    __disable_irq( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    uint16_t halSize = 2;
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    SpiIn( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );

    __enable_irq( );
}

void sx1280_write_command( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{

    uint16_t _size  = size + 1;
    sx1280_wait_on_busy();

    spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX128X_SPI_MODE, SX128X_SPI_SPEED);

    gpio_clear(dev->params.nss_pin);

    tx_buffer[0] = command;
    memcpy((tx_buffer + 1), (uint8_t *)buffer, size * sizeof(uint8_t));

    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, false, tx_buffer, NULL, _size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi);

    if( command != RADIO_SET_SLEEP ) {
        sx1280_wait_on_busy();
    }
}

void sx1280_read_command(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
    uint16_t _size = 2 + size;
    tx_buffer[0] = command;
    tx_buffer[1] = 0x00;
    for(uint16_t index = 0; index < size; index++) {
        tx_buffer[2 + index] = 0x00;
    }

    sx1280_wait_on_busy();

    spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX128X_SPI_MODE, SX128X_SPI_SPEED);

    gpio_clear(dev->params.nss_pin);

    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, false, tx_buffer, rx_buffer, _size);

    memcpy(buffer, rx_buffer + 2, size);

    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi);

    sx1280_wait_on_busy();
}

void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, size );

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    SpiIn( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX1280HalWaitOnBusy( );
}

void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}

void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 4, size );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX1280HalWaitOnBusy( );
}

uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( address, &data, 1 );

    return data;
}

void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = offset;
    memcpy( halTxBuffer + 2, buffer, size );

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    SpiIn( halTxBuffer, halSize );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX1280HalWaitOnBusy( );
}

void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 3, size );

    GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX1280HalWaitOnBusy( );
}

uint8_t SX1280HalGetDioStatus( void )
{
	uint8_t Status = GpioRead( RADIO_BUSY_PORT, RADIO_BUSY_PIN );
	
#if( RADIO_DIO1_ENABLE )
	Status |= (GpioRead( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin ) << 1);
#endif
#if( RADIO_DIO2_ENABLE )
	Status |= (GpioRead( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin ) << 2);
#endif
#if( RADIO_DIO3_ENABLE )
	Status |= (GpioRead( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin ) << 3);
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO" 
#endif
	
	return Status;
}
