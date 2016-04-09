/*
 * Copyright (C) 2016 cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1276
 * @{
 * @file
 * @brief       Basic functionality of sx1276 driver
 *
 * @author      Cr0s
 * @}
 */
#include <stdbool.h>

#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"

#include "sx1276.h"

/*
 * Local types definition
 */

/**
 * Radio registers definition
 */
typedef struct {
	sx1276_radio_modems_t modem;
	uint8_t addr;
	uint8_t value;

} sx1276_radio_registers_t;

/*
 * Private functions prototypes
 */

/**
 * @brief Performs the Rx chain calibration for LF and HF bands
 * Must be called just after the reset so all registers are at their
 *         default values
 */
static void _rx_chain_calibration(sx1276_t *dev);

/**
 * @brief Resets the SX1276
 */
void sx1276_reset(sx1276_t *dev);

/**
 * @brief Sets the SX1276 in transmission mode for the given time
 * @param [IN] timeout Transmission timeout [us] [0: continuous, others timeout]
 */
void sx1276_set_tx(sx1276_t *dev, uint32_t timeout);

/**
 * @brief Writes the buffer contents to the SX1276 FIFO
 *
 * @param [IN] buffer Buffer containing data to be put on the FIFO.
 * @param [IN] size Number of bytes to be written to the FIFO
 */
void sx1276_write_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Reads the contents of the SX1276 FIFO
 *
 * @param [OUT] buffer Buffer where to copy the FIFO read data.
 * @param [IN] size Number of bytes to be read from the FIFO
 */
void sx1276_read_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Sets the SX1276 operating mode
 *
 * @param [IN] op_mode New operating mode
 */
void sx1276_set_op_mode(sx1276_t *dev, uint8_t op_mode);

/*
 * SX1276 DIO _irq callback functions prototype
 */

/**
 * @brief DIO 0 _irq callback
 */
void sx1276_on_dio_0_irq(void);

/**
 * @brief DIO 1 _irq callback
 */
void sx1276_on_dio_1_irq(void);

/**
 * @brief DIO 2 _irq callback
 */
void sx1276_on_dio_2_irq(void);

/**
 * @brief DIO 3 _irq callback
 */
void sx1276_on_dio_3_irq(void);

/**
 * @brief DIO 4 _irq callback
 */
void sx1276_on_dio_4_irq(void);

/**
 * @brief DIO 5 _irq callback
 */
void sx1276_on_dio_5_irq(void);

/**
 * @brief Tx & Rx timeout timer callback
 */
void sx1276_on_timeout_irq(void);

/*
 * Private global constants
 */

/**
 * Radio hardware registers initialization
 * RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const sx1276_radio_registers_t radio_regs_init[] = RADIO_INIT_REGISTERS_VALUE;

/**
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*
 * Private global variables
 */

/**
 * Hardware DIO _irq callback initialization
 */
sx1276_dio_irq_handler *dio_irq[] = { sx1276_on_dio_0_irq, sx1276_on_dio_1_irq,
		sx1276_on_dio_2_irq, sx1276_on_dio_3_irq, sx1276_on_dio_4_irq, NULL };

/**
 * Tx and Rx timers
 */
// TODO: use of RIOT timers
/*TimerEvent_t TxTimeoutTimer;
 TimerEvent_t RxTimeoutTimer;
 TimerEvent_t RxTimeoutSyncWord;*/

void sx1276_init(sx1276_t* dev, sx_1276_events_t *events) {
	uint8_t i;

	dev->events = events;

	// Initialize driver timeout timers
	// TODO: use of RIOT timers
	/*TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );
	 TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
	 TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );*/

	sx1276_reset(dev);

	_rx_chain_calibration(dev);

	sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);

	sx1276_irq_init(dev, dio_irq);

	for (i = 0; i < sizeof(radio_regs_init) / sizeof(sx1276_radio_registers_t);
			i++) {
		sx1276_set_modem(dev, radio_regs_init[i].modem);
		sx1276_reg_write(dev, radio_regs_init[i].addr,
				radio_regs_init[i].value);
	}

	sx1276_set_modem(dev, MODEM_FSK);

	dev->settings->state = RF_IDLE;
}

sx1276_radio_state_t sx1276_get_status(sx1276_t *dev) {
	return dev->settings->state;
}

void sx1276_set_channel(sx1276_t *dev, uint32_t freq) {
	dev->settings->channel = freq;

	freq = (uint32_t)((double) freq / (double) FREQ_STEP);

	// Write frequency settings into chip
	sx1276_reg_write(dev, REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
	sx1276_reg_write(dev, REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
	sx1276_reg_write(dev, REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

bool sx1276_is_channel_free(sx1276_t *dev, uint32_t freq, uint16_t rssi_thresh) {
	int16_t rssi = 0;

	sx1276_set_channel(dev, freq);
	sx1276_set_op_mode(dev, RF_OPMODE_RECEIVER);

	xtimer_usleep(1000); // wait 1 millisecond

	rssi = sx1276_read_rssi(dev);
	sx1276_set_sleep(dev);

	if (rssi > rssiThresh) {
		return false;
	}

	return true;
}

void sx1276_set_modem(sx1276_t *dev, sx1276_radio_modems_t modem) {
	if (dev->spi == NULL) {
		while (1)
			;
	}
	if (dev->settings.modem == modem) {
		return;
	}

	dev->settings.modem = modem;

	switch (dev->settings.modem) {
	default:
	case MODEM_FSK:
		sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);
		sx1276_reg_write(dev, REG_OPMODE,
				(sx1276_reg_read(dev, REG_OPMODE)
						& RFLR_OPMODE_LONGRANGEMODE_MASK)
						| RFLR_OPMODE_LONGRANGEMODE_OFF);

		sx1276_reg_write(dev, REG_DIOMAPPING1, 0x00);
		sx1276_reg_write(dev, REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
		break;
	case MODEM_LORA:
		sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);
		sx1276_reg_write(REG_OPMODE,
				(sx1276_reg_read(dev, REG_OPMODE)
						& RFLR_OPMODE_LONGRANGEMODE_MASK)
						| RFLR_OPMODE_LONGRANGEMODE_ON);

		sx1276_reg_write(REG_DIOMAPPING1, 0x00);
		sx1276_reg_write(REG_DIOMAPPING2, 0x00);
		break;
	}
}

uint32_t sx1275_random(sx1276_t *dev) {
	uint8_t i;
	uint32_t rnd = 0;

	/*
	 * Radio setup for random number generation
	 */
	sx1276_set_modem(dev, MODEM_LORA);	// Set LoRa modem ON

	// Disable LoRa modem interrupts
	sx1275_reg_write(dev, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
	RFLR_IRQFLAGS_RXDONE |
	RFLR_IRQFLAGS_PAYLOADCRCERROR |
	RFLR_IRQFLAGS_VALIDHEADER |
	RFLR_IRQFLAGS_TXDONE |
	RFLR_IRQFLAGS_CADDONE |
	RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
	RFLR_IRQFLAGS_CADDETECTED);

	// Set radio in continuous reception
	sx1276_set_op_mode(dev, RF_OPMODE_RECEIVER);

	for (i = 0; i < 32; i++) {
		xtimer_usleep(1000); // wait 1 millisecond

		// Not filtered RSSI value reading. Only takes the LSB value
		rnd |= ((uint32_t) sx1276_read(dev, REG_LR_RSSIWIDEBAND) & 0x01) << i;
	}

	sx1276_set_sleep(dev);

	return rnd;
}

/**
 * @brief Performs the Rx chain calibration for LF and HF bands
 * @note Must be called just after the reset so all registers are at their
 *         default values
 */
static void _rx_chain_calibration(void) {
	uint8_t reg_pa_config_init_val;
	uint32_t initial_freq;

	// Save context
	reg_pa_config_init_val = sx1276_read(dev, REG_PACONFIG);
	initial_freq = (double) (((uint32_t) sx1276_read(dev, REG_FRFMSB) << 16)
			| ((uint32_t) sx1276_read(dev, REG_FRFMID) << 8)
			| ((uint32_t) sx1276_read(dev, REG_FRFLSB))) * (double) FREQ_STEP;

	// Cut the PA just in case, RFO output, power = -1 dBm
	sx1276_reg_write(dev, REG_PACONFIG, 0x00);

	// Launch Rx chain calibration for LF band
	sx1276_reg_write(dev, REG_IMAGECAL,
			(sx1276_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK)
					| RF_IMAGECAL_IMAGECAL_START);

	while ((sx1276_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
			== RF_IMAGECAL_IMAGECAL_RUNNING) {
	}

	// Set a frequency in HF band
	sx1276_set_channel(dev, CHANNEL_HF);

	// Launch Rx chain calibration for HF band
	sx1276_reg_write(dev, REG_IMAGECAL,
			(sx1276_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK)
					| RF_IMAGECAL_IMAGECAL_START);
	while ((sx1276_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
			== RF_IMAGECAL_IMAGECAL_RUNNING) {
	}

	// Restore context
	sx1276_reg_write(dev, REG_PACONFIG, reg_pa_config_init_val);
	sx1275_set_channel(dev, initial_freq);
}
