/*
 * Copyright (C) 2016 Cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sx1276 SX1276
 * @ingroup     drivers_netdev
 * @brief       Semtech SX1276
 * @{
 * @file
 * @brief       Public interface for SX1276 driver
 * @author      Cr0s
 */

#include "periph/gpio.h"
#include "periph/spi.h"

#ifndef SX1276_H
#define SX1276_H

#define RADIO_WAKEUP_TIME                           1000 /**< [us] */
#define CHANNEL_HF									868000000 /**< [Hz] */

#define RX_BUFFER_SIZE                              256

#define RF_MID_BAND_THRESH                          525000000
#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}                                                 \


#ifdef __cplusplus
extern "C" {
#endif

/**
 * Radio driver supported modems
 */
typedef enum {
	MODEM_FSK = 0, MODEM_LORA,
} sx1276_radio_modems_t;

typedef struct {
	uint8_t power;
	uint32_t bandwidth;
	uint32_t datarate;
	bool low_datarate_optimize;
	uint8_t coderate;
	uint16_t preamble_len;
	bool fix_len;
	uint8_t payload_len;
	bool crc_on;
	bool freq_hop_on;
	uint8_t hop_period;
	bool iq_inverted;
	bool rx_continuous;
	uint32_t tx_timeout;

} sx1276_lora_settings_t;

/**
 * LoRa packet handler state.
 */
typedef struct {
	uint8_t snr_value;
	uint16_t rssi_value;
	uint8_t size;

} sx1276_lora_packet_handler_t;

/**
 * Radio driver internal state machine states definition.
 */
typedef enum {
	RF_IDLE = 0, RF_RX_RUNNING, RF_TX_RUNNING, RF_CAD,

} sx1276_radio_state_t;

/**
 * Radio settings.
 */
typedef struct {
	sx1276_radio_state_t state;
	uint32_t channel;
	sx1276_lora_settings_t lora;
	sx1276_lora_packet_handler_t lora_packet_handler;
	sx1276_radio_modems_t modem;

} sx1276_settings_t;

typedef struct {
	/**
	 * @brief  Tx Done callback prototype.
	 */
	void (*tx_done)(void);

	/**
	 * @brief  Tx Timeout callback prototype.
	 */
	void (*tx_timeout)(void);

	/**
	 * @brief Rx Done callback prototype.
	 *
	 * @param [IN] payload Received buffer pointer
	 * @param [IN] size    Received buffer size
	 * @param [IN] rssi    RSSI value computed while receiving the frame [dBm]
	 * @param [IN] snr     Raw SNR value given by the radio hardware [dB]
	 */
	void (*rx_done)(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

	/**
	 * @brief  Rx Timeout callback prototype.
	 */
	void (*rx_timeout)(void);

	/**
	 * @brief Rx Error callback prototype.
	 */
	void (*rx_error)(void);

	/**
	 * @brief  FHSS Change Channel callback prototype.
	 *
	 * @param [IN] current_channel   Index number of the current channel
	 */
	void (*fhss_change_channel)(uint8_t currnet_channel);

	/**
	 * @brief CAD Done callback prototype.
	 *
	 * @param [IN] activity_detected    Channel Activity detected during the CAD
	 */
	void (*cad_done)(bool activity_detected);

} sx1276_events_t;

/**
 * SX1276 hardware and global parameters.
 */
typedef struct sx1276_s {
	spi_t spi; /**< SPI */
	gpio_t nss_pin; /**< SPI NSS pin */

	gpio_t reset_pin; /**< Reset pin */
	gpio_t dio0_pin;
	gpio_t dio1_pin;
	gpio_t dio2_pin;
	gpio_t dio3_pin;
	gpio_t dio4_pin;
	gpio_t dio5_pin;

	uint8_t rxtx;
	sx1276_settings_t settings;

	sx1276_events_t events; /**< Radio events callbacks */

	uint8_t rx_tx_buffer[RX_BUFFER_SIZE]; /**< Reception/Transmission buffer */

} sx1276_t;

/**
 * Hardware IO IRQ callback function definition.
 */

typedef void (sx1276_dio_irq_handler)(void);

/**
 * SX1276 definitions.
 */
#define	XTAL_FREQ		32000000 /**< 32MHz */
#define	FREQ_STEP		61.03515625
#define	RX_BUFFER_SIZE	256

/**
 * Public functions prototypes.
 */

/**
 * @brief Initializes the transceiver.
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 * @param	[IN]	events	Events structure containing callback functions
 */

void sx1276_init(sx1276_t *dev, sx1276_events_t *events);

/**
 * @brief Gets current status of transceiver.
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @return radio status [RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
sx1276_radio_state_t sx1276_get_status(sx1276_t *dev);

/**
 * @brief Sets the channel frequency
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 */
void sx1276_set_channel(sx1276_t *dev, uint32_t freq);

/**
 * @brief Checks that channel is free with specified RSSI threshold
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 * @param	[IN]	freq channel RF frequency
 * @param	[IN]	rssi_thresh RSSI treshold
 *
 * @return channel is free or not [true: channel is free, false: channel is not free]
 */
bool sx1276_is_channel_free(sx1276_t *dev, uint32_t freq, uint16_t rssi_thresh);

/**
 * @brief generates 32 bits random value based on the RSSI readings
 * This function sets the radio in LoRa mode and disables all interrupts from it.
 * After calling this function either sx1276_set_rx_config or sx1276_set_tx_config
 * functions must be called.
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @return random 32 bits value
 */
uint32_t sx1276_random(sx1276_t *dev);

/**
 * @brief Sets the reception parameters
 * Only bandwidths 125, 250 and 500 kHz are supported due to LoRa usage.
 *
 * @param	[IN]	dev				The sx1276 device structure pointer
 *
 * @param	[IN]	modem			Modem to be configured
 *
 * @param	[IN]	bandwidth		Sets the bandwidth
 *									[0: 125 kHz,	1:	250 kHz,
 * 					 				2:	500 kHz,	3:	Reserved]
 *
 * @param	[IN]	datarate		Sets the data rate
 * 									[6: 64, 7: 128, 8: 256, 9: 512,
 * 					 			 	10: 1024, 11: 2048, 12: 4096 chips]
 *
 * @param	[IN] 	coderate		Sets the LoRa coding rate
 * 									[1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 *
 * @param	[IN]	preambleLen		Sets the Preamble length
 * 									Length in symbols (the hardware adds 4 more symbols)
 *
 * @param	[IN]	symb_timeout  	Sets the RxSingle timeout value in symbols
 *
 * @param	[IN] 	fix_len			Fixed length packets [0: variable, 1: fixed]
 *
 * @param	[IN] 	payload_len		Sets payload length when fixed length is used
 *
 * @param	[IN] 	crc_on			Enables or disables the CRC [0: OFF, 1: ON]
 *
 * @param	[IN] 	freq_hop_on		Enables or disables the intra-packet frequency hopping
 *                          		[0: OFF, 1: ON]
 *
 * @param	[IN] 	hop_period		Number of symbols between each hop
 *
 * @param	[IN] 	iq_inverted		Inverts IQ signals (LoRa only)
 *									[0: not inverted, 1: inverted]
 *
 * @param	[IN] 	rx_continuous	Sets the reception in continuous mode
 *                          		[false: single mode, true: continuous mode]
 */
void sx1276_set_rx_config(sx1276_t *dev, sx1276_radio_modems_t modem, uint32_t bandwidth, uint32_t datarate,
		uint8_t coderate, uint16_t preamble_len, uint16_t symb_timeout,
		bool fix_len, uint8_t payload_len, bool crc_on, bool freq_hop_on,
		uint8_t hop_period, bool iq_inverted, bool rx_continuous);

/**
 * @brief Sets the transmission parameters
 * Only bandwidths 125, 250 and 500 kHz are supported due to LoRa usage
 *
 * @param	[IN]	dev				The sx1276 device structure pointer
 *
 * @param	[IN]	modem			Modem to be configured
 *
 * @param	[IN]	power			Sets the output power [dBm]
 *
 * @param	[IN]	bandwidth		Sets the bandwidth for LoRa
 *									[0: 125 kHz, 1: 250 kHz,
 *									2: 500 kHz, 3: Reserved]
 *
 * @param	[IN]	datarate		Sets the data rate
 *									[6: 64, 7: 128, 8: 256, 9: 512,
 *									10: 1024, 11: 2048, 12: 4096  chips]
 *
 * @param	[IN]	coderate		Sets the coding rate for LoRa
 *									[1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 *
 * @param	[IN]	preamble_len	Sets the preamble length
 *									LoRa: Length in symbols (the hardware adds 4 more symbols)
 *
 * @param	[IN]	fix_len			Fixed length packets [0: variable, 1: fixed]
 *
 * @param	[IN]	crc_on			Enables or disables the CRC [0: OFF, 1: ON]
 *
 * @param	[IN]	freq_hop_on		Enables or disables the intra-packet frequency hopping
 *									[0: OFF, 1: ON]
 *
 * @param	[IN]	hop_period		Number of symbols between each hop in number of symbols
 *
 * @param	[IN]	iq_inverted		Inverts IQ signals (LoRa only)
 *									[0: not inverted, 1: inverted]
 *
 * @param	[IN]	timeout			Transmission timeout [us]
 */

void sx1276_set_tx_config(sx1276_t *dev, sx1276_radio_modems_t modem, int8_t power, uint32_t bandwidth,
		uint32_t datarate, uint8_t coderate, uint16_t preamble_len,
		bool fix_len, bool crc_on, bool freq_hop_on, uint8_t hop_period,
		bool iq_inverted, uint32_t timeout);
/**
 * @brief Computes the packet time on air in us for the given payload
 * Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * @param	[IN]	dev			The sx1276 device structure pointer
 *
 * @param	[IN]	modem		Modem to use
 *
 * @param	[IN]	pk_len		Packet payload length
 *
 * @return computed air time (us) for the given packet payload length
 */
uint32_t sx1276_get_time_on_air(sx1276_t *dev, sx1276_radio_modems_t modem, uint8_t pkt_len);

/**
 * @brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * @param	[IN]	dev			The sx1276 device structure pointer
 *
 * @param	[IN]	buffer		Buffer pointer
 *
 * @param	[IN]	size		Buffer size
 */
void sx1276_send(sx1276_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Sets the radio in sleep mode
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 */
void sx1276_set_sleep(sx1276_t *dev);

/**
 * @brief Sets the radio in stand-by mode
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 */
void sx1276_set_standby(sx1276_t *dev);

/**
 * @brief Sets the radio in reception mode for given amount of time
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	timeout	reception timeout [us] [0: continuous, others: timeout]
 */
void sx1276_set_rx(sx1276_t *dev, uint32_t timeout);

/**
 * @brief Sets the radio in transmission mode for given amount of time
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	timeout	reception timeout [us] [0: continuous, others: timeout]
 */
void sx1276_set_tx(sx1276_t *dev, uint32_t timeout);

/**
 * @brief Start a channel activity detection
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 */

void sx1276_start_cad(sx1276_t *dev);

/**
 * @brief Reads the current RSSI value
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @return current value of RSSI in [dBm]
 */
int16_t sx1276_read_rssi(sx1276_t *dev);

/**
 * @brief Writes the radio register at specified address
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	addr Register address
 *
 * @param	[IN]	data New register value
 */
void sx1276_reg_write(sx1276_t *dev, uint8_t addr, uint8_t data);

/**
 * @brief Reads the radio register at specified address
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	addr	Register address
 *
 * @return	Register value
 */
uint8_t sx1276_reg_read(sx1276_t *dev, uint8_t addr);

/**
 * @brief Writes multiple radio registers starting at address (burst-mode)
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	addr	First radio register address
 *
 * @param	[IN]	buffer	buffer containing the new register's values
 *
 * @param	[IN]	size	Number of registers to be written
 */
void sx1276_reg_write_burst(sx1276_t *dev, uint8_t addr, uint8_t *buffer,
		uint8_t size);

/**
 * @brief Reads multiple radio registers starting at address
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	addr	First radio register address
 *
 * @param	[OUT]	bufer	Buffer where to copy registers data
 *
 * @param	[IN]	size	Number of registers to be read
 */
void sx1276_reg_read_burst(sx1276_t *dev, uint8_t addr, uint8_t *buffer,
		uint8_t size);

/**
 * @brief Sets the maximum payload length
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @param	[IN]	modem	Modem to use
 *
 * @param	[IN]	maxlen	Maximum payload length in bytes
 */
void sx1276_set_max_payload_len(sx1276_t *dev, sx1276_radio_modems_t modem, uint8_t maxlen);

#ifdef __cplusplus
}
#endif

#endif /* SX1276_H */

/** @} */
