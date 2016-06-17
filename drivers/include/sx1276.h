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

#define RADIO_WAKEUP_TIME                           1000        /**< [us] */
#define CHANNEL_HF                                  868000000   /**< [Hz] */

#define RX_BUFFER_SIZE                              256

#define RF_MID_BAND_THRESH                          525000000
                                               \
    # ifdef __cplusplus
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
    bool implicit_header;
    uint8_t payload_len;
    bool crc_on;
    bool freq_hop_on;
    uint8_t hop_period;
    bool iq_inverted;
    bool rx_continuous;
    uint32_t tx_timeout;

} sx1276_lora_settings_t;

/**
 * LoRa received packet.
 */
typedef struct {
    uint8_t snr_value;
    int16_t rssi_value;

    char *content;
    uint8_t size;
} sx1276_rx_packet_t;

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
    sx1276_radio_modems_t modem;

} sx1276_settings_t;

typedef enum {
    RX_DONE = 0,
    TX_DONE,

    RX_TIMEOUT,
    TX_TIMEOUT,

    RX_ERROR,

    FHSS_CHANGE_CHANNEL,
    CAD_DONE,

} sx1276_event_type_t;

typedef struct {
    sx1276_event_type_t type;

    void *event_data;
} sx1276_event_t;

/**
 * SX1276 hardware and global parameters.
 */
typedef struct sx1276_s {
    spi_t spi;          /**< SPI */
    gpio_t nss_pin;     /**< SPI NSS pin */

    gpio_t reset_pin;   /**< Reset pin */
    gpio_t dio0_pin;
    gpio_t dio1_pin;
    gpio_t dio2_pin;
    gpio_t dio3_pin;
    gpio_t dio4_pin;
    gpio_t dio5_pin;

    uint8_t rxtx;
    sx1276_settings_t settings;

    kernel_pid_t event_handler_thread_pid;
    kernel_pid_t dio_polling_thread_pid;    /**< sx1276 DIO interrupt line flags */
} sx1276_t;

/**
 * Hardware IO IRQ callback function definition.
 */

typedef void (sx1276_dio_irq_handler)(sx1276_t *dev);

/**
 * SX1276 definitions.
 */
#define XTAL_FREQ       32000000 /**< 32MHz */
#define FREQ_STEP       61.03515625
#define RX_BUFFER_SIZE  256

/**
 * Public functions prototypes.
 */

/**
 * @brief Tests the transceiver
 *
 * @param	[IN]	dev	The sx1276 device structure pointer
 * @return true if test passed, false otherwise
 */
bool sx1276_test(sx1276_t *dev);

/**
 * @brief Initializes the transceiver.
 *
 * @param	[IN]	dev					The sx1276 device structure pointer
 */

void sx1276_init(sx1276_t *dev);

/**
 * @brief Gets current status of transceiver.
 *
 * @param	[IN]	dev		The sx1276 device structure pointer
 *
 * @return radio status [RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
sx1276_radio_state_t sx1276_get_status(sx1276_t *dev);

/**
 * @brief Configures the radio with the given modem
 *
 * @param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void sx1276_set_modem(sx1276_t *dev, sx1276_radio_modems_t modem);

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
 * @brief Reads the raw temperature value of the chip
 *
 * @param	[IN]	dev	The sx1276 device pointer
 *
 * @return signed 8 bit integer value of the current temperature of the sx1276 chip
 */
int8_t sx1276_read_temp(sx1276_t *dev);

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
 *                                  2:	500 kHz,	3:	Reserved]
 *
 * @param	[IN]	datarate		Sets the data rate
 *                                  [6: 64, 7: 128, 8: 256, 9: 512,
 *                                  10: 1024, 11: 2048, 12: 4096 chips]
 *
 * @param	[IN]    coderate		Sets the LoRa coding rate
 *                                  [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 *
 * @param	[IN]	preambleLen		Sets the Preamble length
 *                                  Length in symbols (the hardware adds 4 more symbols)
 *
 * @param	[IN]	symb_timeout    Sets the RxSingle timeout value in symbols
 *
 * @param	[IN]    implicit_header	Implicit header mode [0: explicit, 1: implicit]
 *
 * @param	[IN]    payload_len		Sets payload length when fixed length is used
 *
 * @param	[IN]    crc_on			Enables or disables the CRC [0: OFF, 1: ON]
 *
 * @param	[IN]    freq_hop_on		Enables or disables the intra-packet frequency hopping
 *                                  [0: OFF, 1: ON]
 *
 * @param	[IN]    hop_period		Number of symbols between each hop
 *
 * @param	[IN]    iq_inverted		Inverts IQ signals (LoRa only)
 *									[0: not inverted, 1: inverted]
 *
 * @param	[IN]    rx_continuous	Sets the reception in continuous mode
 *                                  [false: single mode, true: continuous mode]
 */
void sx1276_set_rx_config(sx1276_t *dev, sx1276_radio_modems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint32_t bandwidth_afc, uint16_t preamble_len,
                          uint16_t symb_timeout, bool implicit_header,
                          uint8_t payload_len,
                          bool crc_on, bool freq_hop_on, uint8_t hop_period,
                          bool iq_inverted, bool rx_continuous);

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
 * @param	[IN]    implicit_header	Implicit header mode [0: explicit, 1: implicit]
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

void sx1276_set_tx_config(sx1276_t *dev, sx1276_radio_modems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preamble_len,
                          bool implicit_header, bool crc_on, bool freq_hop_on,
                          uint8_t hop_period, bool iq_inverted, uint32_t timeout);
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

/**
 * @brief sx1276 state machine hanlder thread body.
 *
 * @param	[IN]	arg	an sx1276 device instance
 */
void *dio_polling_thread(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* SX1276_H */

/** @} */
