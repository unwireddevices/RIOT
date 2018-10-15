/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "cpu.h"

/* These are set to zero as ShockBurst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      	(0UL)  		/**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      	(0UL)  		/**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  	(8UL)  		/**< Packet length field size in bits. */

#define PACKET_BASE_ADDRESS_LENGTH  (4UL) 		//!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH        (0UL) 		//!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE      (128UL)		//!< Packet payload maximum size in bytes

#define RADIO_CHANNEL_11			(5UL)
#define RADIO_CHANNEL_12    		(10UL)
#define RADIO_CHANNEL_13    		(15UL)
#define RADIO_CHANNEL_14    		(20UL)
#define RADIO_CHANNEL_15    		(25UL)
#define RADIO_CHANNEL_16    		(30UL)
#define RADIO_CHANNEL_17    		(35UL)
#define RADIO_CHANNEL_18    		(40UL)
#define RADIO_CHANNEL_19    		(45UL)
#define RADIO_CHANNEL_20    		(50UL)
#define RADIO_CHANNEL_21    		(55UL)
#define RADIO_CHANNEL_22    		(60UL)
#define RADIO_CHANNEL_23    		(65UL)
#define RADIO_CHANNEL_24    		(70UL)
#define RADIO_CHANNEL_25    		(75UL)
#define RADIO_CHANNEL_26    		(80UL)

#define RX_BUFFER_LEN    			(128)

static uint8_t rx_buffer[RX_BUFFER_LEN];		/*Packet to transmit*/

/**
 * @brief Function for swapping/mirroring bits in a byte.
 *
 *@verbatim
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *@endverbatim
 *
 * @param[in] inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped/mirrored input byte.
 */
static uint32_t swap_bits(uint32_t inp);

/**
 * @brief Function for swapping bits in a 32 bit word for each byte individually.
 *
 * The bits are swapped as follows:
 * @verbatim
 * output[31:24] = input[24:31]
 * output[23:16] = input[16:23]
 * output[15:8]  = input[8:15]
 * output[7:0]   = input[0:7]
 * @endverbatim
 * @param[in] input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}


/**
 * @brief Function for configuring the radio to operate in ShockBurst compatible mode.
 *
 * To configure the application running on nRF24L series devices:
 *
 * @verbatim
 * uint8_t tx_address[5] = { 0xC0, 0x01, 0x23, 0x45, 0x67 };
 * hal_nrf_set_rf_channel(7);
 * hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
 * hal_nrf_set_address(HAL_NRF_TX, tx_address);
 * hal_nrf_set_address(HAL_NRF_PIPE0, tx_address);
 * hal_nrf_open_pipe(0, false);
 * hal_nrf_set_datarate(HAL_NRF_1MBPS);
 * hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
 * hal_nrf_setup_dynamic_payload(0xFF);
 * hal_nrf_enable_dynamic_payload(false);
 * @endverbatim
 *
 * When transmitting packets with hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length),
 * match the length with PACKET_STATIC_LENGTH.
 * hal_nrf_write_tx_payload(payload, PACKET_STATIC_LENGTH);
 *
*/
static void radio_configure(void)
{
    // Radio config
    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->FREQUENCY = RADIO_CHANNEL_26; 
    NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

    // Radio address config
    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                       (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                       (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
                       (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |
                       (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
}

/**@brief Function for reading packet.
 */
uint32_t read_packet(void)
{
	NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;
	
    uint32_t result = 0;

    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    if (NRF_RADIO->CRCSTATUS == 1U)
    {
        result = (uint32_t)rx_buffer;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    return result;
}

int main(void)
{
    puts("Radio receiver example started.");
	puts("Wait for first packet");

	// Set radio configuration parameters
    radio_configure();
	
	while (true)
    {
        uint8_t *received = (uint8_t*)read_packet();
		
		if(received != 0)
		{
			puts("Packet was received");

			printf("The contents of the package is \n");
			
			for(uint8_t i = 1; i <= received[0]; i++)
				printf("%i ", received[i]);
			
			printf("\n");
		}
		
    }
		
    return 0;
}




