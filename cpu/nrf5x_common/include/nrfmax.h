/*
 * Copyright (C) 2015-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_nrf5x_nrfmax NRF minimal radio driver
 * @ingroup     drivers_netdev
 * @brief       Minimal driver for the NRF51 radio
 *
 * This driver uses the nRF5x radio in a proprietary/custom way, defining our
 * own custom link layer. This custom link layer resembles some characteristics
 * of the IEEE802.15.4 link layer, but is not at all compatible to it.
 *
 * One key point is, that this custom link layer is only meant to operate
 * between nRF5x devices, which let's us make some very nice assumptions:
 *  - all communicating hosts are little-endian
 *    -> we define host byte order := network byte order
 *
 * The driver is using a Nordic proprietary physical layer, configured to a
 * bitrate of 1Mbit. The maximum payload length can be freely configured, but
 * the maximal supported value is 250 byte (default is 200 byte).
 *
 * We define the nrfmax link layer to use 64-bit addresses. On the physical
 * layer we encode these addresses by putting these addresses into the 5 least
 * significant bytes of the supported 5-byte addresses.
 *
 * For out custom link layer, we define our own proprietary link layer format
 * (all fields are in host byte order (little endian)):
 *
 *    byte0 | byte1 - byte8 | byte9 - byte16 | byte17 | byte18 - byteN
 *   ------ | ------------- | -------------- | ------ | -------------
 *   length |   src_addr    |    dst_addr    | proto  |   payload...
 *
 * With:
 * - length: length of the packet, including the header -> payload len + 17
 * - src_addr: 64-bit source address
 * - dst_addr: 64-bit destination address
 * - proto: type of data transferred (similar to an Ethertype field)
 *
 * SUMMARY:
 * This driver / link layer supports:
 *   - 64-bit addressing (64-bit)
 *     -> extract default address from CPU ID
 *   - broadcast (broadcast address is ff:ff:ff:ff:ff:ff:ff:ff)
 *   - channels from 0 to 31 [2400MHz to 2524MHz, 4MHz per channel]
 *   - setting of TX power [+4dBm to -20dBm, in ~4dBm steps]
 *   - 8-bit packet type/proto field (to be used as seen fit)
 *   - setting device state (RX, SLEEP)
 *
 * But so far no support for:
 *   - link layer ACKs
 *   - retransmissions
 *
 * @todo        So far the driver uses only a single RX buffer that is locked
 *              until the data was read/discarded. This can potentially lead to
 *              a lot of packet loss -> using more than one buffer would help
 *              here...
 *
 * @{
 *
 * @file
 * @brief       Interface definition for the nrfmax NRF51822 radio driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 */

#ifndef NRFMAX_H
#define NRFMAX_H

#include "net/netdev.h"
#include "net/gnrc/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   nrfmax channel configuration
 * @{
 */
#define NRFMAX_CHAN_MIN             (0U)	/* Minimal channel */
#define NRFMAX_CHAN_DEFAULT         (0U)	/* 2400MHz */
#define NRFMAX_CHAN_MAX             (32)	/* Maximum channel */
/** @} */

/**
 * @brief   Default transmission power used
 */
#define NRFMAX_TXPOWER_DEFAULT      (RADIO_TXPOWER_TXPOWER_Pos4dBm)			/* 4dBm */

/**
 * @brief   Default maximum payload length (must be <= 250)
 */
#ifndef NRFMAX_PAYLOAD_MAX
#define NRFMAX_PAYLOAD_MAX          (200U)
#endif

/**
 * @brief   Export some information on header and packet lengths
 * @{
 */
#define NRFMAX_HDR_LEN              (sizeof(nrfmax_hdr_t))					/* Header lengths */				
#define NRFMAX_PKT_MAX              (NRFMAX_HDR_LEN + NRFMAX_PAYLOAD_MAX)	/* Maximum packet lengths */
/** @} */

/**
 * @brief   Header format used for our custom nrfmax link layer
 */
typedef struct __attribute__((packed)) {
    uint8_t len;            /**< packet length, including this header */
    eui64_t src_addr;      	/**< source address of the packet */
    eui64_t dst_addr;     	/**< destination address */
    uint8_t proto;          /**< protocol of payload */
} nrfmax_hdr_t;

/**
 * @brief   In-memory structure of a nrfmax radio packet
 */
typedef union {
    struct __attribute__((packed)) {
        nrfmax_hdr_t hdr;                       /**< the nrfmax header */
        uint8_t payload[NRFMAX_PAYLOAD_MAX];    /**< actual payload */
    } pkt;                                      /**< typed packet access */
    uint8_t raw[NRFMAX_PKT_MAX];                /**< raw packet access */
} nrfmax_pkt_t;

/**
 * @brief   Possible internal device states
 */
typedef enum {
    STATE_OFF,                  /**< device is powered off */
    STATE_IDLE,                 /**< device is in idle mode */
    STATE_RX,                   /**< device is in receive mode */
    STATE_TX,                   /**< device is transmitting data */
} state_t;

/**
 * @brief   Export the netdev device descriptor
 */
extern netdev_t nrfmax_dev;

/**
 * @brief   For faster lookup we remember our own 64-bit address
 */
extern eui64_t nrfmax_eui64;

/**
 * @brief   We need to keep track of the radio state in SW (-> PAN ID 20)
 *
 * See nRF51822 PAN ID 20: RADIO State Register is not functional.
 */
extern volatile state_t nrfmax_state;

/**
 * @brief   We also remember the 'long-term' state, so we can resume after TX
 */
extern volatile state_t nrfmax_target_state;

/**
 * @brief   Reference to the netdev driver interface
 */
extern const netdev_driver_t nrfmax_netdev;

/**
 * @brief   Set radio into the target state as defined by `nrfmax_target_state`
 *
 * Trick here is, that the driver can go back to it's previous state after a
 * send operation, so it can differentiate if the driver was in DISABLED or in
 * RX mode before the send process had started.
 */
void nrfmax_goto_target_state(void);

/**
 * @brief   Setup the device driver's data structures
 */
void nrfmax_setup(void);

/**
 * @brief   Set the 64-bit radio address
 *
 * @param[in] addr      address to set
 */
void nrfmax_set_eui64(eui64_t *get_eui64);

/**
 * @brief   Get the currently active address

 * @return  the 64-bit node address
 */
void nrfmax_get_eui64(eui64_t *get_eui64);

/**
 * @brief   Get the IID build from the 16-bit node address
 *
 * @param[out] iid      the 64-bit IID, as array of 4 * 16-bit
 */
// void nrfmax_get_iid(uint16_t *iid);

/**
 * @brief   Get the current channel
 *
 * @return  currently active channel
 */
uint16_t nrfmax_get_channel(void);

/**
 * @brief   Get the current frequency
 *
 * @return  currently active frequency
 */
uint32_t nrfmax_get_frequency(void);

/**
 * @brief   Set the active channel
 *
 * @param[in] chan      targeted channel [0-31]
 *
 * @return  sizeof(uint16_t) on success
 * @return  -EOVERFLOW if channel is not applicable
 */
int nrfmax_set_channel(uint16_t chan);

/**
 * @brief   Get the current radio state
 *
 * @return  state the radio is currently in
 */
netopt_state_t nrfmax_get_state(void);

/**
 * @brief   Put the device into the given state
 *
 * @param[in] val       target state
 *
 * @return  sizeof(netopt_state_t) on success
 * @return  -ENOTSUP if target state is not applicable
 */
int nrfmax_set_state(netopt_state_t val);

/**
 * @brief   Get the current transmit power
 *
 * @return  transmission power in [dBm]
 */
int16_t nrfmax_get_txpower(void);

/**
 * @brief   Set the used transmission power
 *
 * @param[in] power     targeted power, in [dBm]
 */
void nrfmax_set_txpower(int16_t power);



#ifdef __cplusplus
}
#endif

#endif /* NRFMAX_H */
/** @} */
