/*
 * Copyright (C) 2015-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_nrf5x_nrfmax
 * @{
 *
 * @file
 * @brief       Implementation of the nrfmax radio driver for nRF51 radios
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <errno.h>

#include "cpu.h"
#include "mutex.h"
#include "assert.h"

#include "periph_conf.h"
#include "periph/cpuid.h"

#include "nrfmax.h"
#include "net/netdev.h"

#ifdef MODULE_GNRC_SIXLOWPAN
#include "net/gnrc/nettype.h"
#endif

#define ENABLE_DEBUG            (0)
#include "debug.h"

/**
 * @brief   Set the 64-bit radio address
 *
 * @param[in] addr      address to set
 */
void nrfmax_set_eui64(eui64_t *get_eui64)
{
	memcpy(&nrfmax_eui64, get_eui64, sizeof(eui64_t));
}

/**
 * @brief   Get the currently active address

 * @return  the 64-bit node address
 */
void nrfmax_get_eui64(eui64_t *get_eui64)
{
	memcpy(get_eui64, &nrfmax_eui64, sizeof(eui64_t));
}

/**
 * @brief   Get the IID build from the 16-bit node address
 *
 * @param[out] iid      the 64-bit IID, as array of 4 * 16-bit
 */
// void nrfmax_get_iid(uint16_t *iid)
// {
    // iid[0] = 0;
    // iid[1] = 0xFF00;
    // iid[2] = 0x00FE;
    // iid[3] = my_addr;
// }

/**
 * @brief   Get the current channel
 *
 * @return  currently active channel
 */
uint16_t nrfmax_get_channel(void)
{
    return (uint16_t)(NRF_RADIO->FREQUENCY >> 2);
}

/**
 * @brief   Get the current frequency
 *
 * @return  currently active frequency
 */
uint32_t nrfmax_get_frequency(void)
{
	return (uint32_t)(2400 + NRF_RADIO->FREQUENCY);
}

/**
 * @brief   Get the current radio state
 *
 * @return  state the radio is currently in
 */
netopt_state_t nrfmax_get_state(void)
{
    switch (nrfmax_state) {
        case STATE_OFF:  return NETOPT_STATE_OFF;
        case STATE_IDLE: return NETOPT_STATE_SLEEP;
        case STATE_RX:   return NETOPT_STATE_IDLE;
        case STATE_TX:   return NETOPT_STATE_TX;
        default:         return NETOPT_STATE_RESET;     /* should never show */
    }
}

/**
 * @brief   Get the current transmit power
 *
 * @return  transmission power in [dBm]
 */
int16_t nrfmax_get_txpower(void)
{
    int8_t p = (int8_t)NRF_RADIO->TXPOWER;
    if (p < 0) {
        return (int16_t)(0xFF00 | p);
    }
    return (int16_t)p;
}

/**
 * @brief   Set the active channel
 *
 * @param[in] chan      targeted channel [0-31]
 *
 * @return  sizeof(uint16_t) on success
 * @return  -EOVERFLOW if channel is not applicable
 */
int nrfmax_set_channel(uint16_t chan)
{
    if (chan > NRFMAX_CHAN_MAX) {
        return -EOVERFLOW;
    }

    NRF_RADIO->FREQUENCY = (chan << 2);
    nrfmax_goto_target_state();

    return sizeof(uint16_t);
}

/**
 * @brief   Set the used transmission power
 *
 * @param[in] power     targeted power, in [dBm]
 */
void nrfmax_set_txpower(int16_t power)
{
    if (power > 2) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;
    }
    else if (power > -2) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm;
    }
    else if (power > -6) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg4dBm;
    }
    else if (power > -10) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg8dBm;
    }
    else if (power > -14) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg12dBm;
    }
    else if (power > -18) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg16dBm;
    }
    else if (power > -25) {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg20dBm;
    }
    else {
        NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Neg40dBm;
    }
}

/**
 * @brief   Put the device into the given state
 *
 * @param[in] val       target state
 *
 * @return  sizeof(netopt_state_t) on success
 * @return  -ENOTSUP if target state is not applicable
 */
int nrfmax_set_state(netopt_state_t val)
{
    /* Make sure radio is turned on and no transmission is in progress */
    NRF_RADIO->POWER = 1;

    switch (val) {
        case NETOPT_STATE_OFF:
            nrfmax_target_state = STATE_OFF;
            break;
        case NETOPT_STATE_SLEEP:
            nrfmax_target_state = STATE_IDLE;
            break;
        case NETOPT_STATE_IDLE:
            nrfmax_target_state = STATE_RX;
            break;
        default:
            return -ENOTSUP;
    }

    nrfmax_goto_target_state();

    return sizeof(netopt_state_t);
}