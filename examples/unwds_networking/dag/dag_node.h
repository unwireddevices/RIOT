/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_gnrc_pktdump Dump Network Packets
 * @ingroup     net_gnrc
 * @brief       Dump network packets to STDOUT for debugging
 *
 * @{
 *
 * @file
 * @brief       Interface for a generic network packet dumping module
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef DAG_NODE_H
#define DAG_NODE_H

// #include "kernel_types.h"
// #include "protocol.h"

#define CLICK					0x00
#define LONG_CLICK				0x80

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief dag_node_init
 *  
 * @return 
 */
int dag_node_init(void);

/**
 * @brief Конструктор пакета
 *
 * @param[in]		device_id	device_id 
 * @param[in]       data_type	data_type
 * @param[in]       payload_len	payload_len
 * @param[in]       payload		payload
 */
void unwds_pack_sender( uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload);

/**
 * @brief Конструктор пакета
 *
 * @param[in]		button_number	button_number 
 * @param[in]       click_type		click_type
 */						
void button_status_dag_sender ( uint8_t button_number,
								uint8_t click_type);

/**
 * @brief Совершить замер освещенности
 *
 */										
bool lit_measure_dag_sender(void);

#ifdef __cplusplus
}
#endif

#endif /* DAG_NODE_H */
/** @} */
