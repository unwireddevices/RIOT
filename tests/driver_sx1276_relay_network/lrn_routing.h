/*
 * Copyright (C) 2016 Evgeniy Ponomarev
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		lrn_routing.h
 * @brief       LRN static routing definitions
 * @author      Evgeniy Ponomarev
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_ROUTING_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_ROUTING_H_

#include "lrn_mac_types.h"
#include "lrn.h"

#define LRN_MAX_ROUTING_TABLE_SIZE 75

typedef struct {
	lrn_addr_t addr;			/**< Node address */
	lrn_datarate_t dr;			/**< Data rate for this node */

	bool is_gateway;			/**< Is this node a gateway */
} lrn_routing_table_entry_t;

typedef struct {
	uint8_t table_size;

	lrn_routing_table_entry_t table[LRN_MAX_ROUTING_TABLE_SIZE];
} lrn_join_ack_t;

lrn_routing_table_entry_t lrn_routing_next_hop(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t my_addr, lrn_addr_t dest);
lrn_addr_t lrn_routing_find_gateway_addr(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t my_addr, lrn_addr_t src);
int lrn_routing_nohops(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t from, lrn_addr_t to);

void lrn_routing_load_eeprom(lrn_routing_table_entry_t *routing_table, int *rt_size);
void lrn_routing_store_eeprom(lrn_routing_table_entry_t *routing_table, int rt_size);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_ROUTING_H_ */
