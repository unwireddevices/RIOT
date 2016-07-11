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
 * @file		lrn_routing.c
 * @brief       LRN static routing implementation
 * @author      cr0s
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "lrn.h"
#include "lrn_mac_types.h"
#include "lrn_routing.h"

lrn_routing_table_entry_t lrn_routing_next_hop(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t my_addr, lrn_addr_t dest) {
	int i;
	int my_pos = 0;
	int dest_pos = 0;

	/* Lookup table for the our position and the position of the destination node */
	for(i = 0; i < table_size; i++) {
		if (routing_table[i].addr == my_addr)
			my_pos = i;

		if (routing_table[i].addr == dest)
			dest_pos = i;
	}

	/* Determine neighbor's selection direction */
	int d = dest_pos - my_pos;

	/* Select neighbor */
	i = (d > 0) ? my_pos + 1 : my_pos - 1;

	/* Unreachable address */
	if (i < 0 || i >= table_size) {
		lrn_routing_table_entry_t e;
		e.addr = LRN_ADDR_UNDEFINED;
		e.dr = DR3;

		return e;
	}

	return routing_table[i];
}

lrn_addr_t lrn_routing_find_gateway_addr(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t my_addr, lrn_addr_t src) {
	int gw_pos = -1, my_pos = -1, src_pos = -1;

	/* Find positions in table */
	for(int i = 0; i < table_size; i++) {
		if (routing_table[i].is_gateway)
			gw_pos = i;

		if (routing_table[i].addr == my_addr)
			my_pos = i;

		if (routing_table[i].addr == src)
			src_pos = i;
	}

	if (my_pos == -1)
		return LRN_ADDR_UNDEFINED;

	if (src_pos == -1)
		return LRN_ADDR_UNDEFINED;

	if (gw_pos == -1)
		return LRN_ADDR_UNDEFINED;

	if (gw_pos > my_pos) {
		if (my_pos > src_pos)
			return routing_table[gw_pos].addr;
	} else if (gw_pos < my_pos) {
		if (my_pos < src_pos)
			return routing_table[gw_pos].addr;
	}

	return LRN_ADDR_UNDEFINED;
}

int lrn_routing_nohops(lrn_routing_table_entry_t *routing_table, int table_size, lrn_addr_t from, lrn_addr_t to) {
	int fpos = -1, tpos = -1;

	/* Find positions in table */
	for(int i = 0; i < table_size; i++) {
		if (routing_table[i].addr == from)
			fpos = i;

		if (routing_table[i].addr == to)
			tpos = i;
	}

	int res = tpos - fpos;

	return (res < 0) ? -res : res;
}

#ifdef __cplusplus
}
#endif
