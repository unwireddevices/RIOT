/*
 * Copyright (C) 2016 Unwired Devices
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
 * @file		ls-gate-device-list.h
 * @brief       Device list definitions
 * @author      Eugene Ponomarev
 */
#ifndef UNWIRED_MODULES_LORA_STAR_GATE_INCLUDE_LS_GATE_DEVICE_LIST_H_
#define UNWIRED_MODULES_LORA_STAR_GATE_INCLUDE_LS_GATE_DEVICE_LIST_H_

#include <stdint.h>
#include <stdbool.h>

#include "mutex.h"

#include "ls-crypto.h"
#include "ls-mac-types.h"

#include "ls-frame-fifo.h"

/**
 * Max devices number that gate can hold
 */
#define LS_GATE_MAX_NODES 100

/**
 * Max nonce count to remember in order to prevent nonce reuse
 */
#define LS_GATE_MAX_NONCES 128

typedef struct {
	int32_t nonce;
	void* next;
} ls_gate_node_nl_entry_t;

typedef struct {
	void *node_ch;				/**< Node's channel */

	uint64_t node_id;			/**< Node unique ID */
	uint64_t app_id;			/**< Application unique ID */

	ls_node_class_t node_class;	/**< Node's class */
	uint64_t node_ability;		/**< Node's ability */

	ls_addr_t addr;				/**< Node unique address in network */
	uint32_t last_seen;			/**< Time of the last node's activity in network */

	uint32_t last_nonce;		/**< Last actual device nonce*/
	uint32_t app_nonce;			/**< Application nonce */

	ls_gate_node_nl_entry_t *nonce_list; /**< List of accepted device nonces */
	uint8_t num_nonces;			/**< Number of remembered nonces  */

	ls_device_status_t status;	/**< Last received device status */

	uint8_t num_pending;		/**< Number of frames pending */
} ls_gate_node_t;

typedef struct {
	ls_gate_node_t nodes[LS_GATE_MAX_NODES];
	bool nodes_free_list[LS_GATE_MAX_NODES];

	size_t num_nodes;

	mutex_t mutex;
} ls_gate_devices_t;

void ls_devlist_init(ls_gate_devices_t *devlist);
ls_gate_node_t *ls_devlist_add(ls_gate_devices_t *devlist, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch);

bool ls_devlist_check_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce);
ls_gate_node_t *add_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce);

ls_gate_node_t *ls_devlist_get(ls_gate_devices_t *devlist, ls_addr_t addr);
ls_gate_node_t *ls_devlist_get_by_nodeid(ls_gate_devices_t *devlist, uint64_t nodeid);

bool ls_devlist_is_added(ls_gate_devices_t *devlist, uint64_t node_id);
bool ls_devlist_is_in_network(ls_gate_devices_t *devlist, ls_addr_t addr);
bool ls_devlist_is_full(ls_gate_devices_t *devlist);

bool ls_devlist_remove_device(ls_gate_devices_t *devlist, ls_addr_t addr);

#endif /* UNWIRED_MODULES_LORA_STAR_GATE_INCLUDE_LS_GATE_DEVICE_LIST_H_ */
