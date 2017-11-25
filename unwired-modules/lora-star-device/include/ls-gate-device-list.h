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
#ifndef LS_GATE_DEVICE_LIST_H_
#define LS_GATE_DEVICE_LIST_H_

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
#define LS_GATE_NONCES_PER_DEVICE 8

typedef struct __attribute__((__packed__)){
    uint64_t node_id;			/**< Node unique ID */
	uint64_t app_id;			/**< Application unique ID */    
	uint32_t last_seen;			/**< Time of the last node's activity in network */
	uint32_t app_nonce;			/**< Application nonce */
    ls_addr_t addr;				/**< Node unique address in network */
	void *node_ch;				/**< Node's channel */
    ls_nonce_t nonce[LS_GATE_NONCES_PER_DEVICE]; /**< List of accepted device nonces */
	ls_node_class_t node_class;	/**< Node's class */
    ls_device_status_t status;	/**< Last received device status */
	ls_frame_id_t last_fid;		/**< Last received frame ID */
	uint8_t num_nonces;			/**< Number of remembered nonces  */
	uint8_t num_pending;		/**< Number of frames pending */
	bool is_static;				/**< Statically personalized device, won't be kicked for idle */
} ls_gate_node_t;

typedef struct {
	ls_gate_node_t nodes[LS_GATE_MAX_NODES];
	bool nodes_free_list[LS_GATE_MAX_NODES];
    size_t num_nodes;
    mutex_t mutex;
} ls_gate_devices_t;

void ls_devlist_init(ls_gate_devices_t *devlist);
ls_gate_node_t *ls_devlist_add(ls_gate_devices_t *devlist, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch);
ls_gate_node_t *ls_devlist_add_by_addr(ls_gate_devices_t *devlist, ls_addr_t addr, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch);

bool ls_devlist_check_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce);
ls_gate_node_t *add_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce);

ls_gate_node_t *ls_devlist_get(ls_gate_devices_t *devlist, ls_addr_t addr);
ls_gate_node_t *ls_devlist_get_by_nodeid(ls_gate_devices_t *devlist, uint64_t nodeid);

bool ls_devlist_is_added(ls_gate_devices_t *devlist, uint64_t node_id);
bool ls_devlist_is_in_network(ls_gate_devices_t *devlist, ls_addr_t addr);
bool ls_devlist_is_full(ls_gate_devices_t *devlist);

bool ls_devlist_remove_device(ls_gate_devices_t *devlist, ls_addr_t addr);

#endif /* LS_GATE_DEVICE_LIST_H_ */
