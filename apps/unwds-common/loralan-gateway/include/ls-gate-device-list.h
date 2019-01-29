/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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
#include "bloom.h"
#include "bitfield.h"
#include "hashes.h" /* for bloom filter */

#include "ls-crypto.h"
#include "ls-mac-types.h"

#include "ls-frame-fifo.h"

/**
 * Max device number that gate can hold simultaneously and nonce counts per device to remember. Nonces used by node are stored in Bloom filter.
 * depend on available RAM
 */

#if defined(CPU_FAM_STM32L4)
    #define LS_GATE_MAX_NODES 1000
    #define LS_GATE_NONCES_PER_DEVICE 128
#else
    #define LS_GATE_MAX_NODES 100
    #define LS_GATE_NONCES_PER_DEVICE 32
#endif

typedef struct __attribute__((__packed__)){
    uint64_t node_id;			/**< Node unique ID */
	uint64_t app_id;			/**< Application unique ID */    
	uint32_t last_seen;			/**< Time of the last node's activity in network */
	uint32_t app_nonce;			/**< Application nonce */
    ls_addr_t addr;				/**< Node unique address in network */
	void *node_ch;				/**< Node's channel */
    bloom_t nonces;                                     /**< Bloom filter of used nonces, gives number of possibly remembered nonces proportional to the filter size */
    ls_nonce_t last_nonce;                              /**< Last accepted nonce used for key derivation */
    uint8_t bloom_bits[LS_GATE_NONCES_PER_DEVICE / 8];  /**< Bitset for Bloom filter */
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
