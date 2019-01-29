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
 * @file		ls-gate-device-list.c
 * @brief       Device list implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "xtimer.h"
#include "mutex.h"

#include "hashes.h"
#include "bloom.h"

#include "ls-mac-types.h"
#include "ls-gate-device-list.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* Set of different hash functions used for Bloom filter of used nonces */
#define BLOOM_NUM_HASHES (6)
hashfp_t hashes[BLOOM_NUM_HASHES] = {
    (hashfp_t) fnv_hash,
    (hashfp_t) sax_hash,
    (hashfp_t) sdbm_hash,
    (hashfp_t) djb2_hash,
    (hashfp_t) kr_hash,
    (hashfp_t) dek_hash,
};

/**
 * @brief Initialize list of connected nodes
 */
void ls_devlist_init(ls_gate_devices_t *devlist) {
	memset(devlist, 0, sizeof(ls_gate_devices_t));

	for(int i = 0; i < LS_GATE_MAX_NODES; i++) {
        ls_gate_node_t *node = &devlist->nodes[i];
        bloom_init(&node->nonces, LS_GATE_NONCES_PER_DEVICE, node->bloom_bits, hashes, BLOOM_NUM_HASHES);
        memset(node->bloom_bits, 0, ((LS_GATE_NONCES_PER_DEVICE) + 7) / 8);

		devlist->nodes_free_list[i] = true;
    }
	mutex_init(&devlist->mutex);    
    DEBUG("ls-gate-device-list: device list initialized\n");
}

/**
 * @brief Clears tracked nonces list
 */
static void clear_nonce_list(ls_gate_devices_t *devlist, ls_addr_t addr) {
    DEBUG("ls-gate-device-list: clearing nonce list\n");
	if (addr >= LS_GATE_MAX_NODES) {
        DEBUG("ls-gate-device-list: address out of range\n");
		return;
    }

	ls_gate_node_t *node = &devlist->nodes[addr];
    bloom_init(&node->nonces, LS_GATE_NONCES_PER_DEVICE, node->bloom_bits, hashes, BLOOM_NUM_HASHES);
    memset(node->bloom_bits, 0, ((LS_GATE_NONCES_PER_DEVICE) + 7) / 8);

	node->num_nonces = 0;
    
    DEBUG("ls-gate-device-list: nonce list cleared\n");
}

ls_gate_node_t *add_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce) {
    DEBUG("ls-gate-device-list: adding nonce\n");
	/* Look for a free cell (and address) to insert */
	for (uint32_t i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (!devlist->nodes_free_list[i]) {
			ls_gate_node_t *node = &devlist->nodes[i];

			if (node->node_id == node_id) {
				/* Clear nonces list if it's full */
				if (node->num_nonces == LS_GATE_NONCES_PER_DEVICE) {
					clear_nonce_list(devlist, i);
				}

				/* Add current nonce to used nonces set */
                bloom_add(&node->nonces, (uint8_t *) &nonce, sizeof(nonce));
                node->last_nonce = nonce;
                node->num_nonces++;

                DEBUG("ls-gate-device-list: nonce successfully added\n");

                return node;
			}
		}
	}
    DEBUG("ls-gate-device-list: error adding nonce\n");
	return NULL;
}

static void init_node(ls_gate_devices_t *devlist, ls_gate_node_t *node, ls_addr_t addr, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
    DEBUG("ls-gate-device-list: initialize node\n");
	node->node_ch = ch;

	node->node_id = node_id;
	node->app_id = app_id;
	node->addr = addr;
	node->is_static = false;

	/* Clear nonces list if it's full */
	if (node->num_nonces >= LS_GATE_NONCES_PER_DEVICE) {
		DEBUG("ls-gate-device-list: clear nonce list");
		clear_nonce_list(devlist, addr);
	}

    bloom_add(&node->nonces, (uint8_t *) &nonce, sizeof(nonce));
    node->last_nonce = nonce;
    node->num_nonces++;
    
    DEBUG("ls-gate-device-list: node initialized\n");
}

ls_gate_node_t *ls_devlist_add_by_addr(ls_gate_devices_t *devlist, ls_addr_t addr, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
    DEBUG("ls-gate-device-list: adding device to the list by address\n");
	/* List is full? */
	if(ls_devlist_is_full(devlist)) {
        DEBUG("ls-gate-device-list: device list is full\n");
		return NULL;
    }

	/* Device is already added? */
	if (ls_devlist_is_added(devlist, node_id)) {
        DEBUG("ls-gate-device-list: device already added to the list\n");
		return NULL;
    }

	if (addr >= LS_GATE_MAX_NODES) {
        DEBUG("ls-gate-device-list: maximum node count has been reached\n");
		return NULL;
    }

	/* This network address is occupied */
	if (!devlist->nodes_free_list[addr]) {
        DEBUG("ls-gate-device-list: network address already occupied\n");
		return NULL;
    }

	mutex_lock(&devlist->mutex);

	/* Occupy node record */
	devlist->nodes_free_list[addr] = false;

	/* Fill node record */
	ls_gate_node_t *node = &devlist->nodes[addr];
	init_node(devlist, node, addr, node_id, app_id, nonce, ch);

	node->last_fid = 255;
	node->app_nonce = 0;
	node->is_static = true;

    bloom_add(&node->nonces, (uint8_t *) &nonce, sizeof(nonce));
    node->num_nonces = 1;

	/* Increase number of connected devices */
	devlist->num_nodes++;

	mutex_unlock(&devlist->mutex);
    DEBUG("ls-gate-device-list: device successfully added\n");
	return node;
}

ls_gate_node_t *ls_devlist_add(ls_gate_devices_t *devlist, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
    DEBUG("ls-gate-device-list: adding device to the list\n");
	/* List is full? */
	if(ls_devlist_is_full(devlist)) {
        DEBUG("ls-gate-device-list: device list is full\n");
		return NULL;
    }

	/* Device is already added? */
	if (ls_devlist_is_added(devlist, node_id)) {
        DEBUG("ls-gate-device-list: device already added to the list\n");
		return NULL;
    }

	mutex_lock(&devlist->mutex);

	/* Look for a free cell (and address) to insert */
	for (uint32_t i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (devlist->nodes_free_list[i]) {
			/* Occupy node record */
			devlist->nodes_free_list[i] = false;

			/* Fill node record */
			ls_gate_node_t *node = &devlist->nodes[i];
			init_node(devlist, node, i, node_id, app_id, nonce, ch);

			/* Increase number of connected devices */
			devlist->num_nodes++;

			/* Free lock */
			mutex_unlock(&devlist->mutex);

			/* Return pointer to the node in the list */
            DEBUG("ls-gate-device-list: device successfully added\n");
			return node;
		}
	}

	mutex_unlock(&devlist->mutex);
    DEBUG("ls-gate-device-list: error adding device\n");
	return NULL;
}

bool ls_devlist_check_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce) {
    DEBUG("ls-gate-device-list: checking nonce for the device\n");
	for (uint32_t i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (devlist->nodes_free_list[i])
			continue;

		ls_gate_node_t *node = &devlist->nodes[i];

		if (node->node_id == node_id) {
			bool res = !bloom_check(&node->nonces, (uint8_t *) &nonce, sizeof(nonce));

            if (res) {
                DEBUG("ls-gate-device-list: ok, nonce wasn't used before\n");
            } else {
                DEBUG("ls-gate-device-list: rejected, nonce was used before\n");
            }

            return res;
		}
	}

    DEBUG("ls-gate-device-list: no node in the list\n");
	return true;
}

bool ls_devlist_is_added(ls_gate_devices_t *devlist, uint64_t node_id) {
    DEBUG("ls-gate-device-list: check if device is in the list\n");
	for (uint32_t i = 0; i < LS_GATE_MAX_NODES; i++) {
		/* Skip free cells */
		if (devlist->nodes_free_list[i])
			continue;

		ls_gate_node_t *node = &devlist->nodes[i];

		if (node->node_id == node_id) {
            DEBUG("ls-gate-device-list: device found\n");
			return true;
        }
	}
    DEBUG("ls-gate-device-list: device not found\n");
	return false;
}

bool ls_devlist_is_in_network(ls_gate_devices_t *devlist, ls_addr_t addr) {
    DEBUG("ls-gate-device-list: checking if device is in the list\n");
	if (addr >= LS_GATE_MAX_NODES) {
        DEBUG("ls-gate-device-list: invalid address\n");
		return false;
    }
    
#if ENABLE_DEBUG
    if (!devlist->nodes_free_list[addr]) {
        DEBUG("ls-gate-device-list: device is in the list\n");
    } else {
        DEBUG("ls-gate-device-list: device is not in the list\n");
    }
#endif

	return !devlist->nodes_free_list[addr];
}

bool ls_devlist_remove_device(ls_gate_devices_t *devlist, ls_addr_t addr) {
    DEBUG("ls-gate-device-list: removing device from the list\n");
	if (addr >= LS_GATE_MAX_NODES) {
        DEBUG("ls-gate-device-list: invalid address\n");
		return false;
    }

	if (devlist->nodes_free_list[addr]) {
        DEBUG("ls-gate-device-list: device already removed\n");
		return false;
    }

	mutex_lock(&devlist->mutex);

	/* Remove all tracked nonces from memory */
	clear_nonce_list(devlist, addr);

	/* Mark cell as free */
	devlist->nodes_free_list[addr] = true;

	/* Decrease counter */
	devlist->num_nodes--;

	mutex_unlock(&devlist->mutex);
    
    DEBUG("ls-gate-device-list: device removed\n");

	return true;
}

bool ls_devlist_is_full(ls_gate_devices_t *devlist) {
	return (devlist->num_nodes >= LS_GATE_MAX_NODES);
}

ls_gate_node_t *ls_devlist_get_by_nodeid(ls_gate_devices_t *devlist, uint64_t nodeid) {
	for (uint32_t i = 0; i < LS_GATE_MAX_NODES; i++) {
		/* Skip free cells */
		if (devlist->nodes_free_list[i])
			continue;

		ls_gate_node_t *node = &devlist->nodes[i];

		if (node->node_id == nodeid)
			return &devlist->nodes[i];
	}

	return NULL;
}

ls_gate_node_t *ls_devlist_get(ls_gate_devices_t *devlist, ls_addr_t addr) {
	if (addr >= LS_GATE_MAX_NODES)
		return false;

	if (!ls_devlist_is_in_network(devlist, addr))
			return NULL;

	return &devlist->nodes[addr];
}

#ifdef __cplusplus
}
#endif
