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

#include "ls-mac-types.h"
#include "ls-gate-device-list.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief Initialize list of connected nodes
 */
void ls_devlist_init(ls_gate_devices_t *devlist) {
	memset(devlist, 0, sizeof(ls_gate_devices_t));

	for(int i = 0; i < LS_GATE_MAX_NODES; i++) {
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
    
    memset((void *)node->nonce, 0, sizeof(ls_nonce_t) * LS_GATE_NONCES_PER_DEVICE);
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

				/* Add current nonce to nonce list */
                for (uint32_t j = 0; j < LS_GATE_NONCES_PER_DEVICE; j++) {
                    if (node->nonce[j] == 0) {
                        node->nonce[j] = nonce;
                        node->num_nonces++;
                        DEBUG("ls-gate-device-list: nonce successfully added\n");
                        break;
                    }
                }
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

	/* Append nonce to the nonce list */
    for (uint32_t j = 0; j < LS_GATE_NONCES_PER_DEVICE; j++) {
        if (node->nonce[j] == 0) {
            node->nonce[j] = nonce;
            node->num_nonces++;
            DEBUG("ls-gate-device-list: nonce successfully added\n");
            break;
        }
    }
    
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

	node->num_nonces = 1;
	node->nonce[0] = nonce;

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
			/* Iterate through remembered nonce list */
            for (uint32_t k = 0; k < LS_GATE_NONCES_PER_DEVICE; k++) {
                if (node->nonce[k] == 0) {
                    DEBUG("ls-gate-device-list: end of nonce list\n");
                    break;
                }
                
                if (node->nonce[k] == nonce) {
                    DEBUG("ls-gate-device-list: nonce value was used before\n");
					return false;
                }
            }
            break;
		}
	}
    DEBUG("ls-gate-device-list: nonce checked, is ok\n");
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
