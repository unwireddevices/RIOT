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

void ls_devlist_init(ls_gate_devices_t *devlist) {
	memset(devlist, 0, sizeof(ls_gate_devices_t));

	for(int i = 0; i < LS_GATE_MAX_NODES; i++)
		devlist->nodes_free_list[i] = true;

	mutex_init(&devlist->mutex);
}

/**
 * @brief Clears tracked nonces list
 */
static void clear_nonce_list(ls_gate_devices_t *devlist, ls_addr_t addr) {
	if (addr >= LS_GATE_MAX_NODES)
		return;

	ls_gate_node_t *node = &devlist->nodes[addr];
	ls_gate_node_nl_entry_t* curr = node->nonce_list;
	ls_gate_node_nl_entry_t* next;
	while (curr != NULL) {
		next = curr->next;

		free(curr);

		curr = next;
	}

	node->num_nonces = 0;
	node->nonce_list = NULL;
}

ls_gate_node_t *add_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce) {
	/* Look for a free cell (and address) to insert */
	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (!devlist->nodes_free_list[i]) {
			ls_gate_node_t *node = &devlist->nodes[i];

			if (node->node_id == node_id) {
				/* Clear nonces list if it's full */
				if (node->num_nonces == LS_GATE_MAX_NONCES) {
					clear_nonce_list(devlist, i);
				}

				node->last_nonce = nonce;

				/* Add current nonce to nonce list */
				ls_gate_node_nl_entry_t *e = malloc(sizeof(ls_gate_node_nl_entry_t));
				e->nonce = nonce;
				e->next = node->nonce_list;
				node->nonce_list = e;
				node->num_nonces++;

				return node;
			}
		}
	}

	return NULL;
}

static void init_node(ls_gate_devices_t *devlist, ls_gate_node_t *node, ls_addr_t addr, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
	node->node_ch = ch;

	node->node_id = node_id;
	node->app_id = app_id;
	node->addr = addr;
	node->is_static = false;

	node->last_nonce = nonce;

	/* Clear nonces list if it's full */
	if (node->num_nonces >= LS_GATE_MAX_NONCES) {
		puts("gate: nonces list cleared"); // XXX: debug
		clear_nonce_list(devlist, addr);
	}

	/* Append nonce to the nonce list */
	ls_gate_node_nl_entry_t *e = malloc(sizeof(ls_gate_node_nl_entry_t));
	e->nonce = nonce;
	e->next = node->nonce_list;
	node->nonce_list = e;

	node->last_fid = 0;
	node->num_pending = 0;
}

ls_gate_node_t *ls_devlist_add_by_addr(ls_gate_devices_t *devlist, ls_addr_t addr, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
	/* List is full? */
	if(ls_devlist_is_full(devlist))
		return NULL;

	/* Device is already added? */
	if (ls_devlist_is_added(devlist, node_id))
		return NULL;

	if (addr >= LS_GATE_MAX_NODES)
		return NULL;

	/* This network address is occupied */
	if (!devlist->nodes_free_list[addr])
		return NULL;

	mutex_lock(&devlist->mutex);

	/* Occupy node record */
	devlist->nodes_free_list[addr] = false;

	/* Fill node record */
	ls_gate_node_t *node = &devlist->nodes[addr];
	init_node(devlist, node, addr, node_id, app_id, nonce, ch);

	node->last_fid = 255;
	node->app_nonce = 0;
	node->is_static = true;

	/* Increase number of connected devices */
	devlist->num_nodes++;

	mutex_unlock(&devlist->mutex);

	return node;
}

ls_gate_node_t *ls_devlist_add(ls_gate_devices_t *devlist, uint64_t node_id, uint64_t app_id, uint32_t nonce, void *ch) {
	/* List is full? */
	if(ls_devlist_is_full(devlist))
		return NULL;

	/* Device is already added? */
	if (ls_devlist_is_added(devlist, node_id))
		return NULL;

	mutex_lock(&devlist->mutex);

	/* Look for a free cell (and address) to insert */
	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
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
			return node;
		}
	}

	mutex_unlock(&devlist->mutex);

	return NULL;
}

bool ls_devlist_check_nonce(ls_gate_devices_t *devlist, uint64_t node_id, uint32_t nonce) {
	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (devlist->nodes_free_list[i])
			continue;

		ls_gate_node_t *node = &devlist->nodes[i];

		if (node->node_id == node_id) {
			/* Nonce repeated, reject */
			if(node->last_nonce == nonce)
				return false;

			/* No nonce remembered, reject */
			if (node->nonce_list == NULL)
				return false;

			/* Iterate through remembered nonce list */
			ls_gate_node_nl_entry_t* curr = node->nonce_list;
			while (curr != NULL) {
				if (curr->nonce == nonce)
					return false;

				curr = curr->next;
			}
		}
	}

	return true;
}

bool ls_devlist_is_added(ls_gate_devices_t *devlist, uint64_t node_id) {
	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
		/* Skip free cells */
		if (devlist->nodes_free_list[i])
			continue;

		ls_gate_node_t *node = &devlist->nodes[i];

		if (node->node_id == node_id)
			return true;
	}

	return false;
}

bool ls_devlist_is_in_network(ls_gate_devices_t *devlist, ls_addr_t addr) {
	if (addr >= LS_GATE_MAX_NODES)
		return false;

	return !devlist->nodes_free_list[addr];
}

bool ls_devlist_remove_device(ls_gate_devices_t *devlist, ls_addr_t addr) {
	if (addr >= LS_GATE_MAX_NODES)
		return false;

	if (devlist->nodes_free_list[addr])
		return false;

	mutex_lock(&devlist->mutex);

	/* Remove all tracked nonces from memory */
	clear_nonce_list(devlist, addr);

	/* Mark cell as free */
	devlist->nodes_free_list[addr] = true;

	/* Decrease counter */
	devlist->num_nodes--;

	mutex_unlock(&devlist->mutex);

	return true;
}

bool ls_devlist_is_full(ls_gate_devices_t *devlist) {
	return (devlist->num_nodes >= LS_GATE_MAX_NODES);
}

ls_gate_node_t *ls_devlist_get_by_nodeid(ls_gate_devices_t *devlist, uint64_t nodeid) {
	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
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
