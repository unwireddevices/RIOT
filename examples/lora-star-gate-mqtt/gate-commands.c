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
 * @file		gate-commands.c
 * @brief       gate mqtt-uart commands implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pending-fifo.h"
#include "gate-commands.h"
#include "ls-gate.h"

static void exec_command(ls_gate_t *ls, kernel_pid_t writer, gc_pending_fifo_t *fifo, char cmd, char *type, char *addr, char *msg) {
	ls_gate_devices_t *devs = &ls->devices;

	mqtt_cmd_type_t c = cmd;

	switch (c) {
	case CMD_DEVLIST:
		for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
			if (!devs->nodes_free_list[i]) {
				char buf[64];

				/* mqtt|topic|message */
				sprintf(buf, "%c|%u|{app_id:0x%X,ability:0x%X,last_seen:%d}\n", REPLY_MQTT,
						(unsigned int) devs->nodes[i].node_id,
						(unsigned int) devs->nodes[i].app_id,
						(unsigned int) devs->nodes[i].node_ability,
						(unsigned int) ((ls->_internal.ping_count - devs->nodes[i].last_seen) * LS_PING_TIMEOUT_S));

				if (!gc_pending_fifo_push(fifo, buf)) {
					puts("gc: pending fifo overflowed!");
				}
			}
		}

		break;

	case CMD_IND:;
		uint64_t nodeid = strtol(addr, NULL, 10);
		ls_gate_node_t *node = ls_devlist_get_by_nodeid(devs, nodeid);
		if (node == NULL)
			return;

		/* type:msg */
		size_t len = strlen(type) + strlen(msg);
		char *buf = malloc(len + 1);
		memset(buf, 0, len + 1);
		sprintf(buf, "%s,%s", type, msg);
		buf[len] = '\0'; /* Embed trailing zero terminator */

		/* Can send immediately */
		if (node->node_class == LS_ED_CLASS_B) {
			ls_gate_send_to(ls, node->addr, (uint8_t *) buf, len + 1);
		}

		break;

	case CMD_GIMME:;
		msg_t msg;
		msg_send(&msg, writer);

		break;
	}
}

void gc_parse_command(ls_gate_t *ls, kernel_pid_t writer, gc_pending_fifo_t *fifo, char *cmd) {
	char argv[4][15] = { { '\0', }, };
	int maxlen[4] = { 1, 15, 15, 15 };
	int argc = 0, j = 0;
	int len = strlen(cmd);

	/* Tokenize string in tokens, splitted by '|' character */
	for (int i = 0; i < len; i++) {
		if (cmd[i] == '|') {
			argc++;
			j = 0;
		} else {
			if (j > maxlen[argc])
				continue;

			argv[argc][j++] = cmd[i];
		}
	}

	if (argc == 0) {
		char cmd = argv[0][0];

		exec_command(ls, writer, fifo, cmd, NULL, NULL, NULL);
	} else if (argc == 3) {
		char cmd = argv[0][0];
		char *type = argv[1];
		char *addr = argv[2];
		char *msg = argv[3];

		printf("cmd: %c\ntype: %s\naddr: %s\nmsg: %s\n", cmd, type, addr, msg);

		exec_command(ls, writer, fifo, cmd, type, addr, msg);
	}
}

#ifdef __cplusplus
}
#endif
