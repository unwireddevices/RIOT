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
 * @file		gate-commands.h
 * @brief       definitions for the gate commands
 * @author      Eugene Ponomarev
 */
#ifndef EXAMPLES_LORA_STAR_GATE_MQTT_GATE_COMMANDS_H_
#define EXAMPLES_LORA_STAR_GATE_MQTT_GATE_COMMANDS_H_

#include "ls-gate.h"
#include "pending-fifo.h"

typedef enum {
	CMD_DEVLIST = '1',		/* Command to get devices list from a gate */
	CMD_IND = '2',			/* Individual command to the mote by address */
	CMD_HAS_PENDING = '3',	/* Individual device has N pending packets */

	CMD_GIMME = '0',		/* Command to get all pending info */
} mqtt_cmd_type_t;

typedef enum {
	REPLY_MQTT = '1',	/* Reply for the device list command */
	REPLY_IND = '2',	/* Reply from the individual mote */

	REPLY_JOIN = '3',	/* Node is joined to the network */
	REPLY_KICK = '4',	/* Node is kicked from the network */

	REPLY_ACK = '5',	/* Application data acknowledged by the node */
	REPLY_LNKCHK = '6',	/* Link check from the node */
} mqtt_reply_type_t;

void gc_parse_command(ls_gate_t *ls, kernel_pid_t writer, gc_pending_fifo_t *fifo, char *cmd);

#endif /* EXAMPLES_LORA_STAR_GATE_MQTT_GATE_COMMANDS_H_ */
