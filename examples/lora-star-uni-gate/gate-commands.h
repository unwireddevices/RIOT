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
#ifndef GATE_COMMANDS_H_
#define GATE_COMMANDS_H_

#include "ls-gate.h"
#include "pending-fifo.h"

typedef enum {
	CMD_PING = 'P',			/* Command to ping/pong with client */
	CMD_DEVLIST = 'L',		/* Command to get devices list from a gate */
	CMD_IND = 'I',			/* Individual command to the mote by address */
	CMD_HAS_PENDING = '?',	/* Individual device has N pending packets */
	CMD_INVITE = 'V',		/* Individual invite to join network for class C devices */
	CMD_BROADCAST = 'B',	/* Broadcast message */

	CMD_FLUSH = 'F',		/* Command to get all pending info */
} gate_cmd_type_t;

typedef enum {
	REPLY_PONG = '!',		/* Reply for ping command from client */
	REPLY_LIST = 'L',		/* Reply for the device list command */
	REPLY_IND = 'I',		/* Reply from the individual mote */

	REPLY_JOIN = 'J',		/* Node is joined to the network */
	REPLY_KICK = 'K',		/* Node is kicked from the network */

	REPLY_ACK = 'A',		/* Application data acknowledged by the node */

	REPLY_PENDING_REQ = 'R', /* Gate requesting pending frames from upper layer */
} gate_reply_type_t;

void gc_parse_command(ls_gate_t *ls, kernel_pid_t writer, gc_pending_fifo_t *fifo, char *cmd);

#endif /* GATE_COMMANDS_H_ */
