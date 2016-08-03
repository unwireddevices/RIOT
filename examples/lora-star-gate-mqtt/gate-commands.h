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
	CMD_DEVLIST = '1',	/* Command to get devices list from a gate */
	CMD_IND = '2',		/* Individual command to the mote by address */

	CMD_GIMME = '0',	/* Flush pending replies fifo to the gate */
} mqtt_cmd_type_t;

typedef enum {
	REPLY_MQTT = '1',		/* Message to the mqtt */
	REPLY_IND = '2',		/* Reply from the individual mote */
} mqtt_reply_type_t;

void gc_parse_command(ls_gate_t *ls, kernel_pid_t writer, gc_pending_fifo_t *fifo, char *cmd);

#endif /* EXAMPLES_LORA_STAR_GATE_MQTT_GATE_COMMANDS_H_ */
