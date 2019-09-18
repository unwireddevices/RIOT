/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "unwds-udp.h"
#include <stdio.h>

#include "shell.h"
#include "msg.h"
#include "net/gnrc/netif.h"

#include "crypto/aes.h"

#define ENABLE_DEBUG		(0)
#include "debug.h"
#include "od.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "unwds_udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};


int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT Unwired Devices mesh network.");
	
	if(root_node_init() < 0)
    {
        puts("Error ROOT node init");
        return -1;
    }

	puts("Unwired Devices mesh network initialization is over\n");
	
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	
    /* should be never reached */
    return 0;
}
