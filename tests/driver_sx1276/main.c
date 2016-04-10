/*
 * Copyright (C) 2016 Cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for SX1276 modem driver
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>

#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"

#include "common.h"

#define _STACKSIZE      (THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF)
#define MSG_TYPE_ISR    (0x3456)

static char stack[_STACKSIZE];
static kernel_pid_t _recv_pid;

void test1(int argc, char **argv) {
	puts("test 1 called");
}

void test2(int argc, char **argv) {
	puts("test 2 called");
}

static const shell_command_t shell_commands[] = {
    { "test1", "Test 1", test1 },
    { "test2", "Test 2", test2 },
    { NULL, NULL, NULL }
};

static void _event_cb(netdev2_t *dev, netdev2_event_t event, void *data)
{
    (void) data;
}

void *_recv_thread(void *arg)
{
}

int main(void)
{
    puts("SX1276 device driver test");
    xtimer_init();

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
