/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     test
 * @{
 *
 * @file
 * @brief       minimal RIOT application to test RTC subsecond timers
 *
 * @author      Oleg Artamonov
 *
 * @}
 */

#include <stdio.h>

#include "lpm.h"
#include "periph/rtc.h"
#include "arch/lpm_arch.h"
#include "thread.h"
#include "xtimer.h"
#include "board.h"

#include "rtctimers.h"
#include "rtctimers-millis.h"

static msg_t timer1_msg;
static msg_t timer2_msg;

static kernel_pid_t timer1_pid;
static kernel_pid_t timer2_pid;

static rtctimers_millis_t timer1;
static rtctimers_millis_t timer2;

static volatile uint32_t timer1_prev;
static volatile uint32_t timer2_prev;

#define TIMER1_PERIOD 300
#define TIMER2_PERIOD 800

char stack1[2048];
char stack2[2048];

static void *timer1_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("Timer1 thread started");

    while (1) {
        msg_receive(&msg);
        
        volatile uint32_t now = xtimer_now_usec()/1000;
        printf("Timer1: %lu ms\n", now - timer1_prev);
        timer1_prev = now;
        
        /* 800 ms */
        rtctimers_millis_set_msg(&timer1, TIMER1_PERIOD, &timer1_msg, timer1_pid);
    }
    return NULL;
}

static void *timer2_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("Timer2 thread started");

    while (1) {
        msg_receive(&msg);
        
        volatile uint32_t now = xtimer_now_usec()/1000;
        printf("Timer2: %lu ms\n", now - timer2_prev);
        timer2_prev = now;
        
        /* 300 ms */
        rtctimers_millis_set_msg(&timer2, TIMER2_PERIOD, &timer2_msg, timer2_pid);
    }
    return NULL;
}

int main(void)
{
    puts("rtctimers-millis test");
    
    lpm_prevent_sleep = 1;
    lpm_prevent_switch = 1;

	rtctimers_millis_init();
	xtimer_init();

	lpm_prevent_sleep = 0;
    
    puts("Creating timer1 thread");
    timer1_pid = thread_create(stack1, 2048, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer1_thread, NULL, "Timer 1");
    
    puts("Creating timer2 thread");
    timer2_pid = thread_create(stack2, 2048, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer2_thread, NULL, "Timer 2");
    
    puts("Threads created");
    
    uint32_t now = xtimer_now_usec()/1000;
    timer1_prev = now;
    timer2_prev = now;
    
    puts("Sending messages");
    
    /* 800 ms */
    rtctimers_millis_set_msg(&timer1, TIMER1_PERIOD, &timer1_msg, timer1_pid);
    /* 300 ms */
    rtctimers_millis_set_msg(&timer2, TIMER2_PERIOD, &timer2_msg, timer2_pid);
    
    while (1) {
        
    };
    
    puts("Done.");

    return 0;
}
